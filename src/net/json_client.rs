// JSON client handler
// Manages individual JSON client connections

use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};
use tokio::time;
use std::io;
use std::sync::Arc;
use rand::Rng;

/// Count of message prefixes we've logged (for "first few messages" diagnostic).
static LOGGED_MESSAGE_PREFIXES: AtomicUsize = AtomicUsize::new(0);
/// Count of read errors (early eof, etc.); we rate-limit to avoid flooding during shutdown.
static READ_ERROR_COUNT: AtomicUsize = AtomicUsize::new(0);
/// Count of disconnect logs; we rate-limit to avoid flooding.
static DISCONNECT_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);

use flate2::{Compress, Compression, Decompress, FlushCompress, FlushDecompress};

use super::connection::Connection;
use super::messages::{server_message_to_wire, ClientMessage, HandshakeRequest, ServerMessage, SyncPayload};

/// Python util.fuzzy(t): random value in [0.9*t, 1.1*t] rounded to integer. Used for reconnect_in.
fn fuzzy(t: f64) -> u32 {
    let r = rand::thread_rng().gen_range(0.9 * t..=1.1 * t);
    r.round() as u32
}

/// State of a JSON client connection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientState {
    /// Waiting for initial handshake
    AwaitingHandshake,
    /// Authenticated and ready to receive messages
    Authenticated,
    /// Connection closed
    Closed,
}

/// Optional UDP (host, port) for this listener. When set, handshake includes udp_transport so client sends sync/mlat over UDP.
pub type OptionalUdp = Option<(String, u16)>;

/// Compression negotiated with client (Python: zlib2, zlib, or none).
fn negotiate_compress(client_compress: &[String]) -> Option<&'static str> {
    let set: std::collections::HashSet<_> = client_compress.iter().map(|s| s.as_str()).collect();
    for &candidate in &["zlib2", "zlib", "none"] {
        if set.contains(candidate) {
            return Some(candidate);
        }
    }
    None
}

/// JSON client handler
pub struct JsonClient {
    connection: Connection,
    state: ClientState,
    receiver_id: Option<usize>,
    last_message_time: Instant,
    motd: String,
    coordinator: Arc<crate::coordinator::Coordinator>,
    /// If Some(host, port), handshake response will include udp_transport (host, port, key).
    optional_udp: OptionalUdp,
    /// After handshake: true if client sends zlib/zlib2 compressed packets (Python handle_zlib_messages).
    use_zlib: bool,
    /// When true, server→client must be sent as zlib2 frames (Python: write_zlib). When false, use write_json.
    use_zlib2_send: bool,
    /// One decompressor per connection when use_zlib (Python: single _decompressor in handle_zlib_messages).
    zlib_decompressor: Option<Decompress>,
    /// One compressor per connection when use_zlib2_send (Python: _compressor in _flush_zlib).
    zlib_compressor: Option<Compress>,
    /// Partial line across decompressed chunks (Python: linebuf).
    zlib_line_buf: String,
}

impl JsonClient {
    /// Create a new JSON client. If optional_udp is Some(host, port), handshake will include udp_transport so client can send sync/mlat over UDP.
    pub fn new(
        connection: Connection,
        motd: String,
        coordinator: Arc<crate::coordinator::Coordinator>,
        optional_udp: OptionalUdp,
    ) -> Self {
        JsonClient {
            connection,
            state: ClientState::AwaitingHandshake,
            receiver_id: None,
            last_message_time: Instant::now(),
            motd,
            coordinator,
            optional_udp,
            use_zlib: false,
            use_zlib2_send: false,
            zlib_decompressor: None,
            zlib_compressor: None,
            zlib_line_buf: String::new(),
        }
    }
    
    /// Get the current state
    pub fn state(&self) -> ClientState {
        self.state
    }
    
    /// Get the receiver ID (if authenticated)
    pub fn receiver_id(&self) -> Option<usize> {
        self.receiver_id
    }
    
    /// Run the client handler loop
    ///
    /// This processes the handshake, then enters the message loop.
    /// Returns when the connection is closed or an error occurs.
    pub async fn run(&mut self) -> io::Result<()> {
        // Create a channel for receiving messages from the coordinator
        let (tx, mut rx) = tokio::sync::mpsc::channel(100);
        
        // Process handshake first
        if !self.process_handshake().await? {
            return Ok(());  // Handshake failed, connection closed
        }
        
        // Register client in coordinator
        if let Some(receiver_id) = self.receiver_id {
            self.coordinator.register_client(receiver_id, tx).await;
        }
        
        // Heartbeat and timeout constants (Python: write_heartbeat_interval=30, read_heartbeat_interval=150)
        let heartbeat_interval = Duration::from_secs(crate::constants::HEARTBEAT_INTERVAL_SECS);
        let read_timeout = Duration::from_secs(crate::constants::READ_TIMEOUT_SECS);
        
        // Persistent heartbeat timer (Python uses call_later callback, not recreated each loop)
        let mut heartbeat_timer = time::interval(heartbeat_interval);
        heartbeat_timer.tick().await; // skip the immediate first tick
        
        let result = loop {
            // Check for read timeout (match Python: "No recent messages seen, closing connection")
            if self.last_message_time.elapsed() > read_timeout {
                eprintln!("No recent messages seen, closing connection");
                self.state = ClientState::Closed;
                break Ok(());
            }
            
            // Select between: reading client data, coordinator messages, and heartbeat timer.
            // The heartbeat timer is persistent (tokio::time::interval) so it fires reliably
            // regardless of how long message processing takes. Coordinator calls inside
            // handle_message are spawned as separate tasks to avoid blocking this loop.
            tokio::select! {
                result = self.read_next_client_data() => {
                    match result {
                        Ok(None) => {
                            // EOF (match Python: "Client EOF")
                            self.state = ClientState::Closed;
                            break Ok(());
                        }
                        Ok(Some(())) => {
                            self.last_message_time = Instant::now();
                        }
                        Err(e) => {
                            let n = READ_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
                            if n < 3 {
                                eprintln!("Read error: {}", e);
                            } else if n == 3 {
                                eprintln!("Read error: {} (further read errors suppressed until next run)", e);
                            }
                            self.state = ClientState::Closed;
                            break Err(e);
                        }
                    }
                }
                Some(srv_msg) = rx.recv() => {
                    // Send flat JSON (match Python wire format; no "type" field). Use zlib2 when negotiated.
                    for json in server_message_to_wire(&srv_msg) {
                        self.send_json_line(&json).await?;
                    }
                }
                _ = heartbeat_timer.tick() => {
                    // Send heartbeat on persistent timer (fires every 30s regardless of read activity)
                    self.send_heartbeat().await?;
                }
            }
        };

        // Unregister, disconnect from coordinator, and log (match Python: "Disconnected: ({conn_info})"); rate-limit to avoid flood during shutdown.
        if let Some(receiver_id) = self.receiver_id {
            if let Some(conn_info) = self.coordinator.get_receiver_connection_info(receiver_id).await {
                let n = DISCONNECT_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
                if n < 5 {
                    eprintln!("Disconnected: ({})", conn_info);
                } else if n == 5 {
                    eprintln!("Disconnected: (further disconnects suppressed)");
                }
            }
            self.coordinator.unregister_client(receiver_id).await;
            self.coordinator.receiver_disconnect(receiver_id).await;
        }

        result
    }

    /// Read next client input: one line (none) or one zlib packet (zlib/zlib2). Returns None on EOF, Some(()) when data was read (and processed).
    async fn read_next_client_data(&mut self) -> io::Result<Option<()>> {
        if self.use_zlib {
            // Python handle_zlib_messages: one decompressor per connection; 2-byte BE length, then payload; Z_SYNC_FLUSH suffix 0x00 0x00 0xff 0xff
            let mut len_buf = [0u8; 2];
            self.connection.read_exact(&mut len_buf).await?;
            let len = u16::from_be_bytes(len_buf) as usize;
            if len > 65535 {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "zlib packet length too large"));
            }
            let mut payload = vec![0u8; len];
            self.connection.read_exact(&mut payload).await?;
            // Empty packet: skip (some clients send len=0); do not feed to decompressor
            if payload.is_empty() {
                return Ok(Some(()));
            }
            // Python server removes sync bytes before sending; some clients may include them. Append only if not already present.
            const SYNC_SUFFIX: &[u8; 4] = &[0x00, 0x00, 0xff, 0xff];
            if payload.len() < 4 || payload[payload.len() - 4..] != SYNC_SUFFIX[..] {
                payload.extend_from_slice(SYNC_SUFFIX);
            }
            let decomp = self.zlib_decompressor.get_or_insert_with(|| Decompress::new(true));
            let mut input = payload.as_slice();
            const OUT_BUF_SIZE: usize = 65536;
            let mut out_buf = [0u8; OUT_BUF_SIZE];
            let mut lines_to_process: Vec<String> = Vec::new();
            let mut retried = false;
            loop {
                let before_in = decomp.total_in();
                let before_out = decomp.total_out();
                match decomp.decompress(input, &mut out_buf, FlushDecompress::None) {
                    Ok(_) => {}
                    Err(e) => {
                        // First packet only: some clients send a complete zlib stream per packet; retry once with fresh decompressor.
                        if !retried && before_in == 0 {
                            retried = true;
                            decomp.reset(true);
                            input = payload.as_slice();
                            continue;
                        }
                        return Err(io::Error::new(io::ErrorKind::InvalidData, e));
                    }
                }
                let consumed = (decomp.total_in() - before_in) as usize;
                let produced = (decomp.total_out() - before_out) as usize;
                if consumed == 0 && produced == 0 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "Decompressor made no progress",
                    ));
                }
                input = &input[consumed..];
                let s = std::str::from_utf8(&out_buf[..produced]).map_err(|_| {
                    io::Error::new(io::ErrorKind::InvalidData, "invalid UTF-8 in decompressed stream")
                })?;
                self.zlib_line_buf.push_str(s);
                let parts: Vec<&str> = self.zlib_line_buf.split('\n').collect();
                let (complete, last) = if parts.len() <= 1 {
                    (&parts[..0], self.zlib_line_buf.as_str())
                } else {
                    let last_idx = parts.len() - 1;
                    (&parts[..last_idx], parts[last_idx])
                };
                for line in complete {
                    let line = line.trim_end_matches('\r');
                    if !line.is_empty() {
                        lines_to_process.push(line.to_string());
                    }
                }
                self.zlib_line_buf = last.to_string();
                if self.zlib_line_buf.len() > 1024 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        "Client sent a very long line",
                    ));
                }
                if input.is_empty() {
                    break;
                }
                // Python: mitigate DoS with highly compressible data (await asyncio.sleep(0.1))
                let dos_sleep_ms = (crate::constants::ZLIB_DOS_SLEEP_SECS * 1000.0) as u64;
                time::sleep(Duration::from_millis(dos_sleep_ms)).await;
            }
            if !self.zlib_line_buf.is_empty() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Client sent a packet that was not newline terminated",
                ));
            }
            for line in lines_to_process {
                self.handle_message(&line).await?;
            }
            Ok(Some(()))
        } else {
            let line = self.connection.read_line().await?;
            if line.is_empty() {
                return Ok(None);
            }
            self.handle_message(&line).await?;
            Ok(Some(()))
        }
    }
    
    /// Process the initial handshake
    ///
    /// Returns true if handshake succeeded, false if it failed.
    /// Accepts flat handshake format (no "type" field) as sent by mlat-client and test client.
    /// If the first line starts with "PROXY " (HAProxy etc.), parses client IP/port and uses the next line as handshake (Python parity).
    async fn process_handshake(&mut self) -> io::Result<bool> {
        // Read first line with timeout (Python: wait_for(readline(), timeout=15.0))
        let mut line = match time::timeout(
            Duration::from_secs(crate::constants::HANDSHAKE_TIMEOUT_SECS),
            self.connection.read_line()
        ).await {
            Ok(Ok(l)) => l,
            Ok(Err(e)) => return Err(e),
            Err(_) => {
                eprintln!("Handshake failed: timeout");
                return Ok(false);
            }
        };

        // Python: if line.startswith('PROXY '), parse source IP/port and read next line as handshake
        let proxy_source: Option<(String, u16)> = if line.starts_with("PROXY ") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() >= 5 {
                let ip = parts[2].to_string();
                if let Ok(port) = parts[4].parse::<u16>() {
                    let next_line = match time::timeout(
                        Duration::from_secs(crate::constants::HANDSHAKE_TIMEOUT_SECS),
                        self.connection.read_line()
                    ).await {
                        Ok(Ok(l)) => l,
                        Ok(Err(e)) => return Err(e),
                        Err(_) => {
                            eprintln!("Handshake failed: timeout after PROXY");
                            return Ok(false);
                        }
                    };
                    line = next_line;
                    Some((ip, port))
                } else {
                    None
                }
            } else {
                None
            }
        } else {
            None
        };

        self.coordinator.log_handshake(&line).await;

        let handshake: HandshakeRequest = match serde_json::from_str(&line) {
            Ok(h) => h,
            Err(_e) => {
                eprintln!("Handshake failed: Invalid handshake format");
                self.send_handshake_error("Invalid handshake format").await?;
                return Ok(false);
            }
        };

        let HandshakeRequest { version, user, lat, lon, alt, clock_type, uuid, privacy, compress: client_compress, .. } = handshake;

        // Negotiate compression (Python: first of zlib2, zlib, none that client supports)
        let compress = match negotiate_compress(&client_compress) {
            Some(c) => c,
            None => {
                self.send_handshake_error("No mutually usable compression type").await?;
                return Ok(false);
            }
        };
        self.use_zlib = compress == "zlib" || compress == "zlib2";
        self.use_zlib2_send = compress == "zlib2";
        if self.use_zlib2_send {
            self.zlib_compressor = Some(Compress::new(Compression::new(1), true));
        }

        // Validate version
        if version != 2 && version != 3 {
            self.send_handshake_error("Unsupported protocol version").await?;
            return Ok(false);
        }

        // Sanitize user (Python: re.sub(r"[^A-Za-z0-9_.-]", "_", user)[:40], then ensure len >= 3)
        let user: String = user
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() || "_.-".contains(c) { c } else { '_' })
            .collect::<String>()
            .chars()
            .take(40)
            .collect();
        let user = if user.len() < 3 {
            let suffix = 10 + (std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis() % 90) as u32;
            format!("{}_{}", user, suffix)
        } else {
            user
        };

        // Validate coordinates
        if lat < -90.0 || lat > 90.0 {
            self.send_handshake_error("Invalid latitude").await?;
            return Ok(false);
        }

        if lon < -180.0 || lon > 360.0 {
            self.send_handshake_error("Invalid longitude").await?;
            return Ok(false);
        }

        if alt < -1000.0 || alt > 10000.0 {
            self.send_handshake_error("Invalid altitude").await?;
            return Ok(false);
        }

        let clock_str = clock_type.as_deref().unwrap_or("dump1090");
        // client_wants_udp must be computed before connection_info (Python: udp="udp" if self.use_udp else "tcp")
        let client_wants_udp = handshake.udp_transport.unwrap_or(0) == 2;
        let transport = if client_wants_udp && self.optional_udp.is_some() {
            "udp"
        } else {
            "tcp"
        };
        let connection_info = format!("{} v{} {} unknown {} {}", user, version, clock_str, transport, compress);

        // Create receiver via coordinator (enforces one connection per user)
        let return_stats = handshake.return_stats.unwrap_or(false);
        let receiver_id = match self.coordinator.new_receiver(
            user.clone(),
            lat,
            lon,
            alt,
            clock_str,
            uuid.clone(),
            privacy,
            connection_info.clone(),
            return_stats,
        ).await {
            Ok(id) => id,
            Err(e) => {
                self.send_handshake_error(&e).await?;
                return Ok(false);
            }
        };

        self.receiver_id = Some(receiver_id);
        self.connection.set_receiver_id(receiver_id);
        let (source_ip, source_port) = proxy_source
            .unwrap_or_else(|| {
                let peer = self.connection.peer_addr();
                (peer.ip().to_string(), peer.port())
            });
        self.coordinator.set_receiver_source(receiver_id, source_ip, source_port).await;

        eprintln!("Handshake successful ({})", connection_info);

        // Only advertise UDP when server has UDP and client requested it (Python: use_udp = udp_protocol is not None and hs.get('udp_transport', 0) == 2)
        let udp_transport = if client_wants_udp {
            if let Some((ref host, port)) = self.optional_udp {
                let key = rand::thread_rng().gen::<u32>();
                self.coordinator.register_udp_key(key, receiver_id).await;
                Some((host.clone(), port, key))
            } else {
                None
            }
        } else {
            None
        };

        // Send handshake response as flat JSON (no "type" field) to match Python exactly.
        // Some clients only start sending sync/mlat after seeing this exact shape.
        let mut response = serde_json::json!({
            "compress": compress,
            "reconnect_in": fuzzy(10.0),
            "selective_traffic": true,
            "heartbeat": true,
            "return_results": true,
            "return_stats": handshake.return_stats.unwrap_or(false),
            "rate_reports": true,
            "motd": self.motd,
        });
        if let Some((host, port, key)) = udp_transport {
            response["udp_transport"] = serde_json::json!([host, port, key]);
        }
        self.connection.write_json(&response).await?;

        self.state = ClientState::Authenticated;
        self.last_message_time = Instant::now();

        Ok(true)
    }
    
    /// Send a handshake error message (Python: reconnect_in=util.fuzzy(900) => 810-990)
    async fn send_handshake_error(&mut self, error: &str) -> io::Result<()> {
        let msg = serde_json::json!({
            "deny": [error],
            "reconnect_in": fuzzy(900.0)
        });
        self.connection.write_json(&msg).await
    }

    /// Send one JSON object to the client. When use_zlib2_send, encodes as zlib2 frame (length + zlib payload without 4-byte suffix).
    async fn send_json_line(&mut self, json: &serde_json::Value) -> io::Result<()> {
        let line = serde_json::to_string(json).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let line_bytes = format!("{}\n", line).into_bytes();
        if self.use_zlib2_send {
            let compressor = self.zlib_compressor.as_mut().expect("zlib2_send set without compressor");
            // Python: single stateful _compressor across all messages; Z_SYNC_FLUSH creates
            // flush points within the continuous stream. Do NOT reset() — that would start a
            // new zlib stream and break the client's decompressor.
            // NOTE: reset() was previously here but is WRONG; see Python _flush_zlib which
            // never resets. The client decompressor is also stateful across frames.
            let mut out = Vec::with_capacity(line_bytes.len() * 2 + 32);
            compressor.compress_vec(&line_bytes, &mut out, FlushCompress::Sync).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
            // Python _flush_zlib: strip Z_SYNC_FLUSH 4-byte suffix (0x00 0x00 0xff 0xff)
            if out.len() >= 4 {
                out.truncate(out.len() - 4);
            }
            self.connection.write_zlib2_frame(&out).await?;
        } else {
            self.connection.write_json(json).await?;
        }
        Ok(())
    }

    /// Handle a message from the client.
    /// Accepts both tag format {"type": "sync", ...} and wrapped format {"sync": {"et", "ot", "em", "om"}}.
    async fn handle_message(&mut self, line: &str) -> io::Result<()> {
        self.coordinator.record_message_received().await;

        // One-time diagnostic: log first 10 message "keys" (first key in JSON) to see what client sends
        let n = LOGGED_MESSAGE_PREFIXES.fetch_add(1, Ordering::Relaxed);
        if n < 10 {
            let key = serde_json::from_str::<serde_json::Value>(line)
                .ok()
                .and_then(|v| v.as_object().and_then(|o| o.keys().next().map(String::from)))
                .unwrap_or_else(|| "parse_fail".to_string());
            eprintln!("[sync-diag] message #{} key: {}", n + 1, key);
        }
        // Diagnostic: count raw lines that look like they contain sync (to distinguish "no sync sent" vs "parse failed")
        let coordinator = self.coordinator.clone();
        let receiver_id = self.receiver_id;

        if line.contains("\"sync\"") {
            static FIRST_SYNC_LINE_LOGGED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
            if !FIRST_SYNC_LINE_LOGGED.swap(true, Ordering::Relaxed) {
                let prefix: String = line.chars().take(200).collect();
                eprintln!("[sync-diag] first line containing \"sync\" (prefix): {}...", prefix);
            }
            let c = coordinator.clone();
            tokio::spawn(async move { c.record_line_containing_sync().await; });
        }
        // Try tag-based format first (ClientMessage)
        if let Ok(msg) = serde_json::from_str::<ClientMessage>(line) {
            return self.dispatch_client_message(msg).await;
        }

        // Try wrapped format (Python/mlat-client: {"sync": {...}} or {"mlat": {...}})
        let value: serde_json::Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("Failed to parse message: {}", e);
                return Ok(());
            }
        };

        let obj = match value.as_object() {
            Some(o) => o,
            None => return Ok(()),
        };

        // Python: message_counter += 1 per JSON message (for message_rate in write_state)
        // Spawn coordinator calls to avoid blocking the heartbeat loop (lock contention with
        // many concurrent clients would cause heartbeat starvation and client disconnects).
        if let Some(rid) = receiver_id {
            let c = coordinator.clone();
            tokio::spawn(async move { c.increment_receiver_message_count(rid).await; });
        }

        if let Some(sync_val) = obj.get("sync") {
            let c = coordinator.clone();
            tokio::spawn(async move { c.record_sync_key_seen().await; });
            match serde_json::from_value::<SyncPayload>(sync_val.clone()) {
                Ok(payload) => {
                    if let Some(rid) = receiver_id {
                        let c = coordinator.clone();
                        let em = payload.em.clone();
                        let om = payload.om.clone();
                        tokio::spawn(async move {
                            c.handle_sync(rid, payload.et, payload.ot, &em, &om).await;
                        });
                    }
                    return Ok(());
                }
                Err(_) => {
                    let c = coordinator.clone();
                    tokio::spawn(async move { c.record_sync_parse_fail().await; });
                }
            }
        }

        if let Some(mlat_val) = obj.get("mlat") {
            if let Some(mlat_obj) = mlat_val.as_object() {
                if let (Some(t), Some(m)) = (mlat_obj.get("t").and_then(|v| v.as_f64()), mlat_obj.get("m").and_then(|v| v.as_str())) {
                    if let Some(rid) = receiver_id {
                        let c = coordinator.clone();
                        let m = m.to_string();
                        tokio::spawn(async move {
                            c.handle_mlat(rid, t, &m, 0.0).await;
                        });
                    }
                    return Ok(());
                }
            }
        }

        if let Some(seen_val) = obj.get("seen") {
            if let Some(arr) = seen_val.as_array() {
                let icaos: Vec<u32> = arr.iter().filter_map(|v| v.as_str().and_then(|s| u32::from_str_radix(s, 16).ok())).collect();
                if let Some(rid) = receiver_id {
                    let c = coordinator.clone();
                    tokio::spawn(async move {
                        c.receiver_tracking_add(rid, &icaos).await;
                    });
                }
                return Ok(());
            }
        }

        if let Some(lost_val) = obj.get("lost") {
            if let Some(arr) = lost_val.as_array() {
                let icaos: Vec<u32> = arr.iter().filter_map(|v| v.as_str().and_then(|s| u32::from_str_radix(s, 16).ok())).collect();
                if let Some(rid) = receiver_id {
                    let c = coordinator.clone();
                    tokio::spawn(async move {
                        c.receiver_tracking_remove(rid, &icaos).await;
                    });
                }
                return Ok(());
            }
        }

        if obj.get("input_connected").is_some() {
            if let Some(rid) = receiver_id {
                let c = coordinator.clone();
                tokio::spawn(async move { c.receiver_clock_reset(rid, true).await; });
            }
            return Ok(());
        }
        if obj.get("input_disconnected").is_some() {
            if let Some(rid) = receiver_id {
                let c = coordinator.clone();
                tokio::spawn(async move { c.receiver_clock_reset(rid, false).await; });
            }
            return Ok(());
        }
        if obj.get("clock_reset").is_some() {
            if let Some(rid) = receiver_id {
                let c = coordinator.clone();
                tokio::spawn(async move { c.receiver_clock_reset(rid, false).await; });
            }
            return Ok(());
        }
        if obj.get("clock_jump").is_some() {
            if let Some(rid) = receiver_id {
                let c = coordinator.clone();
                tokio::spawn(async move { c.receiver_clock_reset(rid, false).await; });
            }
            return Ok(());
        }

        if let Some(rr_val) = obj.get("rate_report") {
            if let Some(rr_obj) = rr_val.as_object() {
                let rates: std::collections::HashMap<u32, f64> = rr_obj.iter()
                    .filter_map(|(k, v)| v.as_f64().and_then(|f| u32::from_str_radix(k, 16).ok().map(|u| (u, f))))
                    .collect();
                if rates.is_empty() && !rr_obj.is_empty() {
                    static RR_DBG: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                    if RR_DBG.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 3 {
                        let sample: Vec<_> = rr_obj.iter().take(3).collect();
                        eprintln!("WARN rate_report parse: {} entries, all filtered. sample={:?}", rr_obj.len(), sample);
                    }
                }
                if let Some(rid) = receiver_id {
                    let c = coordinator.clone();
                    tokio::spawn(async move {
                        c.handle_rate_report(rid, rates).await;
                    });
                }
                return Ok(());
            }
        }

        // Python: elif 'heartbeat' in msg: self.process_heartbeat_message(msg['heartbeat']) (no-op)
        if obj.get("heartbeat").is_some() {
            return Ok(());
        }

        // Python: elif 'quine' in msg: self.process_quine_message(msg['quine']) — accept and no-op for parity
        if obj.get("quine").is_some() {
            return Ok(());
        }

        eprintln!("Unknown message format: {}", line);
        Ok(())
    }

    async fn dispatch_client_message(&mut self, msg: ClientMessage) -> io::Result<()> {
        // Spawn coordinator calls to avoid blocking the heartbeat loop (same as handle_message).
        let coordinator = self.coordinator.clone();
        let receiver_id = self.receiver_id;
        if let Some(rid) = receiver_id {
            let c = coordinator.clone();
            tokio::spawn(async move { c.increment_receiver_message_count(rid).await; });
        }
        match msg {
            ClientMessage::Heartbeat {} => {},
            ClientMessage::Handshake { .. } => {
                eprintln!("Unexpected handshake after authentication");
            },
            ClientMessage::Mlat { timestamp, message, utc } => {
                if let Some(rid) = receiver_id {
                    let c = coordinator.clone();
                    tokio::spawn(async move {
                        c.handle_mlat(rid, timestamp, &message, utc.unwrap_or(0.0)).await;
                    });
                }
            },
            ClientMessage::Sync { even_timestamp, odd_timestamp, even_message, odd_message } => {
                if let Some(rid) = receiver_id {
                    let c = coordinator.clone();
                    tokio::spawn(async move {
                        c.handle_sync(rid, even_timestamp, odd_timestamp, &even_message, &odd_message).await;
                    });
                }
            },
            ClientMessage::RateReport { rates } => {
                if let Some(rid) = receiver_id {
                    let c = coordinator.clone();
                    tokio::spawn(async move {
                        c.handle_rate_report(rid, rates).await;
                    });
                }
            }
        }
        Ok(())
    }
    
    /// Send a heartbeat message (flat format: {"heartbeat": {"server_time": ...}}, match Python)
    async fn send_heartbeat(&mut self) -> io::Result<()> {
        let msg = ServerMessage::Heartbeat {
            server_time: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        };
        let wire = server_message_to_wire(&msg);
        if let Some(json) = wire.first() {
            self.send_json_line(json).await?;
        }
        Ok(())
    }
    
    /// Send a traffic request (flat format: {"start_sending": [...]} and/or {"stop_sending": [...]}, match Python)
    pub async fn send_traffic_request(&mut self, start: Vec<String>, stop: Vec<String>) -> io::Result<()> {
        let msg = ServerMessage::TrafficRequest { start, stop };
        for json in server_message_to_wire(&msg) {
            self.send_json_line(&json).await?;
        }
        Ok(())
    }
    
    /// Send an MLAT result (flat format: {"result": {"@", "addr", "lat", ...}}, match Python)
    pub async fn send_mlat_result(
        &mut self,
        icao: String,
        lat: f64,
        lon: f64,
        alt: f64,
    ) -> io::Result<()> {
        let msg = ServerMessage::MlatResult {
            icao,
            lat,
            lon,
            alt,
            callsign: None,
            squawk: None,
            hdop: None,
            vdop: None,
            tdop: None,
            gdop: None,
            nstations: None,
        };
        let wire = server_message_to_wire(&msg);
        if let Some(json) = wire.first() {
            self.send_json_line(json).await?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_client_state() {
        // Basic state transitions
        assert_eq!(ClientState::AwaitingHandshake, ClientState::AwaitingHandshake);
        assert_ne!(ClientState::AwaitingHandshake, ClientState::Authenticated);
    }
}
