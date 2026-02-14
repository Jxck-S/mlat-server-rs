// JSON client handler
// Manages individual JSON client connections

use std::time::{Duration, Instant};
use tokio::time;
use std::io;
use std::sync::Arc;
use rand::Rng;

use super::connection::Connection;
use super::messages::{ClientMessage, HandshakeRequest, ServerMessage, SyncPayload};

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

/// JSON client handler
pub struct JsonClient {
    connection: Connection,
    state: ClientState,
    receiver_id: Option<usize>,
    last_message_time: Instant,
    motd: String,
    coordinator: Arc<crate::coordinator::Coordinator>,
}

impl JsonClient {
    /// Create a new JSON client
    pub fn new(
        connection: Connection,
        motd: String,
        coordinator: Arc<crate::coordinator::Coordinator>,
    ) -> Self {
        JsonClient {
            connection,
            state: ClientState::AwaitingHandshake,
            receiver_id: None,
            last_message_time: Instant::now(),
            motd,
            coordinator,
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
        
        // Start heartbeat task
        let heartbeat_interval = Duration::from_secs(30);
        let read_timeout = Duration::from_secs(150);
        
        let result = loop {
            // Check for read timeout (match Python: "No recent messages seen, closing connection")
            if self.last_message_time.elapsed() > read_timeout {
                eprintln!("No recent messages seen, closing connection");
                self.state = ClientState::Closed;
                break Ok(());
            }
            
            // Wait for next message or heartbeat
            tokio::select! {
                result = self.connection.read_line() => {
                    match result {
                        Ok(line) => {
                            if line.is_empty() {
                                // EOF (match Python: "Client EOF")
                                eprintln!("Client EOF");
                                self.state = ClientState::Closed;
                                break Ok(());
                            }
                            
                            self.last_message_time = Instant::now();
                            self.handle_message(&line).await?;
                        }
                        Err(e) => {
                            eprintln!("Read error: {}", e);
                            self.state = ClientState::Closed;
                            break Err(e);
                        }
                    }
                }
                Some(srv_msg) = rx.recv() => {
                    // Send message from coordinator to client
                    let json = serde_json::to_value(&srv_msg)?;
                    self.connection.write_json(&json).await?;
                }
                _ = time::sleep(heartbeat_interval) => {
                    // Send heartbeat
                    self.send_heartbeat().await?;
                }
            }
        };

        // Unregister, disconnect from coordinator, and log (match Python: "Disconnected: ({conn_info})")
        if let Some(receiver_id) = self.receiver_id {
            if let Some(conn_info) = self.coordinator.get_receiver_connection_info(receiver_id).await {
                eprintln!("Disconnected: ({})", conn_info);
            }
            self.coordinator.unregister_client(receiver_id).await;
            self.coordinator.receiver_disconnect(receiver_id).await;
        }

        result
    }
    
    /// Process the initial handshake
    ///
    /// Returns true if handshake succeeded, false if it failed.
    /// Accepts flat handshake format (no "type" field) as sent by mlat-client and test client.
    /// If the first line starts with "PROXY " (HAProxy etc.), parses client IP/port and uses the next line as handshake (Python parity).
    async fn process_handshake(&mut self) -> io::Result<bool> {
        // Read first line with timeout
        let mut line = match time::timeout(
            Duration::from_secs(15),
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
                        Duration::from_secs(15),
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

        let HandshakeRequest { version, user, lat, lon, alt, clock_type, uuid, privacy, .. } = handshake;

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
        let connection_info = format!("{} v{} {} unknown tcp none", user, version, clock_str);

        // Create receiver via coordinator (enforces one connection per user)
        let receiver_id = match self.coordinator.new_receiver(
            user.clone(),
            lat,
            lon,
            alt,
            clock_str,
            uuid.clone(),
            privacy,
            connection_info.clone(),
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

        // Send handshake response (Python: reconnect_in=util.fuzzy(10) => 9-11)
        let response = ServerMessage::Handshake {
            motd: self.motd.clone(),
            reconnect_in: Some(fuzzy(10.0)),
            compress: Some("none".to_string()),
            selective_traffic: Some(true),
            heartbeat: Some(true),
            return_results: Some(true),
        };

        let json = serde_json::to_value(&response)?;
        self.connection.write_json(&json).await?;

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
    
    /// Handle a message from the client.
    /// Accepts both tag format {"type": "sync", ...} and wrapped format {"sync": {"et", "ot", "em", "om"}}.
    async fn handle_message(&mut self, line: &str) -> io::Result<()> {
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
        if let Some(rid) = self.receiver_id {
            self.coordinator.increment_receiver_message_count(rid).await;
        }

        if let Some(sync_val) = obj.get("sync") {
            if let Ok(payload) = serde_json::from_value::<SyncPayload>(sync_val.clone()) {
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.handle_sync(
                        receiver_id,
                        payload.et,
                        payload.ot,
                        &payload.em,
                        &payload.om,
                    ).await;
                }
                return Ok(());
            }
        }

        if let Some(mlat_val) = obj.get("mlat") {
            if let Some(mlat_obj) = mlat_val.as_object() {
                if let (Some(t), Some(m)) = (mlat_obj.get("t").and_then(|v| v.as_f64()), mlat_obj.get("m").and_then(|v| v.as_str())) {
                    if let Some(receiver_id) = self.receiver_id {
                        self.coordinator.handle_mlat(receiver_id, t, m, 0.0).await;
                    }
                    return Ok(());
                }
            }
        }

        if let Some(seen_val) = obj.get("seen") {
            if let Some(arr) = seen_val.as_array() {
                let icaos: Vec<u32> = arr.iter().filter_map(|v| v.as_str().and_then(|s| u32::from_str_radix(s, 16).ok())).collect();
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.receiver_tracking_add(receiver_id, &icaos).await;
                }
                return Ok(());
            }
        }

        if let Some(lost_val) = obj.get("lost") {
            if let Some(arr) = lost_val.as_array() {
                let icaos: Vec<u32> = arr.iter().filter_map(|v| v.as_str().and_then(|s| u32::from_str_radix(s, 16).ok())).collect();
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.receiver_tracking_remove(receiver_id, &icaos).await;
                }
                return Ok(());
            }
        }

        if obj.get("input_connected").is_some() {
            if let Some(receiver_id) = self.receiver_id {
                self.coordinator.receiver_clock_reset(receiver_id, true).await;
            }
            return Ok(());
        }
        if obj.get("input_disconnected").is_some() {
            if let Some(receiver_id) = self.receiver_id {
                self.coordinator.receiver_clock_reset(receiver_id, false).await;
            }
            return Ok(());
        }
        if obj.get("clock_reset").is_some() {
            if let Some(receiver_id) = self.receiver_id {
                self.coordinator.receiver_clock_reset(receiver_id, false).await;
            }
            return Ok(());
        }
        if obj.get("clock_jump").is_some() {
            if let Some(receiver_id) = self.receiver_id {
                self.coordinator.receiver_clock_reset(receiver_id, false).await;
            }
            return Ok(());
        }

        if let Some(rr_val) = obj.get("rate_report") {
            if let Some(rr_obj) = rr_val.as_object() {
                let rates: std::collections::HashMap<u32, f64> = rr_obj.iter()
                    .filter_map(|(k, v)| v.as_f64().and_then(|f| u32::from_str_radix(k, 16).ok().map(|u| (u, f))))
                    .collect();
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.handle_rate_report(receiver_id, rates).await;
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
        // Python: message_counter += 1 per message (tag format path)
        if let Some(rid) = self.receiver_id {
            self.coordinator.increment_receiver_message_count(rid).await;
        }
        match msg {
            ClientMessage::Heartbeat {} => {},
            ClientMessage::Handshake { .. } => {
                eprintln!("Unexpected handshake after authentication");
            },
            ClientMessage::Mlat { timestamp, message, utc } => {
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.handle_mlat(receiver_id, timestamp, &message, utc.unwrap_or(0.0)).await;
                }
            },
            ClientMessage::Sync { even_timestamp, odd_timestamp, even_message, odd_message } => {
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.handle_sync(
                        receiver_id,
                        even_timestamp,
                        odd_timestamp,
                        &even_message,
                        &odd_message,
                    ).await;
                }
            },
            ClientMessage::RateReport { rates } => {
                if let Some(receiver_id) = self.receiver_id {
                    self.coordinator.handle_rate_report(receiver_id, rates).await;
                }
            }
        }
        Ok(())
    }
    
    /// Send a heartbeat message
    async fn send_heartbeat(&mut self) -> io::Result<()> {
        let msg = ServerMessage::Heartbeat {
            server_time: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        };
        
        let json = serde_json::to_value(&msg)?;
        self.connection.write_json(&json).await
    }
    
    /// Send a traffic request to the client
    pub async fn send_traffic_request(&mut self, start: Vec<String>, stop: Vec<String>) -> io::Result<()> {
        let msg = ServerMessage::TrafficRequest { start, stop };
        let json = serde_json::to_value(&msg)?;
        self.connection.write_json(&json).await
    }
    
    /// Send an MLAT result to the client
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
        
        let json = serde_json::to_value(&msg)?;
        self.connection.write_json(&json).await
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
