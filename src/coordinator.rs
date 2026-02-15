// Coordinator - top level glue between receivers, tracker, clock sync, and MLAT processing
// Ported from mlat/coordinator.py

use std::collections::{HashMap, HashSet};
use std::time::Duration;
use tracing::{debug, info};
use std::sync::Arc;
use std::path::Path;
use tokio::sync::{RwLock, mpsc};
use tokio::io::AsyncWriteExt;

use crate::constants::{MTOF, MLAT_DELAY};

/// Username prefix for debug/focus logging (Python config.DEBUG_FOCUS). Receivers with user starting with this get focus=true.
const DEBUG_FOCUS: &str = "euerdorf";

/// Coordinator for MLAT server
///
/// Top-level object that connects network layer, tracker, clock sync,
/// and MLAT processing. Mirrors Python's Coordinator class.
pub struct Coordinator {
    receivers: Arc<RwLock<HashMap<usize, crate::receiver::Receiver>>>,
    /// Map username -> receiver_id so we enforce one connection per user (Python: usernames)
    usernames: Arc<RwLock<HashMap<String, usize>>>,
    /// Map UDP key -> receiver_id for UDP transport (Python: udp_protocol.add_client returns key)
    udp_keys: Arc<RwLock<HashMap<u32, usize>>>,
    tracker: Arc<RwLock<crate::tracker::Tracker>>,
    mlat_tracker: Arc<RwLock<crate::mlattrack::MlatTracker>>,
    clock_tracker: Arc<RwLock<crate::clocktrack::ClockTracker>>,
    outputs: Arc<RwLock<Vec<Box<dyn crate::output::OutputHandler>>>>,
    next_receiver_id: Arc<RwLock<usize>>,
    client_channels: Arc<RwLock<HashMap<usize, tokio::sync::mpsc::Sender<crate::net::messages::ServerMessage>>>>,

    /// Work directory for state files (sync.json, clients.json, etc.). Empty = no file writes.
    #[allow(dead_code)]
    work_dir: String,
    /// Status log interval in seconds. <= 0 = disabled.
    status_interval_secs: i32,

    /// Cohort delay: batches of ICAOs to resolve after MLAT_DELAY (Python: Cohort with call_later(MLAT_DELAY, _process)).
    /// Current cohort: (icaos, creation_instant). New cohort when creation > 50ms ago or len >= 25.
    cohort: Arc<RwLock<Option<Arc<RwLock<(HashSet<u32>, tokio::time::Instant)>>>>>,
    /// Channel to signal "this cohort's icaos are ready to resolve" (sent by delayed task after MLAT_DELAY).
    cohort_ready_tx: mpsc::Sender<HashSet<u32>>,
    /// Receiver for cohort_ready_tx; taken by run() to process delayed cohorts.
    cohort_ready_rx: std::sync::Mutex<Option<mpsc::Receiver<HashSet<u32>>>>,

    // Stats
    pub total_mlat_messages: Arc<RwLock<usize>>,
    pub total_sync_messages: Arc<RwLock<usize>>,
    pub total_solutions: Arc<RwLock<usize>>,
}

impl Coordinator {
    /// Create a new coordinator (no status logging; for tests).
    pub fn new() -> Self {
        Self::new_with_status(String::new(), -1)
    }

    /// Create a new coordinator with work_dir and status_interval (for production).
    /// When status_interval > 0, logs "Status: (N clients ...)" every status_interval seconds.
    pub fn new_with_status(work_dir: String, status_interval: i32) -> Self {
        let (cohort_ready_tx, cohort_ready_rx) = mpsc::channel(64);
        Coordinator {
            receivers: Arc::new(RwLock::new(HashMap::new())),
            usernames: Arc::new(RwLock::new(HashMap::new())),
            udp_keys: Arc::new(RwLock::new(HashMap::new())),
            tracker: Arc::new(RwLock::new(crate::tracker::Tracker::new(0, 1))),
            mlat_tracker: Arc::new(RwLock::new(crate::mlattrack::MlatTracker::new())),
            clock_tracker: Arc::new(RwLock::new(crate::clocktrack::ClockTracker::new())),
            outputs: Arc::new(RwLock::new(Vec::new())),
            next_receiver_id: Arc::new(RwLock::new(1)),
            client_channels: Arc::new(RwLock::new(HashMap::new())),
            work_dir,
            status_interval_secs: status_interval,
            cohort: Arc::new(RwLock::new(None)),
            cohort_ready_tx,
            cohort_ready_rx: std::sync::Mutex::new(Some(cohort_ready_rx)),
            total_mlat_messages: Arc::new(RwLock::new(0)),
            total_sync_messages: Arc::new(RwLock::new(0)),
            total_solutions: Arc::new(RwLock::new(0)),
        }
    }

    /// Register a client's message channel
    pub async fn register_client(&self, receiver_id: usize, tx: tokio::sync::mpsc::Sender<crate::net::messages::ServerMessage>) {
        self.client_channels.write().await.insert(receiver_id, tx);
    }

    /// Unregister a client's message channel
    pub async fn unregister_client(&self, receiver_id: usize) {
        self.client_channels.write().await.remove(&receiver_id);
    }

    /// Get connection_info for a receiver (for disconnect logging). Returns None if receiver not found.
    pub async fn get_receiver_connection_info(&self, receiver_id: usize) -> Option<String> {
        let receivers = self.receivers.read().await;
        receivers.get(&receiver_id).map(|r| r.connection_info.clone())
    }

    /// Add an output handler
    pub async fn add_output(&self, output: Box<dyn crate::output::OutputHandler>) {
        self.outputs.write().await.push(output);
    }

    /// Create work_dir and initial state files (empty JSON + handshakes.log). Call once at startup.
    /// Mirrors Python: same files and schema; Python creates them on first _write_state / handshake.
    pub async fn init_work_dir(&self) {
        if self.work_dir.is_empty() {
            return;
        }
        let dir = Path::new(&self.work_dir);
        if let Err(e) = std::fs::create_dir_all(dir) {
            eprintln!("Failed to create work_dir {}: {}", self.work_dir, e);
            return;
        }
        let empty = "{}";
        for name in &["sync.json", "clients.json", "aircraft.json"] {
            let p = dir.join(name);
            if let Err(e) = std::fs::write(&p, empty) {
                eprintln!("Failed to write {}: {}", p.display(), e);
            }
        }
        let log_path = dir.join("handshakes.log");
        let _ = std::fs::File::create(&log_path); // create empty or truncate; first handshake will append
    }

    /// Log raw handshake line to work_dir/handshakes.log (same format as Python handshake_logger.debug(line)).
    pub async fn log_handshake(&self, line: &str) {
        if self.work_dir.is_empty() {
            return;
        }
        let path = Path::new(&self.work_dir).join("handshakes.log");
        if let Ok(mut f) = tokio::fs::OpenOptions::new().create(true).append(true).open(&path).await {
            let _ = f.write_all(line.as_bytes()).await;
            let _ = f.write_all(b"\n").await;
        }
    }

    /// Set source IP/port for a receiver (from connection peer_addr). Used for clients.json schema.
    pub async fn set_receiver_source(&self, receiver_id: usize, source_ip: String, source_port: u16) {
        if let Some(r) = self.receivers.write().await.get_mut(&receiver_id) {
            r.source_ip = Some(source_ip);
            r.source_port = Some(source_port);
        }
    }

    /// Register a UDP transport key for a receiver (Python: udp_protocol.add_client).
    pub async fn register_udp_key(&self, key: u32, receiver_id: usize) {
        self.udp_keys.write().await.insert(key, receiver_id);
    }

    /// Look up receiver ID by UDP key. Caller should remove key on disconnect (Python: remove_client).
    pub async fn get_receiver_id_for_udp_key(&self, key: u32) -> Option<usize> {
        self.udp_keys.read().await.get(&key).copied()
    }

    /// Remove UDP key when client disconnects (so UDP packets with that key are ignored).
    pub async fn unregister_udp_key(&self, key: u32) {
        self.udp_keys.write().await.remove(&key);
    }

    /// Diagnostics: record that a JSON message was received (for message rate / stats).
    pub async fn record_message_received(&self) {
        // Could increment a counter for status output; no-op for now.
        let _ = self;
    }

    /// Diagnostics: record that a line contained "sync" (for sync parse debugging).
    pub async fn record_line_containing_sync(&self) {
        let _ = self;
    }

    /// Diagnostics: record that the "sync" key was seen in a message.
    pub async fn record_sync_key_seen(&self) {
        let _ = self;
    }

    /// Diagnostics: record that sync payload parse failed.
    pub async fn record_sync_parse_fail(&self) {
        let _ = self;
    }

    /// Create a new receiver from handshake.
    ///
    /// Returns the receiver ID, or Err if this username is already connected (Python: one connection per user).
    /// `connection_info` should match Python format: `{user} v{version} {clock_type} {client_version} {udp} {compress}`
    pub async fn new_receiver(
        &self,
        user: String,
        lat: f64,
        lon: f64,
        alt: f64,
        clock_type: &str,
        uuid: Option<String>,
        privacy: bool,
        connection_info: String,
        return_stats: bool,
    ) -> Result<usize, String> {
        if self.usernames.read().await.contains_key(&user) {
            return Err(format!("User {} is already connected", user));
        }

        let mut next_id = self.next_receiver_id.write().await;
        let receiver_id = *next_id;
        *next_id += 1;
        drop(next_id);

        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        let mut receiver = crate::receiver::Receiver::new(
            receiver_id,
            user.clone(),
            uuid,
            clock_type,
            [lat, lon, alt],
            privacy,
            connection_info.clone(),
            now,
            return_stats,
        );
        if user.starts_with(DEBUG_FOCUS) {
            receiver.focus = true;
        }
        receiver.set_map_position_from_llh();

        println!("Created receiver {}: user={} position=[{}, {}, {}]",
                 receiver_id, user, lat, lon, alt);

        self.receivers.write().await.insert(receiver_id, receiver);
        self.usernames.write().await.insert(user, receiver_id);

        // Update distance matrix for the new receiver
        let mut receivers = self.receivers.write().await;
        crate::receiver::update_distance_matrix(&mut receivers, receiver_id);

        Ok(receiver_id)
    }
    
    /// Handle MLAT message from a receiver
    pub async fn handle_mlat(
        &self,
        receiver_id: usize,
        timestamp: f64,
        message_hex: &str,
        _utc: f64,
    ) {
        *self.total_mlat_messages.write().await += 1;
        // Server Unix time for "last seen" and MLAT wanted (match Python time.time())
        let now_unix = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        // 1. Decode hex message
        debug!(receiver_id, message_hex, "Handling MLAT for receiver");
        let bytes = match hex::decode(message_hex) {
            Ok(b) => b,
            Err(_) => {
                // println!("Invalid hex: {}", message_hex);
                return;
            }, 
        };

        // 2. Parse Mode S message
        let msg = match crate::modes::message::decode(&bytes) {
            Some(m) => m,
            None => {
                debug!(message_hex, "Invalid Mode S message");
                return;
            }, 
        };

        // 3. Extract metadata
        let traj = msg.as_trait();
        let icao = traj.address();
        let df = traj.df();

        // 4. Update Tracker (use now_unix for ac.seen so aircraft.json elapsed_seen is correct)
        let mut tracker = self.tracker.write().await;
        
        // Update aircraft state
        tracker.update_aircraft(
            icao,
            now_unix,
            &msg,
            receiver_id,
        );

        // Ensure MLAT wanted status is up to date
        tracker.update_mlat_wanted(now_unix);

        // 5. MLAT Processing (Python: receiver_mlat adds to cohort; resolution after MLAT_DELAY)
        let mut mlat_tracker = self.mlat_tracker.write().await;

        if mlat_tracker.process_message(receiver_id, bytes, timestamp).is_some() {
            self.add_icao_to_cohort(icao).await;
        }
        
        // Log for debug (can remove later or move to trace level)
        if df == 17 {
            // println!("Parsed DF{} from {:06X}: {}", df, icao, message_hex);
        }
    }

    /// Add ICAO to current cohort; start new cohort if >50ms or >=25 groups (Python: now - cohort.creationTime > 0.05 or cohort.len > 25).
    /// Schedules processing after MLAT_DELAY (0.9s) when a new cohort is created.
    async fn add_icao_to_cohort(&self, icao: u32) {
        const COHORT_MAX_AGE_SECS: f64 = 0.05;
        const COHORT_MAX_GROUPS: usize = 25;

        let mut cohort_guard = self.cohort.write().await;
        let need_new = match cohort_guard.as_ref() {
            None => true,
            Some(arc) => {
                let guard = arc.read().await;
                guard.1.elapsed() > Duration::from_secs_f64(COHORT_MAX_AGE_SECS) || guard.0.len() >= COHORT_MAX_GROUPS
            }
        };

        if need_new {
            let arc = Arc::new(RwLock::new((
                HashSet::from([icao]),
                tokio::time::Instant::now(),
            )));
            *cohort_guard = Some(Arc::clone(&arc));
            drop(cohort_guard);

            let tx = self.cohort_ready_tx.clone();
            tokio::spawn(async move {
                let creation = arc.read().await.1;
                tokio::time::sleep_until(creation + Duration::from_secs_f64(MLAT_DELAY)).await;
                let icaos: HashSet<u32> = arc.write().await.0.drain().collect();
                let _ = tx.send(icaos).await;
            });
        } else {
            let arc = cohort_guard.as_ref().unwrap().clone();
            drop(cohort_guard);
            arc.write().await.0.insert(icao);
        }
    }

    /// Process a cohort that has been delayed by MLAT_DELAY (Python: cohort._process -> group.handle(group) -> _resolve).
    async fn process_delayed_cohort(&self, icaos: HashSet<u32>) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        let receivers = self.receivers.read().await;
        let clock_tracker = self.clock_tracker.read().await;

        for icao in icaos {
            let result = {
                let mut mlat_tracker = self.mlat_tracker.write().await;
                let mut tracker = self.tracker.write().await;
                mlat_tracker.try_resolve_pending(icao, &receivers, &mut tracker, &clock_tracker, now)
            };
            if let Some(ref result) = result {
                self.forward_results(result).await;
                *self.total_solutions.write().await += 1;
                let context = {
                    let tracker = self.tracker.read().await;
                    tracker.get(result.icao).map(|ac| crate::output::OutputContext {
                        callsign: ac.callsign.clone(),
                        squawk: ac.squawk,
                        altitude_ft: ac.altitude,
                        last_altitude_time: ac.last_altitude_time,
                        vrate_fpm: ac.vrate,
                        vrate_time: ac.vrate_time,
                    })
                };
                let mut outputs = self.outputs.write().await;
                for output in outputs.iter_mut() {
                    output.handle_result(result, context.as_ref());
                }
            }
        }
    }
    
    /// Handle sync message from a receiver (CPR even/odd pair)
    /// Mirrors Python `coordinator.receiver_sync` flow.
    pub async fn handle_sync(
        &self,
        receiver_id: usize,
        even_ts: f64,
        odd_ts: f64,
        even_msg: &str,
        odd_msg: &str,
    ) {
        // 1. Hex-decode the messages
        let even_bytes = match hex::decode(even_msg) {
            Ok(b) => b,
            Err(_) => {
                return;
            }
        };
        let odd_bytes = match hex::decode(odd_msg) {
            Ok(b) => b,
            Err(_) => {
                return;
            }
        };

        // 2. Build SyncReceiverInfo for the reporting receiver
        let receivers_lock = self.receivers.read().await;
        let receiver = match receivers_lock.get(&receiver_id) {
            Some(r) => r,
            None => {
                return;
            }
        };

        let sync_receiver = crate::clocktrack::SyncReceiverInfo {
            uid: receiver.uid,
            position: receiver.position,
            clock: receiver.clock.clone(),
            distance: receiver.distance.clone(),
            sync_peers: receiver.sync_peers,
            dead: receiver.dead,
            sync_range_exceeded: receiver.sync_range_exceeded,
            last_sync: receiver.last_sync,
            bad_syncs: receiver.bad_syncs,
        };

        // 3. Build sync receiver info map for all receivers
        let mut sync_receivers: std::collections::HashMap<usize, crate::clocktrack::SyncReceiverInfo> =
            receivers_lock.iter().map(|(&uid, r)| {
                (uid, crate::clocktrack::SyncReceiverInfo {
                    uid: r.uid,
                    position: r.position,
                    clock: r.clock.clone(),
                    distance: r.distance.clone(),
                    sync_peers: r.sync_peers,
                    dead: r.dead,
                    sync_range_exceeded: r.sync_range_exceeded,
                    last_sync: r.last_sync,
                    bad_syncs: r.bad_syncs,
                })
            }).collect();

        // Drop read lock before acquiring write lock
        drop(receivers_lock);

        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        // 4. Call receiver_sync on the clock tracker
        let mut clock_tracker = self.clock_tracker.write().await;
        clock_tracker.receiver_sync(
            &sync_receiver,
            even_ts,
            odd_ts,
            &even_bytes,
            &odd_bytes,
            &mut sync_receivers,
            now,
        );

        // 5. Sync-MLAT integration: ADS-B messages used for sync are also MLAT candidates
        drop(clock_tracker);
        self.handle_mlat(receiver_id, even_ts, even_msg, 0.0).await;
        self.handle_mlat(receiver_id, odd_ts, odd_msg, 0.0).await;
        
        *self.total_sync_messages.write().await += 2;

        // 5. Sync back metrics (sync_peers, sync_range_exceeded) to the reporting receivers
        let mut receivers_write = self.receivers.write().await;
        for (uid, sync_info) in sync_receivers {
            if let Some(r) = receivers_write.get_mut(&uid) {
                r.sync_peers = sync_info.sync_peers;
                r.sync_range_exceeded = sync_info.sync_range_exceeded;
                r.last_sync = sync_info.last_sync;
                r.bad_syncs = sync_info.bad_syncs;
            }
        }


    }
    
    /// Get receiver count
    pub async fn receiver_count(&self) -> usize {
        self.receivers.read().await.len()
    }
    
    /// Get a receiver by ID
    pub async fn get_receiver(&self, receiver_id: usize) -> Option<crate::receiver::Receiver> {
        self.receivers.read().await.get(&receiver_id).cloned()
    }

    /// Increment message counter for a receiver (Python: connection.message_counter += 1 per JSON message).
    /// Used for message_rate = counter/15 in write_state.
    pub async fn increment_receiver_message_count(&self, receiver_id: usize) {
        if let Some(r) = self.receivers.write().await.get_mut(&receiver_id) {
            r.message_counter = r.message_counter.saturating_add(1);
        }
    }

    /// Notes that the given receiver has disconnected. Removes from receivers, usernames, tracker, and clock_tracker.
    /// Mirrors Python coordinator.receiver_disconnect (coordinator.py L643-654).
    pub async fn receiver_disconnect(&self, receiver_id: usize) {
        let user_opt = self.receivers.read().await.get(&receiver_id).map(|r| r.user.clone());

        // 1. Tracker: remove all aircraft for this receiver (Python order)
        {
            let mut tracker = self.tracker.write().await;
            tracker.remove_all(receiver_id);
        }

        // 2. Build sync receiver info map for clock_tracker.receiver_disconnect
        let sync_receivers: std::collections::HashMap<usize, crate::clocktrack::SyncReceiverInfo> = {
            let receivers = self.receivers.read().await;
            receivers.iter().map(|(&uid, r)| {
                (uid, crate::clocktrack::SyncReceiverInfo {
                    uid: r.uid,
                    position: r.position,
                    clock: r.clock.clone(),
                    distance: r.distance.clone(),
                    sync_peers: r.sync_peers,
                    dead: r.dead,
                    sync_range_exceeded: r.sync_range_exceeded,
                    last_sync: r.last_sync,
                    bad_syncs: r.bad_syncs,
                })
            }).collect()
        };

        // 3. Clock tracker: remove pairings involving this receiver (updates sync_peers in sync_receivers)
        let mut sync_receivers = sync_receivers;
        {
            let mut clock_tracker = self.clock_tracker.write().await;
            clock_tracker.receiver_disconnect(receiver_id, &mut sync_receivers);
        }

        // 4. Remove receiver from map (Python: receivers.pop), clean distance entries, write back sync_peers, usernames.pop
        {
            let mut receivers = self.receivers.write().await;
            receivers.remove(&receiver_id);
            crate::receiver::remove_distance_entries(&mut receivers, receiver_id);
            for (uid, sync_info) in sync_receivers {
                if let Some(r) = receivers.get_mut(&uid) {
                    r.sync_peers = sync_info.sync_peers;
                    r.sync_range_exceeded = sync_info.sync_range_exceeded;
                    r.last_sync = sync_info.last_sync;
                    r.bad_syncs = sync_info.bad_syncs;
                }
            }
        }
        if let Some(u) = user_opt {
            self.usernames.write().await.remove(&u);
        }
    }

    /// Forward MLAT result to participating receivers for clock tuning
    /// Mirrors Python coordinator.forward_results (coordinator.py L679-697).
    pub async fn forward_results(
        &self,
        result: &crate::mlattrack::MlatResult,
    ) {
        let (lat, lon, alt) = crate::geodesy::ecef2llh(result.position[0], result.position[1], result.position[2]);
        
        let msg = crate::net::messages::ServerMessage::Position {
            timestamp: result.timestamp,
            addr: format!("{:06x}", result.icao),
            lat,
            lon,
            alt: alt * MTOF, // constants::MTOF (1/0.3038)
            callsign: None,
            squawk: None,
            hdop: None,
            vdop: None,
            tdop: None,
            gdop: None,
            nstations: result.receivers.len(),
        };

        let channels = self.client_channels.read().await;
        for &rid in &result.receivers {
            if let Some(tx) = channels.get(&rid) {
                // Ignore send error if client disconnected
                let _ = tx.send(msg.clone()).await;
            }
        }
    }

    /// Add aircraft to a receiver's tracking set (from client "seen" message). Mirrors Python receiver_tracking_add.
    pub async fn receiver_tracking_add(&self, receiver_id: usize, icao_set: &[u32]) {
        if icao_set.is_empty() {
            return;
        }
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let mut tracker = self.tracker.write().await;
        tracker.add(receiver_id, icao_set, now);
    }

    /// Remove aircraft from a receiver's tracking set (from client "lost" message). Mirrors Python receiver_tracking_remove.
    pub async fn receiver_tracking_remove(&self, receiver_id: usize, icao_set: &[u32]) {
        if icao_set.is_empty() {
            return;
        }
        let mut tracker = self.tracker.write().await;
        tracker.remove(receiver_id, icao_set);
    }

    /// Reset clock sync state for a receiver (on input_connected, input_disconnected, clock_reset, clock_jump).
    /// Mirrors Python receiver.clock_reset(); when clear_counter is true (input_connected), counter is set to 0 after reset.
    pub async fn receiver_clock_reset(&self, receiver_id: usize, clear_counter: bool) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        {
            let mut receivers = self.receivers.write().await;
            if let Some(r) = receivers.get_mut(&receiver_id) {
                r.reset_clock(now);
                if clear_counter {
                    r.clock_reset_counter = 0;
                }
            }
        }
        let mut clock_tracker = self.clock_tracker.write().await;
        clock_tracker.receiver_clock_reset(receiver_id);
    }

    /// Handle rate report from a receiver and update interest sets
    pub async fn handle_rate_report(&self, receiver_id: usize, rates: HashMap<u32, f64>) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        // 1. Update receiver's rate report
        {
            let mut receivers = self.receivers.write().await;
            if let Some(r) = receivers.get_mut(&receiver_id) {
                r.last_rate_report = Some(rates);
            } else {
                return;
            }
        }

        // 2. Perform update
        self.update_receiver_interest(receiver_id, now).await;
    }

    /// Internal helper to update interest sets for a single receiver
    async fn update_receiver_interest(&self, receiver_id: usize, now: f64) {
        // 1. Perform interest set recalculation
        let (new_sync, new_mlat, new_adsb) = {
            let receivers = self.receivers.read().await;
            let mut tracker = self.tracker.write().await;
            // Use constants from config equivalent
            tracker.update_interest(receiver_id, &receivers, now, 15, 12.0)
        };

        // 2. Update interest sets and get traffic request changes
        let (start, stop) = {
            let mut receivers = self.receivers.write().await;
            if let Some(r) = receivers.get_mut(&receiver_id) {
                r.update_interest_sets(new_sync, new_mlat, new_adsb)
            } else {
                return;
            }
        };

        // 3. Send traffic request if there are changes
        if !start.is_empty() || !stop.is_empty() {
            let start_hex: Vec<String> = start.iter().map(|&icao| format!("{:06x}", icao)).collect();
            let stop_hex: Vec<String> = stop.iter().map(|&icao| format!("{:06x}", icao)).collect();
            
            let msg = crate::net::messages::ServerMessage::TrafficRequest {
                start: start_hex,
                stop: stop_hex,
            };

            let channels = self.client_channels.read().await;
            if let Some(tx) = channels.get(&receiver_id) {
                let _ = tx.send(msg).await;
            }
        }
    }

    /// Refresh interest sets for all receivers periodically
    pub async fn refresh_interest(&self) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        let receiver_ids: Vec<usize> = {
            let receivers = self.receivers.read().await;
            receivers.keys().copied().collect()
        };

        for rid in receiver_ids {
            self.update_receiver_interest(rid, now).await;
        }
    }

    /// Write sync.json, clients.json, aircraft.json to work_dir (same schema as Python _write_state).
    async fn write_state(&self) {
        if self.work_dir.is_empty() {
            return;
        }
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let dir = Path::new(&self.work_dir);

        let (sync_json, clients_json, aircraft_json, num_receivers) = {
            let mut receivers = self.receivers.write().await;
            let n = receivers.len();
            let mut tracker = self.tracker.write().await;
            let mut clock_tracker = self.clock_tracker.write().await;
            let pairing_rows = clock_tracker.dump_receiver_state(now);
            let id_to_user: HashMap<usize, String> = receivers.iter().map(|(&id, r)| (id, r.user.clone())).collect();

            let mut receiver_states: HashMap<String, HashMap<String, Vec<serde_json::Value>>> = HashMap::new();
            for ((base_id, peer_id), (n, error_us, drift_us, i_drift_us, outlier_pct, jumped, time_since)) in pairing_rows {
                let base_user = match id_to_user.get(&base_id) {
                    Some(u) => u.clone(),
                    None => continue,
                };
                let peer_user = match id_to_user.get(&peer_id) {
                    Some(u) => u.clone(),
                    None => continue,
                };
                let peer_bad_syncs = receivers.get(&peer_id).map(|r| (r.bad_syncs * 100.0).round() / 100.0).unwrap_or(0.0);
                let base_bad_syncs = receivers.get(&base_id).map(|r| (r.bad_syncs * 100.0).round() / 100.0).unwrap_or(0.0);
                let peer_sync_peers: usize = receivers.get(&peer_id).map(|r| r.sync_peers.iter().sum()).unwrap_or(0);
                let base_sync_peers: usize = receivers.get(&base_id).map(|r| r.sync_peers.iter().sum()).unwrap_or(0);
                receiver_states.entry(base_user.clone()).or_default().insert(peer_user.clone(), vec![
                    serde_json::json!(n),
                    serde_json::json!(error_us),
                    serde_json::json!(drift_us),
                    serde_json::json!(peer_bad_syncs),
                    serde_json::json!(jumped),
                    serde_json::json!(outlier_pct),
                    serde_json::json!(time_since),
                    serde_json::json!(peer_sync_peers),
                ]);
                receiver_states.entry(peer_user).or_default().insert(base_user.clone(), vec![
                    serde_json::json!(n),
                    serde_json::json!(error_us),
                    serde_json::json!(i_drift_us),
                    serde_json::json!(base_bad_syncs),
                    serde_json::json!(jumped),
                    serde_json::json!(outlier_pct),
                    serde_json::json!(time_since),
                    serde_json::json!(base_sync_peers),
                ]);
            }

            let mut sync = serde_json::Map::new();
            let mut clients = serde_json::Map::new();
            for (_uid, r) in receivers.iter_mut() {
                let user = r.user.clone();
                let peers = receiver_states.get(&user).cloned().unwrap_or_default();
                let peers_map: serde_json::Map<String, serde_json::Value> = peers.into_iter().map(|(k, v)| (k, serde_json::Value::Array(v))).collect();
                sync.insert(user.clone(), serde_json::json!({
                    "peers": peers_map,
                    "bad_syncs": r.bad_syncs,
                    "lat": r.map_lat,
                    "lon": r.map_lon
                }));
                let sync_interest: Vec<String> = r.sync_interest.iter().map(|&icao| format!("{:06x}", icao)).collect();
                let mlat_interest: Vec<String> = r.mlat_interest.iter().map(|&icao| format!("{:06x}", icao)).collect();
                // Python: message_rate = round(r.connection.message_counter / 15.0); then reset counter
                let message_rate = (r.message_counter as f64 / 15.0).round() as i64;
                r.message_counter = 0;
                clients.insert(user.clone(), serde_json::json!({
                    "user": r.user,
                    "uid": r.uid,
                    "uuid": r.uuid,
                    "coords": format!("{:.6},{:.6}", r.position_llh[0], r.position_llh[1]),
                    "lat": r.position_llh[0],
                    "lon": r.position_llh[1],
                    "alt": r.position_llh[2],
                    "privacy": r.privacy,
                    "connection": r.connection_info,
                    "source_ip": r.source_ip.as_deref().unwrap_or(""),
                    "source_port": r.source_port.unwrap_or(0),
                    "message_rate": message_rate,
                    "peer_count": peers_map.len(),
                    "bad_sync_timeout": (r.bad_syncs * 15.0 / 0.1).round(),
                    "outlier_percent": (r.outlier_percent_rolling * 10.0).round() / 10.0,
                    "bad_peer_list": "[]",
                    "sync_interest": sync_interest,
                    "mlat_interest": mlat_interest
                }));
            }
            let sync_json = serde_json::Value::Object(sync);
            let clients_json = serde_json::Value::Object(clients);

            let mut aircraft = serde_json::Map::new();
            for (icao, ac) in tracker.aircraft_iter_mut() {
                let elapsed_seen = now - ac.seen;
                let icao_hex = format!("{:06X}", icao);
                let sync_count = (ac.sync_good + ac.sync_bad) as f64;
                let sync_bad_percent = (100.0 * ac.sync_bad as f64 / (sync_count + 0.01)).round() * 10.0 / 10.0;
                ac.sync_bad_percent = sync_bad_percent;
                if ac.sync_bad > 3 && sync_bad_percent > 15.0 {
                    ac.sync_dont_use = true;
                } else {
                    ac.sync_dont_use = false;
                }
                ac.sync_good = ((ac.sync_good as f64) * 0.8) as usize;
                ac.sync_bad = ((ac.sync_bad as f64) * 0.8) as usize;
                let mut s = serde_json::Map::new();
                s.insert("icao".to_string(), serde_json::json!(icao_hex));
                s.insert("elapsed_seen".to_string(), serde_json::json!((elapsed_seen * 10.0).round() / 10.0));
                s.insert("interesting".to_string(), serde_json::json!(if ac.interesting() { 1 } else { 0 }));
                s.insert("allow_mlat".to_string(), serde_json::json!(if ac.allow_mlat { 1 } else { 0 }));
                s.insert("tracking".to_string(), serde_json::json!(ac.tracking.len()));
                s.insert("sync_interest".to_string(), serde_json::json!(ac.sync_interest.len()));
                s.insert("mlat_interest".to_string(), serde_json::json!(ac.mlat_interest.len()));
                s.insert("adsb_seen".to_string(), serde_json::json!(ac.adsb_seen.len()));
                s.insert("mlat_message_count".to_string(), serde_json::json!(ac.mlat_message_count));
                s.insert("mlat_result_count".to_string(), serde_json::json!(ac.mlat_result_count));
                s.insert("mlat_kalman_count".to_string(), serde_json::json!(ac.mlat_kalman_count));
                s.insert("sync_count_1min".to_string(), serde_json::json!(sync_count.round()));
                s.insert("sync_bad_percent".to_string(), serde_json::json!(sync_bad_percent));
                if let (Some(t), Some(p)) = (ac.last_result_time, ac.last_result_position) {
                    let (lat, lon, _alt_llh) = crate::geodesy::ecef2llh(p[0], p[1], p[2]);
                    s.insert("last_result".to_string(), serde_json::json!(((now - t) * 10.0).round() / 10.0));
                    s.insert("lat".to_string(), serde_json::json!((lat * 10000.0).round() / 10000.0));
                    s.insert("lon".to_string(), serde_json::json!((lon * 10000.0).round() / 10000.0));
                    if let Some(alt) = ac.altitude {
                        s.insert("alt".to_string(), serde_json::json!(alt.round()));
                    }
                    if ac.kalman.valid {
                        if let Some(h) = ac.kalman.heading {
                            s.insert("heading".to_string(), serde_json::json!(h.round()));
                        }
                        if let Some(sp) = ac.kalman.ground_speed {
                            s.insert("speed".to_string(), serde_json::json!(sp.round()));
                        }
                    }
                }
                if elapsed_seen > 600.0 {
                    let uids: Vec<usize> = ac.tracking.iter().copied().collect();
                    s.insert("tracking_receivers".to_string(), serde_json::json!(uids));
                }
                aircraft.insert(icao_hex, serde_json::Value::Object(s));
            }
            let aircraft_json = serde_json::Value::Object(aircraft);
            // Clear sync point cache periodically (Python: every 15s after _write_state)
            clock_tracker.clear_all_sync_points();
            (sync_json, clients_json, aircraft_json, n)
        };

        info!("write_state: {} receivers -> {}", num_receivers, self.work_dir);

        for (name, value) in [("sync.json", sync_json), ("clients.json", clients_json), ("aircraft.json", aircraft_json)] {
            let tmp = dir.join(format!("{}.tmp", name));
            let path = dir.join(name);
            if let Ok(s) = serde_json::to_string(&value) {
                if let Err(e) = std::fs::write(&tmp, s) {
                    eprintln!("write_state: failed to write {}: {}", tmp.display(), e);
                } else if let Err(e) = std::fs::rename(&tmp, &path) {
                    let _ = std::fs::remove_file(&tmp);
                    eprintln!("write_state: failed to rename {} -> {}: {}", tmp.display(), path.display(), e);
                }
            }
        }
    }

    /// Log status line (match Python: "Status: (N clients M bad sync) (o outlier%) (m mlat s sync t tracked)")
    async fn log_status(&self) {
        let (num_receivers, bad_receivers, outlier_pct, ac_mlat, ac_sync, ac_total) = {
            let receivers = self.receivers.read().await;
            let tracker = self.tracker.read().await;
            let bad_receivers = receivers.values().filter(|r| r.bad_syncs > 0.0).count();
            let outlier_sum: f64 = receivers.values().map(|r| r.outlier_percent_rolling).sum();
            let n = receivers.len();
            let outlier_pct = if n > 0 { outlier_sum / (n as f64) } else { 0.0 };
            (
                receivers.len(),
                bad_receivers,
                outlier_pct,
                tracker.num_mlat_wanted(),
                tracker.num_sync_interest(),
                tracker.num_aircraft(),
            )
        };
        let title_string = format!(
            "Status: ({} clients {} bad sync) ({:.2} outlier_percentage) ({} mlat {} sync {} tracked)",
            num_receivers, bad_receivers, outlier_pct, ac_mlat, ac_sync, ac_total
        );
        eprintln!("{}", title_string);
    }

    /// Run periodic tasks for the coordinator (refresh interest, status log, write_state when enabled).
    /// Also receives delayed cohort batches and processes them (MLAT_DELAY 0.9s).
    pub async fn run(&self) {
        use std::time::Duration;
        let mut cohort_rx = match self.cohort_ready_rx.lock().unwrap().take() {
            Some(r) => r,
            None => return, // run() already called or not using cohort channel
        };
        let mut ticker = tokio::time::interval(Duration::from_millis(500));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let status_secs = self.status_interval_secs;
        let mut next_status = if status_secs > 0 {
            Some(
                tokio::time::Instant::now()
                    + Duration::from_secs(status_secs as u64),
            )
        } else {
            None
        };
        // Write state: first time after 2s (let clients connect), then every 5s so work_dir files stay updated
        let write_interval_secs = 5u64;
        let mut next_write_state = if !self.work_dir.is_empty() {
            Some(tokio::time::Instant::now() + Duration::from_secs(2))
        } else {
            None
        };
        // Clock pairing cleanup every 10s (Python _cleanup); decrement sync_peers for removed pairings
        let cleanup_interval_secs = 10u64;
        let mut next_cleanup = tokio::time::Instant::now() + Duration::from_secs(cleanup_interval_secs);
        loop {
            tokio::select! {
                msg = cohort_rx.recv() => match msg {
                    Some(icaos) => self.process_delayed_cohort(icaos).await,
                    None => break, // channel closed
                },
                tick = ticker.tick() => {
                    let _ = tick; // ignore Instant
                    self.refresh_interest().await;
                    if let Some(ref mut next) = next_status {
                        if tokio::time::Instant::now() >= *next {
                            *next += Duration::from_secs(status_secs as u64);
                            self.log_status().await;
                        }
                    }
                    if let Some(ref mut next) = next_write_state {
                        if tokio::time::Instant::now() >= *next {
                            *next += Duration::from_secs(write_interval_secs);
                            self.write_state().await;
                        }
                    }
                    if tokio::time::Instant::now() >= next_cleanup {
                        next_cleanup += Duration::from_secs(cleanup_interval_secs);
                        let now = std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_secs_f64();
                        let removed = {
                            let mut clock_tracker = self.clock_tracker.write().await;
                            clock_tracker.cleanup(now)
                        };
                        if !removed.is_empty() {
                            let mut receivers = self.receivers.write().await;
                            for (base_id, peer_id, cat) in removed {
                                if cat < 5 {
                                    if let Some(r) = receivers.get_mut(&base_id) {
                                        r.sync_peers[cat] = r.sync_peers[cat].saturating_sub(1);
                                    }
                                    if let Some(r) = receivers.get_mut(&peer_id) {
                                        r.sync_peers[cat] = r.sync_peers[cat].saturating_sub(1);
                                    }
                                }
                            }
                        }
                    }
                },
            }
        }
    }
}

impl Default for Coordinator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_coordinator_new() {
        let coordinator = Coordinator::new();
        assert_eq!(coordinator.receiver_count().await, 0);
    }
    
    #[tokio::test]
    async fn test_new_receiver() {
        let coordinator = Coordinator::new();

        let receiver_id = coordinator.new_receiver(
            "test_user".to_string(),
            37.5,
            -122.0,
            100.0,
            "dump1090",
            Some("test-uuid".to_string()),
            false,
            "test_user v2 dump1090 unknown tcp none".to_string(),
        ).await.unwrap();

        assert_eq!(receiver_id, 1);
        assert_eq!(coordinator.receiver_count().await, 1);
        
        let receiver = coordinator.get_receiver(receiver_id).await.unwrap();
        assert_eq!(receiver.user, "test_user");
        assert_eq!(receiver.uuid, Some("test-uuid".to_string()));
    }
    
    #[tokio::test]
    async fn test_multiple_receivers() {
        let coordinator = Coordinator::new();

        let id1 = coordinator.new_receiver(
            "user1".to_string(),
            37.5,
            -122.0,
            100.0,
            "dump1090",
            None,
            false,
            "user1 v2 dump1090 unknown tcp none".to_string(),
        ).await.unwrap();

        let id2 = coordinator.new_receiver(
            "user2".to_string(),
            38.0,
            -121.0,
            200.0,
            "radarcape_gps",
            None,
            true,
            "user2 v2 radarcape_gps unknown tcp none".to_string(),
        ).await.unwrap();

        assert_eq!(id1, 1);
        assert_eq!(id2, 2);
        assert_eq!(coordinator.receiver_count().await, 2);
    }

    #[tokio::test]
    async fn test_duplicate_user_rejected() {
        let coordinator = Coordinator::new();

        let id1 = coordinator.new_receiver(
            "same_user".to_string(),
            37.5,
            -122.0,
            100.0,
            "dump1090",
            None,
            false,
            "same_user v2 dump1090 unknown tcp none".to_string(),
        ).await.unwrap();
        assert_eq!(id1, 1);

        let r2 = coordinator.new_receiver(
            "same_user".to_string(),
            38.0,
            -121.0,
            200.0,
            "dump1090",
            None,
            false,
            "same_user v2 dump1090 unknown tcp none".to_string(),
        ).await;
        assert!(r2.is_err());
        assert!(r2.unwrap_err().contains("already connected"));
        assert_eq!(coordinator.receiver_count().await, 1);

        coordinator.receiver_disconnect(id1).await;
        let id3 = coordinator.new_receiver(
            "same_user".to_string(),
            38.0,
            -121.0,
            200.0,
            "dump1090",
            None,
            false,
            "same_user v2 dump1090 unknown tcp none".to_string(),
        ).await.unwrap();
        assert_eq!(id3, 2);
    }

    #[tokio::test]
    async fn test_handle_mlat() {
        let coordinator = Coordinator::new();

        let receiver_id = coordinator.new_receiver(
            "test".to_string(),
            37.5,
            -122.0,
            100.0,
            "dump1090",
            None,
            false,
            "test v2 dump1090 unknown tcp none".to_string(),
        ).await.unwrap();

        // Should not panic
        coordinator.handle_mlat(
            receiver_id,
            123456.789,
            "8D4840D58B...",
            1234567890.123,
        ).await;
    }
    
    #[tokio::test]
    async fn test_handle_sync() {
        let coordinator = Coordinator::new();

        let receiver_id = coordinator.new_receiver(
            "test".to_string(),
            37.5,
            -122.0,
            100.0,
            "dump1090",
            None,
            false,
            "test v2 dump1090 unknown tcp none".to_string(),
        ).await.unwrap();

        // Should not panic
        coordinator.handle_sync(
            receiver_id,
            123456.0,
            123457.0,
            "8D4840D6...",
            "8D4840D7...",
        ).await;
    }
}
