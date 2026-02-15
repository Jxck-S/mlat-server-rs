// MLAT message resolution
// Ported from mlat/mlattrack.py

use std::collections::{HashMap, HashSet};
use crate::modes::message::{self, ModeSMessage, DecodedMessage};
use crate::constants::{CAIR, FTOM, MAX_GROUP, RESOLVE_INTERVAL, RESOLVE_BACKOFF};

/// Result of process_message: coordinator should schedule delayed resolution.
/// Python: cohort groups resolved after MLAT_DELAY (0.9s).
#[derive(Debug, Clone)]
pub enum MlatAction {
    /// New group created; schedule resolution after MLAT_DELAY (0.9s).
    /// Contains the message bytes as the group key.
    Delayed(Vec<u8>),
}

const MAX_TIMESTAMP_DELTA: f64 = 2e-3; // Maximum timestamp difference for clustering (2ms)
const MIN_DISTINCT_DISTANCE: f64 = 1e3; // Minimum distance for distinct receivers (1km)

/// A single copy of a message from a receiver
#[derive(Debug, Clone)]
pub struct MessageCopy {
    pub receiver_id: usize,
    pub timestamp: f64,  // Receiver's local timestamp
    pub utc: f64,        // UTC time when received
}

/// A group of message copies (same message from multiple receivers)
#[derive(Debug)]
pub struct MessageGroup {
    pub message: Vec<u8>,
    pub first_seen: f64,
    pub copies: Vec<MessageCopy>,
    pub receivers: HashSet<usize>,
}

impl MessageGroup {
    pub fn new(message: Vec<u8>, first_seen: f64) -> Self {
        MessageGroup {
            message,
            first_seen,
            copies: Vec::new(),
            receivers: HashSet::new(),
        }
    }
    
    pub fn add_copy(&mut self, receiver_id: usize, timestamp: f64, utc: f64) {
        if self.copies.len() < MAX_GROUP {
            self.copies.push(MessageCopy {
                receiver_id,
                timestamp,
                utc,
            });
        }
        self.receivers.insert(receiver_id);
    }
}

/// A cluster of timestamp measurements from different receivers
#[derive(Debug, Clone)]
pub struct Cluster {
    pub distinct: usize,     // Number of distinct receivers
    pub first_seen: f64,     // Earliest UTC time in cluster
    pub measurements: Vec<Measurement>,
}

/// A single measurement in a cluster
#[derive(Debug, Clone)]
pub struct Measurement {
    pub receiver_id: usize,
    pub timestamp: f64,
    pub variance: f64,
}

/// Result of MLAT resolution
#[derive(Debug, Clone)]
pub struct MlatResult {
    pub icao: u32,
    pub timestamp: f64,
    pub position: [f64; 3],  // ECEF coordinates
    pub covariance: Option<[[f64; 3]; 3]>,
    pub distinct: usize,
    pub dof: usize,
    pub error: f64,  // meters
    pub receivers: Vec<usize>,
    /// Ground speed (m/s) from Kalman when available; used for SBS/CSV speed in kts.
    pub ground_speed: Option<f64>,
    /// Vertical speed (m/s) from Kalman when available; used for SBS/CSV vrate in fpm.
    pub vertical_speed: Option<f64>,
}

/// Cluster timestamps into groups that are probably copies of the same transmission
/// 
/// This implements the clustering algorithm from Python mlattrack.py lines 425-542
/// 
/// # Arguments
/// * `component` - Normalized timestamps from clock tracker: receiver_id -> (variance, [(timestamp, utc), ...])
/// * `min_receivers` - Minimum number of distinct receivers needed
/// * `receiver_distances` - Distance matrix: (receiver_id, receiver_id) -> distance in meters
/// 
/// # Returns
/// Vector of clusters, each containing distinct receivers, first seen time, and measurements
pub fn cluster_timestamps(
    component: &HashMap<usize, (f64, Vec<(f64, f64)>)>,
    min_receivers: usize,
    receiver_distances: &HashMap<(usize, usize), f64>,
) -> Vec<Cluster> {
    // Flatten component into list of (receiver, timestamp, variance, utc)
    let mut flat_component: Vec<(usize, f64, f64, f64)> = Vec::new();
    
    for (&receiver_id, &(variance, ref timestamps)) in component.iter() {
        for &(timestamp, utc) in timestamps {
            flat_component.push((receiver_id, timestamp, variance, utc));
        }
    }
    
    // Sort by timestamp
    flat_component.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    
    // Rough grouping: groups of items with inter-item spacing < 2ms
    // This reduces O(n²) clustering to smaller groups
    let mut groups: Vec<Vec<(usize, f64, f64, f64)>> = Vec::new();
    
    if !flat_component.is_empty() {
        let mut current_group = vec![flat_component[0]];
        
        for &item in &flat_component[1..] {
            if item.1 - current_group.last().unwrap().1 > MAX_TIMESTAMP_DELTA {
                groups.push(current_group);
                current_group = vec![item];
            } else {
                current_group.push(item);
            }
        }
        groups.push(current_group);
    }
    
    // Fine clustering: form clusters greedily from each group
    let mut clusters = Vec::new();
    
    for mut group in groups {
        while group.len() >= min_receivers {
            // Start cluster from latest timestamp (pop from end)
            let (receiver, timestamp, variance, utc) = group.pop().unwrap();
            
            let mut cluster_measurements = vec![Measurement {
                receiver_id: receiver,
                timestamp,
                variance,
            }];
            
            let last_timestamp = timestamp;
            let mut distinct_receivers = 1;
            let mut first_seen = utc;
            
            // Try to add earlier timestamps to cluster
            let mut i = group.len();
            while i > 0 {
                i -= 1;
                
                let (candidate_receiver, candidate_timestamp, candidate_variance, candidate_utc) = group[i];
                
                // Check if timestamp is too far from cluster
                if last_timestamp - candidate_timestamp > MAX_TIMESTAMP_DELTA {
                    break;  // Can't possibly be part of same cluster
                }
                
                // Check if this measurement can be added to cluster
                let mut can_cluster = true;
                let mut is_distinct = true;
                
                for measurement in &cluster_measurements {
                    // Check for duplicate receiver
                    if measurement.receiver_id == candidate_receiver {
                        can_cluster = false;
                        break;
                    }
                    
                    // Get distance between receivers
                    let distance = get_receiver_distance(
                        receiver_distances,
                        candidate_receiver,
                        measurement.receiver_id,
                    );
                    
                    // Check if timestamp difference is consistent with distance
                    let time_diff = (measurement.timestamp - candidate_timestamp).abs();
                    let max_time_diff = (distance * 1.05 + 1e3) / CAIR; // Cair = speed of light
                    
                    if time_diff > max_time_diff {
                        can_cluster = false;
                        break;
                    }
                    
                    // Check if receivers are close enough to be non-distinct
                    if distance < MIN_DISTINCT_DISTANCE {
                        is_distinct = false;
                    }
                }
                
                if can_cluster {
                    cluster_measurements.push(Measurement {
                        receiver_id: candidate_receiver,
                        timestamp: candidate_timestamp,
                        variance: candidate_variance,
                    });
                    
                    first_seen = first_seen.min(candidate_utc);
                    
                    if is_distinct {
                        distinct_receivers += 1;
                    }
                    
                    group.remove(i);
                }
            }
            
            // Only keep cluster if it has enough distinct receivers
            if distinct_receivers >= min_receivers {
                // Reverse to get ascending timestamps
                cluster_measurements.reverse();
                
                clusters.push(Cluster {
                    distinct: distinct_receivers,
                    first_seen,
                    measurements: cluster_measurements,
                });
            }
        }
    }
    
    clusters
}

/// Resolve MLAT position from a message group
///
/// This is the main entry point for MLAT resolution. It:
/// 1. Checks rate limiting
/// 2. Normalizes timestamps (stub for now)
/// 3. Clusters timestamps
/// 4. Tries clusters until a valid position is found
/// 5. Updates Kalman filter
/// 6. Returns MlatResult
///
/// # Arguments
/// * `message_group` - Group of message copies from multiple receivers
/// * `receivers` - Map of receiver ID to Receiver
/// * `aircraft` - Aircraft being tracked (mutable for state updates)
/// * `now` - Current UTC time
///
/// # Returns
/// Some(MlatResult) if position was successfully resolved, None otherwise
pub fn resolve(
    message_group: &MessageGroup,
    receivers: &HashMap<usize, crate::receiver::Receiver>,
    aircraft: &mut crate::tracker::TrackedAircraft,
    clock_tracker: &crate::clocktrack::ClockTracker,
    now: f64,
) -> Option<MlatResult> {
        // Per-failure-point counters
        static RESOLVE_ENTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let re = RESOLVE_ENTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let copies = message_group.copies.len();
        let rcvrs = message_group.receivers.len();
        macro_rules! resolve_fail {
            ($label:expr) => {{
                static C: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if n < 3 || (re < 500 && n % 50 == 0) || n % 500 == 0 {
                    eprintln!("resolve FAIL {}: copies={} rcvrs={} (#{} of {} entries)", $label, copies, rcvrs, n, re);
                }
                return None;
            }};
        }

        // Rate limiting is done by caller (try_resolve_pending_by_message).
        // Set last_resolve_attempt to now so next call respects the interval.
    aircraft.last_resolve_attempt = now;

    // Elapsed and backoff (Python: last_result_time with 120s reset, elapsed, RESOLVE_BACKOFF checks)
    let (last_result_dof, last_result_time) = match (aircraft.last_result_position, aircraft.last_result_time) {
        (None, _) | (_, None) => (0, message_group.first_seen - 120.0),
        (_, Some(t)) if message_group.first_seen - t > 120.0 => (0, message_group.first_seen - 120.0),
        _ => (aircraft.last_result_dof.unwrap_or(0), aircraft.last_result_time.unwrap()),
    };
    let mut elapsed = message_group.first_seen - last_result_time;
    if elapsed < 0.0 {
        elapsed = 0.0;
    }
    if elapsed < RESOLVE_BACKOFF {
        resolve_fail!("backoff");
    }
    
    // Get altitude and altitude DOF
    let (altitude, altitude_dof) = get_altitude_from_group(message_group, aircraft);
    let len_copies = message_group.copies.len();
    if len_copies + altitude_dof < 4 {
        resolve_fail!("copies+alt<4");
    }
    let max_dof = len_copies + altitude_dof - 4;
    if elapsed < 2.0 * RESOLVE_BACKOFF && (max_dof as f64) < (last_result_dof as f64 - elapsed + 0.5) {
        resolve_fail!("dof_backoff");
    }
    
    // Build timestamp map
    let mut timestamp_map: HashMap<usize, Vec<(f64, f64)>> = HashMap::new();
    for copy in &message_group.copies {
        timestamp_map.entry(copy.receiver_id)
            .or_insert_with(Vec::new)
            .push((copy.timestamp, copy.utc));
    }
    let dof_pre = timestamp_map.len() + altitude_dof;
    if dof_pre < 4 {
        resolve_fail!("ts_map<4");
    }
    let dof_pre = dof_pre - 4;
    if elapsed < 2.0 * RESOLVE_BACKOFF && (dof_pre as f64) < (last_result_dof as f64 - elapsed + 0.5) {
        resolve_fail!("ts_dof_backoff");
    }
    
    // Normalize timestamps using graph-based clock synchronization
    let receiver_clocks: HashMap<usize, crate::clocktrack::Clock> = timestamp_map.keys()
        .filter_map(|&rid| receivers.get(&rid).map(|r| (rid, r.clock.clone())))
        .collect();
    let components = clock_tracker.normalize2(&timestamp_map, &receiver_clocks);
    
    // Cluster timestamps
    let receiver_distances = build_distance_matrix(receivers);
    let min_receivers = 4 - altitude_dof;
    let mut clusters = Vec::new();
    
    for component in components {
        if component.len() >= min_receivers {
            clusters.extend(cluster_timestamps(&component, min_receivers, &receiver_distances));
        }
    }
    
    if clusters.is_empty() {
        resolve_fail!("no_clusters");
    }

    {
        static CLUSTER_OK: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let ck = CLUSTER_OK.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if ck < 5 || ck % 500 == 0 {
            let total_m: usize = clusters.iter().map(|c| c.measurements.len()).sum();
            let max_d = clusters.iter().map(|c| c.distinct).max().unwrap_or(0);
            eprintln!("resolve: got {} clusters, max_distinct={}, total_meas={}, altitude={:?}, alt_dof={}, elapsed={:.1} (#{}))",
                clusters.len(), max_d, total_m, altitude.map(|a| (a * 3.28084) as i32), altitude_dof, elapsed, ck);
        }
    }
    
    // Sort clusters (most recent, largest first)
    clusters.sort_by(|a, b| {
        (a.distinct, a.first_seen)
            .partial_cmp(&(b.distinct, b.first_seen))
            .unwrap()
    });
    
    // Try clusters until we get a result
    while let Some(cluster) = clusters.pop() {
        let dof = (cluster.distinct + altitude_dof).saturating_sub(4);
        
        // Python line 280: elapsed = cluster_utc - last_result_time
        // Use the local `last_result_time` (which has synthetic value first_seen-120
        // for first results), NOT aircraft.last_result_time directly.
        let cluster_elapsed = cluster.first_seen - last_result_time;

        // Python line 283-284: dof backoff check
        if cluster_elapsed < 2.0 && (dof as f64) < (last_result_dof as f64 - cluster_elapsed + 0.5) {
            resolve_fail!("cluster_dof_backoff");
        }

        // Skip if DOF == 0 and stale (Python line 300-301)
        if cluster_elapsed > 30.0 && dof == 0 {
            continue;
        }
        
        // Estimate altitude error
        let altitude_error = estimate_altitude_error(altitude, aircraft, cluster.first_seen);
        
        // Python lines 303-306: initial guess selection
        // When elapsed < 60 and we have a prior result, use it
        // When elapsed >= 60 (including first-ever result), use first receiver's position
        let initial_guess = if cluster_elapsed < 60.0 {
            // Use Kalman or last result if available
            if aircraft.kalman.valid {
                aircraft.kalman.position
            } else {
                aircraft.last_result_position
            }
        } else {
            // No recent result: use first receiver's position (Python: cluster[0][0].position)
            cluster.measurements.first()
                .and_then(|m| receivers.get(&m.receiver_id))
                .map(|r| r.position)
        };
        
        // Build measurements for solver
        let measurements: Vec<crate::solver::Measurement> = cluster.measurements.iter()
            .filter_map(|m| {
                receivers.get(&m.receiver_id).map(|r| crate::solver::Measurement {
                    receiver_position: r.position,
                    timestamp: m.timestamp,
                    variance: m.variance,
                })
            })
            .collect();
        
        if measurements.len() < min_receivers {
            continue;
        }
        
        // Solve for position
        let has_guess = initial_guess.is_some();
        let solve_result = solve_position(
            &measurements,
            altitude,
            altitude_error,
            initial_guess,
        );

        if solve_result.is_none() {
            static SOLVE_FAIL: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let sf = SOLVE_FAIL.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if sf < 10 || sf % 200 == 0 {
                eprintln!("resolve: solver returned None, distinct={} meas={} alt={:?} guess={} (#{}))",
                    cluster.distinct, measurements.len(), altitude.is_some(), has_guess, sf);
            }
        }
        
        if let Some((ecef, ecef_cov)) = solve_result {
            // Estimate error (Python: var_est = numpy.trace(ecef_cov), error = sqrt(abs(var_est)))
            let var = if let Some(cov) = &ecef_cov {
                // Python uses trace (sum of diagonal), NOT trace/3
                (cov[(0, 0)] + cov[(1, 1)] + cov[(2, 2)]).abs()
            } else {
                // Python: ecef_cov is None -> continue (reject suspect result)
                continue;
            };
            
            let error = var.sqrt();
            let max_error = 10000.0;
            if error > max_error {
                continue;  // Try next cluster (max_error = 10km)
            }

            // Frequency capping: the higher the accuracy, the higher the frequency of positions that is output
            // Python line 338: elapsed / 20 < error / max_error  (uses cluster-level elapsed)
            if cluster_elapsed / 20.0 < error / max_error {
                continue;
            }
            
            // Update Kalman filter
            update_kalman(
                aircraft, 
                ecef, 
                ecef_cov.clone(), 
                cluster.first_seen, 
                altitude,
                estimate_altitude_error(altitude, aircraft, cluster.first_seen),
                &cluster.measurements,
                receivers,
                dof
            );
            
            // Update aircraft state
            aircraft.last_result_position = Some(ecef);
            aircraft.last_result_var = Some(var);
            aircraft.last_result_dof = Some(dof);
            aircraft.last_result_time = Some(cluster.first_seen);
            aircraft.mlat_result_count += 1;
            
            {
                static SOLVE_OK: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                let sok = SOLVE_OK.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if sok < 20 || sok % 100 == 0 {
                    let (lat, lon, alt) = crate::geodesy::ecef2llh(ecef[0], ecef[1], ecef[2]);
                    eprintln!("MLAT RESULT: icao={:06x} lat={:.4} lon={:.4} alt={:.0}ft err={:.0}m distinct={} dof={} (#{}))",
                        aircraft.icao, lat, lon, alt * 3.28084, error, cluster.distinct, dof, sok);
                }
            }

            // Return result (include Kalman speed/vrate for SBS/CSV output)
            return Some(MlatResult {
                icao: aircraft.icao,
                timestamp: cluster.first_seen,
                position: ecef,
                covariance: ecef_cov.as_ref().map(|cov| {
                    [
                        [cov[(0, 0)], cov[(0, 1)], cov[(0, 2)]],
                        [cov[(1, 0)], cov[(1, 1)], cov[(1, 2)]],
                        [cov[(2, 0)], cov[(2, 1)], cov[(2, 2)]],
                    ]
                }),
                distinct: cluster.distinct,
                dof,
                error,
                receivers: cluster.measurements.iter().map(|m| m.receiver_id).collect(),
                ground_speed: aircraft.kalman.ground_speed,
                vertical_speed: aircraft.kalman.vertical_speed,
            });
        }
    }
    
    None
}

/// Get altitude and altitude DOF from message group
///
/// Returns (altitude in meters, altitude_dof)
/// - altitude_dof = 1 if altitude in message
/// - altitude_dof = 0 if using recent aircraft altitude
/// - altitude_dof = 0 if no altitude available
fn get_altitude_from_group(
    message_group: &MessageGroup,
    aircraft: &crate::tracker::TrackedAircraft,
) -> (Option<f64>, usize) {
    // Try to decode message to get altitude
    if let Some(msg) = message::decode(&message_group.message) {
        let alt_opt = match msg {
            DecodedMessage::DF0(m) => m.altitude(),
            DecodedMessage::DF4(m) => m.altitude(),
            DecodedMessage::DF5(m) => m.altitude(),
            DecodedMessage::DF11(m) => m.altitude(),
            DecodedMessage::DF16(m) => m.altitude(),
            DecodedMessage::DF17(m) => {
                 // DF17 altitude is Option<u32> in struct, but trait returns Option<i32>
                 // We need to check use m.altitude directly or via trait method if imported?
                 // Let's use m.altitude which is Option<u32> and cast
                 m.altitude.map(|a| a as i32) 
            },
            DecodedMessage::DF18(m) => m.altitude.map(|a| a as i32), // DF18 similarly
            DecodedMessage::DF20(m) => m.altitude(),
            DecodedMessage::DF21(m) => m.altitude(),
        };

        if let Some(alt_ft) = alt_opt {
            // Python: decoded.altitude > -1500 and decoded.altitude < 75000
            if alt_ft > -1500 && alt_ft < 75000 {
                return (Some(alt_ft as f64 * FTOM), 1);
            }
        }
    }

    // Use aircraft's last known altitude if recent (Python: last_altitude_time + 45 > first_seen)
    // Also check altitude range (Python: MIN_ALT < altitude < MAX_ALT)
    if let Some(alt_ft) = aircraft.altitude {
        if alt_ft > crate::constants::MIN_ALT_M / FTOM && alt_ft < crate::constants::MAX_ALT_M / FTOM {
            if let Some(last_alt_time) = aircraft.last_altitude_time {
                if message_group.first_seen - last_alt_time < 45.0 {
                    return (Some(alt_ft * FTOM), 0);
                }
            }
        }
    }
    
    (None, 0)
}

/// Estimate altitude error in meters
fn estimate_altitude_error(
    altitude: Option<f64>,
    aircraft: &crate::tracker::TrackedAircraft,
    cluster_utc: f64,
) -> Option<f64> {
    if altitude.is_some() {
        // Fresh altitude from message: 250ft error
        Some(250.0 * FTOM)
    } else if aircraft.altitude.is_some() {
        if let Some(last_alt_time) = aircraft.last_altitude_time {
            // Degrade over time at ~4000fpm
            let age = cluster_utc - last_alt_time;
            Some((250.0 + age * 70.0) * FTOM)
        } else {
            None
        }
    } else {
        None
    }
}

/// Get initial guess for solver (reserved for alternative code paths).
#[allow(dead_code)]
fn get_initial_guess(
    aircraft: &crate::tracker::TrackedAircraft,
    elapsed: f64,
    dof: isize,
) -> Option<[f64; 3]> {
    if elapsed > 30.0 && dof == 0 {
        None
    } else if aircraft.kalman.valid {
        aircraft.kalman.position
    } else {
        aircraft.last_result_position
    }
}

/// Build distance matrix from receivers
fn build_distance_matrix(
    receivers: &HashMap<usize, crate::receiver::Receiver>,
) -> HashMap<(usize, usize), f64> {
    let mut distances = HashMap::new();
    
    for (&uid1, r1) in receivers {
        for (&uid2, r2) in receivers {
            if uid1 != uid2 {
                let dist = crate::receiver::ecef_distance(r1.position, r2.position);
                distances.insert((uid1, uid2), dist);
            }
        }
    }
    
    distances
}



/// Solve for position
fn solve_position(
    measurements: &[crate::solver::Measurement],
    altitude: Option<f64>,
    altitude_error: Option<f64>,
    initial_guess: Option<[f64; 3]>,
) -> Option<([f64; 3], Option<nalgebra::DMatrix<f64>>)> {
    use crate::solver::solve;
    
    let start_pos = initial_guess.unwrap_or([0.0, 0.0, 0.0]);
    
    match solve(measurements, altitude, altitude_error, &start_pos) {
        Some(solution) => Some((solution.position, solution.covariance)),
        None => None,
    }
}

/// Update Kalman filter
fn update_kalman(
    aircraft: &mut crate::tracker::TrackedAircraft,
    ecef: [f64; 3],
    cov: Option<nalgebra::DMatrix<f64>>,
    timestamp: f64,
    altitude: Option<f64>,
    altitude_error: Option<f64>,
    measurements: &[Measurement],
    receivers: &HashMap<usize, crate::receiver::Receiver>,
    dof: usize,
) {
    use nalgebra::{DMatrix, DVector};
    
    // Convert covariance to DMatrix
    let ls_cov = if let Some(c) = cov {
        c
    } else {
        // Fallback large covariance if none
        DMatrix::from_diagonal(&DVector::from_element(3, 1e6))
    };
    
    // Prepare measurements list for Kalman: (position, timestamp, variance)
    // We need to look up receiver positions
    let mut k_measurements = Vec::new();
    for m in measurements {
        if let Some(r) = receivers.get(&m.receiver_id) {
             k_measurements.push((r.position, m.timestamp, m.variance));
        }
    }
    
    if k_measurements.is_empty() {
        return;
    }
    
    // Call Kalman update
    aircraft.kalman.update(
        timestamp,
        &k_measurements,
        altitude,
        altitude_error,
        &ecef,
        &ls_cov,
        dof
    );
}

/// Get distance between two receivers from distance matrix
fn get_receiver_distance(
    distances: &HashMap<(usize, usize), f64>,
    r1: usize,
    r2: usize,
) -> f64 {
    distances.get(&(r1, r2))
        .or_else(|| distances.get(&(r2, r1)))
        .copied()
        .unwrap_or(0.0)
}


/// MLAT Tracker
/// Python: self.pending = {} keyed by message bytes; each unique physical transmission gets a group.
pub struct MlatTracker {
    /// Buffered messages: message_bytes -> MessageGroup
    /// Python: self.pending[message] = MessageGroup(message=message, first_seen=now)
    pending: HashMap<Vec<u8>, MessageGroup>,
}

impl MlatTracker {
    /// Create a new MLAT tracker
    pub fn new() -> Self {
        MlatTracker {
            pending: HashMap::new(),
        }
    }
    
    /// Number of pending groups
    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }

    /// Process a message from a receiver.
    /// Python: receiver_mlat. Uses message bytes as key (not ICAO).
    /// Multiple receivers reporting the same physical message get added to the same group.
    /// Returns Delayed(message_bytes) for new groups so coordinator can schedule resolution.
    pub fn process_message(
        &mut self,
        receiver_id: usize,
        message: Vec<u8>,
        timestamp: f64,
    ) -> Option<MlatAction> {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        // Python: group = self.pending.get(message)
        if let Some(group) = self.pending.get_mut(&message) {
            // Existing group: add this receiver's copy
            // Python: group.receivers.add(receiver); group.copies.append((receiver, timestamp, now))
            let is_new_receiver = !group.receivers.contains(&receiver_id);
            group.receivers.insert(receiver_id);
            if group.copies.len() < MAX_GROUP {
                group.copies.push(MessageCopy {
                    receiver_id,
                    timestamp,
                    utc: now,
                });
            }
            if is_new_receiver {
                static MATCH_COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                let n = MATCH_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if n < 5 || n % 500 == 0 {
                    eprintln!("mlattrack: MATCH! msg_len={} receivers={} copies={} (#{}))", message.len(), group.receivers.len(), group.copies.len(), n);
                }
            }
            // Only schedule resolution when we have at least 2 receivers (avoid ts_map<4 / single-receiver fails)
            if is_new_receiver && group.receivers.len() >= 2 {
                return Some(MlatAction::Delayed(message.clone()));
            }
            return None;
        }

        // New group for this message (single receiver – do not schedule yet; wait for 2nd receiver)
        // Python: group = self.pending[message] = MessageGroup(message=message, first_seen=now)
        let mut group = MessageGroup::new(message.clone(), now);
        group.receivers.insert(receiver_id);
        group.copies.push(MessageCopy {
            receiver_id,
            timestamp,
            utc: now,
        });
        self.pending.insert(message.clone(), group);

        static NEW_GROUP_COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = NEW_GROUP_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if n < 3 || n % 5000 == 0 {
            eprintln!("mlattrack: new group msg_len={} pending={} (#{}))", message.len(), self.pending.len(), n);
        }

        // Don't schedule yet – schedule when 2nd receiver adds (see existing-group path above)
        None
    }

    /// Resolve a pending group for the given ICAO (cohort delivers icaos; we find one matching message).
    /// Python: _resolve(group) called from cohort._process() for each icao in cohort.
    pub fn try_resolve_pending(
        &mut self,
        icao: u32,
        receivers: &HashMap<usize, crate::receiver::Receiver>,
        tracker: &mut crate::tracker::Tracker,
        clock_tracker: &crate::clocktrack::ClockTracker,
        now: f64,
    ) -> Option<MlatResult> {
        let key = self.pending.iter().find_map(|(k, _)| {
            crate::modes::message::decode(k).and_then(|m| {
                if m.as_trait().address() == icao {
                    Some(k.clone())
                } else {
                    None
                }
            })
        })?;
        self.try_resolve_pending_by_message(&key, receivers, clock_tracker, tracker, now)
    }

    /// Resolve a pending group by message key after cohort delay.
    /// Python: _resolve(group) called from cohort._process(), del self.pending[group.message]
    pub fn try_resolve_pending_by_message(
        &mut self,
        message_key: &[u8],
        receivers: &HashMap<usize, crate::receiver::Receiver>,
        clock_tracker: &crate::clocktrack::ClockTracker,
        tracker: &mut crate::tracker::Tracker,
        now: f64,
    ) -> Option<MlatResult> {
        let group = self.pending.remove(message_key)?;

        static RESOLVE_COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let n = RESOLVE_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let rcvrs = group.receivers.len();
        let cps = group.copies.len();
        if n < 5 || n % 2000 == 0 || rcvrs >= 3 {
            eprintln!("mlattrack: try_resolve copies={} receivers={} age={:.3}s msg_key_len={} (#{}))",
                cps, rcvrs, now - group.first_seen, message_key.len(), n);
        }

        // Python: less than 3 messages -> no go
        if cps < 3 {
            return None;
        }

        // Decode message to get aircraft address
        let decoded = crate::modes::message::decode(&group.message)?;
        let traj = decoded.as_trait();
        let icao = traj.address();

        let aircraft = match tracker.get_mut(icao) {
            Some(a) => a,
            None => return None,
        };
        aircraft.seen = now;
        aircraft.mlat_message_count += 1;

        // Update aircraft metadata from message using trait methods
        let traj_ref = decoded.as_trait();
        if let Some(alt) = traj_ref.altitude() {
            aircraft.altitude = Some(alt.into());
            aircraft.last_altitude_time = Some(group.first_seen);
        }
        if let Some(sq) = traj_ref.squawk() {
            aircraft.squawk = u16::from_str_radix(sq, 8).ok().or_else(|| sq.parse().ok());
        }
        if let Some(cs) = traj_ref.callsign() {
            aircraft.callsign = Some(cs.to_string());
        }

        // Python: check resolve interval (don't resolve too frequently)
        // NOTE: resolve() also checks this, so we do it HERE and let resolve() skip its check.
        if now - aircraft.last_resolve_attempt < RESOLVE_INTERVAL {
            static RI_BLOCKED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let b = RI_BLOCKED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if rcvrs >= 3 && (b < 10 || b % 1000 == 0) {
                eprintln!("RESOLVE_INTERVAL blocked: icao={:06x} rcvrs={} copies={} age={:.3}s (#{}))",
                    icao, rcvrs, cps, now - group.first_seen, b);
            }
            return None;
        }
        // Don't set last_resolve_attempt here; let resolve() set it after successful resolution.

        let result = resolve(&group, receivers, aircraft, clock_tracker, now)?;
        // Verify aircraft state was updated
        eprintln!("try_resolve: RESULT icao={:06x} mlat_result_count={} has_last_pos={} has_last_time={}",
            icao, aircraft.mlat_result_count,
            aircraft.last_result_position.is_some(),
            aircraft.last_result_time.is_some());
        Some(result)
    }

    /// Get receiver count for a pending group (for sorting cohort by best groups first).
    pub fn pending_receiver_count(&self, msg_key: &[u8]) -> usize {
        self.pending.get(msg_key).map(|g| g.receivers.len()).unwrap_or(0)
    }

    /// Clean up stale pending groups (older than 5 seconds)
    pub fn cleanup_stale(&mut self, now: f64) {
        self.pending.retain(|_, group| now - group.first_seen < 5.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_message_group() {
        let mut group = MessageGroup::new(vec![0x8D, 0x40, 0x62, 0x1D], 1000.0);
        
        assert_eq!(group.copies.len(), 0);
        assert_eq!(group.receivers.len(), 0);
        
        group.add_copy(1, 100.0, 1000.0);
        group.add_copy(2, 100.1, 1000.0);
        
        assert_eq!(group.copies.len(), 2);
        assert_eq!(group.receivers.len(), 2);
        assert!(group.receivers.contains(&1));
        assert!(group.receivers.contains(&2));
    }
    
    #[test]
    fn test_message_group_max_size() {
        let mut group = MessageGroup::new(vec![0x8D], 1000.0);
        
        // Add MAX_GROUP + 10 copies
        for i in 0..(MAX_GROUP + 10) {
            group.add_copy(i, 100.0, 1000.0);
        }
        
        // Should only keep MAX_GROUP copies
        assert_eq!(group.copies.len(), MAX_GROUP);
        // But track all receivers
        assert_eq!(group.receivers.len(), MAX_GROUP + 10);
    }
    
    #[test]
    fn test_cluster_simple() {
        // Create a simple component with 3 receivers seeing the same message
        let mut component = HashMap::new();
        
        // All timestamps very close (within 1ms)
        component.insert(1, (1e-12, vec![(100.000, 1000.0)]));
        component.insert(2, (1e-12, vec![(100.001, 1000.0)]));
        component.insert(3, (1e-12, vec![(100.002, 1000.0)]));
        
        // Create distance matrix (all receivers 10km apart)
        let mut distances = HashMap::new();
        distances.insert((1, 2), 10000.0);
        distances.insert((1, 3), 10000.0);
        distances.insert((2, 3), 10000.0);
        
        let clusters = cluster_timestamps(&component, 3, &distances);
        
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].distinct, 3);
        assert_eq!(clusters[0].measurements.len(), 3);
    }
    
    #[test]
    fn test_cluster_duplicate_receiver() {
        // Component with duplicate receiver (should be filtered)
        let mut component = HashMap::new();
        component.insert(1, (1e-12, vec![(100.000, 1000.0), (100.001, 1000.0)]));
        component.insert(2, (1e-12, vec![(100.0005, 1000.0)]));
        
        let mut distances = HashMap::new();
        distances.insert((1, 2), 10000.0);
        
        let clusters = cluster_timestamps(&component, 2, &distances);
        
        // Should form cluster with receiver 1 (one copy) and receiver 2
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].measurements.len(), 2);
        assert_eq!(clusters[0].distinct, 2);
    }
    
    #[test]
    fn test_cluster_non_distinct_receivers() {
        // Receivers very close together (< 1km) should count as non-distinct
        let mut component = HashMap::new();
        component.insert(1, (1e-12, vec![(100.000, 1000.0)]));
        component.insert(2, (1e-12, vec![(100.001, 1000.0)]));
        component.insert(3, (1e-12, vec![(100.002, 1000.0)]));
        
        let mut distances = HashMap::new();
        distances.insert((1, 2), 500.0);   // < 1km, non-distinct
        distances.insert((1, 3), 10000.0); // > 1km, distinct
        distances.insert((2, 3), 10000.0); // > 1km, distinct
        
        let clusters = cluster_timestamps(&component, 2, &distances);
        
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].measurements.len(), 3);
        // Only 2 distinct (receivers 1 and 2 are too close)
        assert_eq!(clusters[0].distinct, 2);
    }
    
    #[test]
    fn test_cluster_distance_filter() {
        // Timestamps inconsistent with distance should be filtered
        let mut component = HashMap::new();
        component.insert(1, (1e-12, vec![(100.000, 1000.0)]));
        component.insert(2, (1e-12, vec![(100.100, 1000.0)])); // 100ms difference!
        
        let mut distances = HashMap::new();
        distances.insert((1, 2), 10000.0); // Only 10km apart
        
        // 100ms * 343 m/s = 34.3km, but receivers only 10km apart
        // This should be rejected
        
        let clusters = cluster_timestamps(&component, 2, &distances);
        
        // Should not form a valid cluster
        assert_eq!(clusters.len(), 0);
    }
    
    #[test]
    fn test_get_altitude_from_group() {
        use crate::tracker::TrackedAircraft;
        
        // Test with recent altitude
        let mut aircraft = TrackedAircraft::new(0x123456, true, 1000.0);
        aircraft.altitude = Some(10000.0);  // 10,000 ft
        aircraft.last_altitude_time = Some(1000.0);
        
        let group = MessageGroup::new(vec![0x8D], 1000.0);
        let (alt, dof) = get_altitude_from_group(&group, &aircraft);
        
        assert!(alt.is_some());
        assert_eq!(dof, 0);  // Using aircraft altitude, not from message
        assert!((alt.unwrap() - 10000.0 * FTOM).abs() < 0.1);
    }
    
    #[test]
    fn test_estimate_altitude_error() {
        use crate::tracker::TrackedAircraft;
        
        let mut aircraft = TrackedAircraft::new(0x123456, true, 1000.0);
        
        // Fresh altitude from message
        let error1 = estimate_altitude_error(Some(3000.0), &aircraft, 1000.0);
        assert!(error1.is_some());
        assert!((error1.unwrap() - 250.0 * FTOM).abs() < 0.1);
        
        // Aged altitude from aircraft
        aircraft.altitude = Some(10000.0);
        aircraft.last_altitude_time = Some(1000.0);
        let error2 = estimate_altitude_error(None, &aircraft, 1010.0);  // 10 seconds old
        assert!(error2.is_some());
        // Should be 250 + 10*70 = 950 feet
        assert!((error2.unwrap() - 950.0 * FTOM).abs() < 1.0);
    }
    
    #[test]
    fn test_get_initial_guess() {
        use crate::tracker::TrackedAircraft;
        
        let mut aircraft = TrackedAircraft::new(0x123456, true, 1000.0);
        
        // No guess if elapsed > 30 and dof == 0
        let guess1 = get_initial_guess(&aircraft, 35.0, 0);
        assert!(guess1.is_none());
        
        // Use Kalman if valid
        aircraft.kalman.valid = true;
        aircraft.kalman.position = Some([1000.0, 2000.0, 3000.0]);
        let guess2 = get_initial_guess(&aircraft, 10.0, 1);
        assert_eq!(guess2, Some([1000.0, 2000.0, 3000.0]));
        
        // Use last result if Kalman not valid
        aircraft.kalman.valid = false;
        aircraft.last_result_position = Some([4000.0, 5000.0, 6000.0]);
        let guess3 = get_initial_guess(&aircraft, 10.0, 1);
        assert_eq!(guess3, Some([4000.0, 5000.0, 6000.0]));
    }
    
    #[test]
    fn test_build_distance_matrix() {
        use crate::receiver::Receiver;
        use std::collections::HashMap;
        
        let mut receivers = HashMap::new();
        
        let r1 = Receiver::new(
            1, "r1".to_string(), None, "dump1090",
            [37.0, -122.0, 0.0], false, "r1".to_string(), 0.0, false
        );
        let r2 = Receiver::new(
            2, "r2".to_string(), None, "dump1090",
            [38.0, -122.0, 0.0], false, "r2".to_string(), 0.0, false
        );
        
        receivers.insert(1, r1);
        receivers.insert(2, r2);
        
        let distances = build_distance_matrix(&receivers);
        
        // Should have 2 entries (1->2 and 2->1)
        assert_eq!(distances.len(), 2);
        assert!(distances.contains_key(&(1, 2)));
        assert!(distances.contains_key(&(2, 1)));
        
        // Distances should be equal
        assert_eq!(distances[&(1, 2)], distances[&(2, 1)]);
        assert!(distances[&(1, 2)] > 0.0);
    }
    

}
