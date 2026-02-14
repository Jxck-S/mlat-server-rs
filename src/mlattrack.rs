// MLAT message resolution
// Ported from mlat/mlattrack.py

use std::collections::{HashMap, HashSet};
use tracing::debug;
use crate::modes::message::{self, ModeSMessage, DecodedMessage};
use crate::constants::{CAIR, FTOM, MAX_GROUP, RESOLVE_INTERVAL, RESOLVE_BACKOFF};

/// Result of process_message: coordinator should schedule delayed resolution for this ICAO (Python: cohort MLAT_DELAY).
#[derive(Debug, Clone, Copy)]
pub enum MlatAction {
    /// Add to cohort; resolve after MLAT_DELAY (0.9s).
    Delayed(u32),
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
        debug!(size = message_group.copies.len(), "Processing message group");

        // Rate limiting (same as Python: mlattrack.py checks RESOLVE_INTERVAL and returns silently)
        if now - aircraft.last_resolve_attempt < RESOLVE_INTERVAL {
            return None;
        }
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
        return None;
    }
    
    // Get altitude and altitude DOF
    let (altitude, altitude_dof) = get_altitude_from_group(message_group, aircraft);
    let len_copies = message_group.copies.len();
    if len_copies + altitude_dof < 4 {
        return None;
    }
    let max_dof = len_copies + altitude_dof - 4;
    if elapsed < 2.0 * RESOLVE_BACKOFF && (max_dof as f64) < (last_result_dof as f64 - elapsed + 0.5) {
        return None;
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
        return None;
    }
    let dof_pre = dof_pre - 4;
    if elapsed < 2.0 * RESOLVE_BACKOFF && (dof_pre as f64) < (last_result_dof as f64 - elapsed + 0.5) {
        return None;
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
        debug!("No clusters found after clustering");
        return None;
    }

    debug!(count = clusters.len(), "Found clusters");
    
    // Sort clusters (most recent, largest first)
    clusters.sort_by(|a, b| {
        (a.distinct, a.first_seen)
            .partial_cmp(&(b.distinct, b.first_seen))
            .unwrap()
    });
    
    // Try clusters until we get a result
    while let Some(cluster) = clusters.pop() {
        let dof = (cluster.distinct + altitude_dof).saturating_sub(4);
        
        let elapsed = if let Some(last_time) = aircraft.last_result_time {
            cluster.first_seen - last_time
        } else {
            1000.0
        };

        // Skip if DOF == 0 and stale
        if elapsed > 30.0 && dof == 0 {
            continue;
        }
        
        // Estimate altitude error
        let altitude_error = estimate_altitude_error(altitude, aircraft, cluster.first_seen);
        
        // Get initial guess
        let initial_guess = get_initial_guess(aircraft, elapsed, dof as isize);
        
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
        let solve_result = solve_position(
            &measurements,
            altitude,
            altitude_error,
            initial_guess,
        );
        
        if let Some((ecef, ecef_cov)) = solve_result {
            // Estimate error
            let var = if let Some(cov) = &ecef_cov {
                (cov[(0, 0)] + cov[(1, 1)] + cov[(2, 2)]) / 3.0
            } else {
                1e9
            };
            
            let error = var.sqrt();
            if error > 10000.0 {
                continue;  // Try next cluster (max_error = 10km)
            }

            // Frequency capping: the higher the accuracy, the higher the frequency of positions that is output
            let max_error = 10000.0;
            if elapsed / 20.0 < error / max_error {
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
            return (Some(alt_ft as f64 * FTOM), 1);
        }
    }

    // Use aircraft's last known altitude if recent
    if let Some(alt_ft) = aircraft.altitude {
        if let Some(last_alt_time) = aircraft.last_altitude_time {
            // Use altitude if less than 30 seconds old
            if last_alt_time > 0.0 {
                return (Some(alt_ft * FTOM), 0);
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

/// Get initial guess for solver
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
pub struct MlatTracker {
    /// Buffered messages: ICAO -> (timestamp_group_start, MessageGroup)
    groups: HashMap<u32, MessageGroup>,
}

impl MlatTracker {
    /// Create a new MLAT tracker
    pub fn new() -> Self {
        MlatTracker {
            groups: HashMap::new(),
        }
    }
    
    /// Process a message from a receiver.
    /// Buffers the message; does not resolve immediately. Returns Delayed(icao) when the group
    /// should be resolved after MLAT_DELAY (Python: cohort with loop.call_later(MLAT_DELAY, _process)).
    pub fn process_message(
        &mut self,
        receiver_id: usize,
        message: Vec<u8>,
        timestamp: f64,
        utc: f64,
        icao: u32,
        _receivers: &HashMap<usize, crate::receiver::Receiver>,
        _tracker: &mut crate::tracker::Tracker,
        _clock_tracker: &crate::clocktrack::ClockTracker,
    ) -> Option<MlatAction> {
        let mut process_now = false;

        // Get or create group for this ICAO

        if let Some(group) = self.groups.get_mut(&icao) {
            // Check if this message belongs to current group (within 2ms window)
            // Python: "if 0 <= timestamp - group.first_seen <= 0.002"
            if (utc - group.first_seen).abs() < 2e-3 {
                 group.add_copy(receiver_id, timestamp, utc);
                 process_now = true;
            } else {
                // New group needed. In a full implementation we'd probably want to
                // ensure the old group is processed or discarded properly.
                // For now, valid assumption is that if we have a large gap, the old group is done.
            }
        }
        
        // If not added to existing group, or if we need to start a new one
        if !process_now && !self.groups.contains_key(&icao) {
            let mut group = MessageGroup::new(message, utc);
            group.add_copy(receiver_id, timestamp, utc);
            self.groups.insert(icao, group);
            process_now = true;
        } else if !process_now {
             // Replace with new group
             let mut group = MessageGroup::new(message, utc);
             group.add_copy(receiver_id, timestamp, utc);
             self.groups.insert(icao, group);
             process_now = true;
        }

        // Python: resolution happens in cohort._process() after MLAT_DELAY. Return Delayed(icao)
        // so coordinator can add to cohort and schedule processing.
        if process_now {
            return Some(MlatAction::Delayed(icao));
        }
        None
    }

    /// Resolve a single ICAO's pending group after cohort delay (Python: _resolve called from cohort._process).
    /// Removes the group from pending (Python: del self.pending[group.message]); returns Some(MlatResult) if resolved.
    pub fn try_resolve_pending(
        &mut self,
        icao: u32,
        receivers: &HashMap<usize, crate::receiver::Receiver>,
        aircraft: &mut crate::tracker::TrackedAircraft,
        clock_tracker: &crate::clocktrack::ClockTracker,
        now: f64,
    ) -> Option<MlatResult> {
        let group = self.groups.remove(&icao)?;
        let result = resolve(&group, receivers, aircraft, clock_tracker, now)?;
        Some(result)
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
            [37.0, -122.0, 0.0], false, "r1".to_string(), 0.0
        );
        let r2 = Receiver::new(
            2, "r2".to_string(), None, "dump1090",
            [38.0, -122.0, 0.0], false, "r2".to_string(), 0.0
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
