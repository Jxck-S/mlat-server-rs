// Receiver infrastructure for MLAT server
// Ported from mlat/coordinator.py (Receiver class)

use std::collections::{HashMap, HashSet};
use crate::geodesy;
use crate::clocktrack::Clock;

/// Represents a connected MLAT receiver
#[derive(Debug, Clone)]
pub struct Receiver {
    // Identity
    pub uid: usize,
    pub uuid: Option<String>,
    pub user: String,
    
    // Position
    pub position_llh: [f64; 3],  // [lat, lon, alt] in degrees, degrees, meters
    pub position: [f64; 3],       // ECEF [x, y, z] in meters
    
    // Clock
    pub clock: Clock,
    pub epoch: Option<String>,
    pub last_clock_reset: f64,
    pub clock_reset_counter: usize,
    
    // Sync quality metrics
    pub num_outliers: usize,
    pub num_syncs: usize,
    pub outlier_percent_rolling: f64,
    pub sync_peers: [usize; 5],  // Peers per distance category
    pub last_sync: f64,
    pub bad_syncs: f64,          // 0-6 score (>0 = timed out)
    pub sync_range_exceeded: usize,          // timestamp of last range exceeded
    pub recent_pair_jumps: usize,
    
    // Interest sets (ICAO addresses)
    pub tracking: HashSet<u32>,
    pub adsb_seen: HashSet<u32>,
    pub sync_interest: HashSet<u32>,
    pub mlat_interest: HashSet<u32>,
    pub requested: HashSet<u32>,
    
    // Distance matrix (receiver uid -> distance in meters)
    pub distance: HashMap<usize, f64>,
    
    // Metadata
    pub privacy: bool,
    pub connection_info: String,
    /// Source IP for state file (clients.json); set after handshake from connection peer_addr.
    pub source_ip: Option<String>,
    /// Source port for state file (clients.json).
    pub source_port: Option<u16>,
    pub dead: bool,
    pub connected_since: f64,
    pub focus: bool,  // Debug flag
    
    // Map position (privacy-fudged for display)
    pub map_lat: Option<f64>,
    pub map_lon: Option<f64>,
    pub map_alt: Option<f64>,
    
    // Rate reporting
    pub last_rate_report: Option<HashMap<u32, f64>>,
}

impl Receiver {
    /// Create a new receiver
    pub fn new(
        uid: usize,
        user: String,
        uuid: Option<String>,
        clock_type: &str,
        position_llh: [f64; 3],
        privacy: bool,
        connection_info: String,
        now: f64,
    ) -> Self {
        let position_tuple = geodesy::llh2ecef(position_llh[0], position_llh[1], position_llh[2]);
        let position = [position_tuple.0, position_tuple.1, position_tuple.2];
        
        // Parse clock type string
        let clock_type_enum = match clock_type {
            "radarcape_gps" => crate::clocktrack::ClockType::RadarcapeGps,
            "radarcape" => crate::clocktrack::ClockType::RadarcapeOld,
            "beast" => crate::clocktrack::ClockType::Beast,
            "sbs" => crate::clocktrack::ClockType::Sbs,
            "dump1090" => crate::clocktrack::ClockType::Dump1090,
            _ => crate::clocktrack::ClockType::Unknown,
        };
        let clock = Clock::from_type(clock_type_enum);
        let epoch = if clock_type == "radarcape_gps" {
            Some("gps_midnight".to_string())
        } else {
            None
        };
        
        Receiver {
            uid,
            uuid,
            user,
            position_llh,
            position,
            clock,
            epoch,
            last_clock_reset: now,
            clock_reset_counter: 0,
            num_outliers: 0,
            num_syncs: 0,
            outlier_percent_rolling: 0.0,
            sync_peers: [0; 5],
            last_sync: 0.0,
            bad_syncs: 0.0,
            sync_range_exceeded: 0,
            recent_pair_jumps: 0,
            tracking: HashSet::new(),
            adsb_seen: HashSet::new(),
            sync_interest: HashSet::new(),
            mlat_interest: HashSet::new(),
            requested: HashSet::new(),
            distance: HashMap::new(),
            privacy,
            connection_info,
            source_ip: None,
            source_port: None,
            dead: false,
            connected_since: now,
            focus: false,
            map_lat: None,
            map_lon: None,
            map_alt: None,
            last_rate_report: None,
        }
    }
    
    /// Increment clock jump counter and potentially trigger reset
    pub fn increment_jumps(&mut self) {
        self.recent_pair_jumps += 1;
        let total_peers: usize = self.sync_peers.iter().sum();
        
        if total_peers > 0 {
            let jump_ratio = self.recent_pair_jumps as f64 / total_peers as f64;
            if jump_ratio > 0.25 && self.recent_pair_jumps > 3 {
                self.recent_pair_jumps = 0;
                // Caller should call clock_tracker.receiver_clock_reset()
                self.bad_syncs += 0.1;
            }
        }
    }
    
    /// Reset clock tracking for this receiver
    pub fn reset_clock(&mut self, now: f64) {
        self.last_clock_reset = now;
        self.clock_reset_counter += 1;
    }
    
    /// Update interest sets and return (start, stop) lists for traffic request
    pub fn update_interest_sets(
        &mut self,
        new_sync: HashSet<u32>,
        new_mlat: HashSet<u32>,
        new_adsb: HashSet<u32>,
    ) -> (Vec<u32>, Vec<u32>) {
        self.sync_interest = new_sync.clone();
        self.mlat_interest = new_mlat.clone();
        self.adsb_seen = new_adsb.clone();

        let new_requested = &(&new_sync | &new_mlat) | &new_adsb;
        
        let start: Vec<u32> = new_requested.difference(&self.requested).copied().collect();
        let stop: Vec<u32> = self.requested.difference(&new_requested).copied().collect();
        
        self.requested = new_requested;
        
        (start, stop)
    }
}

impl PartialOrd for Receiver {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Receiver {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.uid.cmp(&other.uid)
    }
}

impl PartialEq for Receiver {
    fn eq(&self, other: &Self) -> bool {
        self.uid == other.uid
    }
}

impl Eq for Receiver {}

/// Calculate ECEF distance between two positions
pub fn ecef_distance(pos1: [f64; 3], pos2: [f64; 3]) -> f64 {
    let dx = pos1[0] - pos2[0];
    let dy = pos1[1] - pos2[1];
    let dz = pos1[2] - pos2[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute distance matrix for all receivers
/// 
/// Updates each receiver's distance HashMap with distances to all other receivers.
/// Distance to self is 0.
pub fn compute_distance_matrix(receivers: &mut HashMap<usize, Receiver>) {
    let uids: Vec<usize> = receivers.keys().copied().collect();
    
    for &uid1 in &uids {
        for &uid2 in &uids {
            let distance = if uid1 == uid2 {
                0.0
            } else {
                let pos1 = receivers[&uid1].position;
                let pos2 = receivers[&uid2].position;
                ecef_distance(pos1, pos2)
            };
            
            receivers.get_mut(&uid1).unwrap().distance.insert(uid2, distance);
        }
    }
}

/// Remove a receiver from all other receivers' distance maps (on disconnect).
/// Mirrors Python: for other_receiver in self.receivers.values(): other_receiver.distance.pop(receiver.uid, None)
pub fn remove_distance_entries(receivers: &mut HashMap<usize, Receiver>, removed_uid: usize) {
    for r in receivers.values_mut() {
        r.distance.remove(&removed_uid);
    }
}

/// Update distance matrix when a receiver is added
///
/// Computes distances from the new receiver to all existing receivers
/// and updates both the new receiver's and existing receivers' distance maps.
pub fn update_distance_matrix(receivers: &mut HashMap<usize, Receiver>, new_uid: usize) {
    let new_pos = receivers[&new_uid].position;
    let uids: Vec<usize> = receivers.keys().copied().collect();
    
    for &other_uid in &uids {
        let distance = if new_uid == other_uid {
            0.0
        } else {
            let other_pos = receivers[&other_uid].position;
            ecef_distance(new_pos, other_pos)
        };
        
        // Update new receiver's distance map
        receivers.get_mut(&new_uid).unwrap().distance.insert(other_uid, distance);
        
        // Update other receiver's distance map
        if new_uid != other_uid {
            receivers.get_mut(&other_uid).unwrap().distance.insert(new_uid, distance);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_receiver_creation() {
        let now = 1000.0;
        let receiver = Receiver::new(
            1,
            "test_user".to_string(),
            Some("uuid-123".to_string()),
            "dump1090",
            [37.5, -122.0, 100.0],  // San Francisco area
            false,
            "test connection".to_string(),
            now,
        );
        
        assert_eq!(receiver.uid, 1);
        assert_eq!(receiver.user, "test_user");
        assert_eq!(receiver.uuid, Some("uuid-123".to_string()));
        assert_eq!(receiver.position_llh, [37.5, -122.0, 100.0]);
        assert_eq!(receiver.connected_since, now);
        assert_eq!(receiver.clock_reset_counter, 0);
        assert!(!receiver.dead);
        assert!(!receiver.privacy);
        
        // Check ECEF position was calculated
        assert!(receiver.position[0].abs() > 0.0);
        assert!(receiver.position[1].abs() > 0.0);
        assert!(receiver.position[2].abs() > 0.0);
    }
    
    #[test]
    fn test_receiver_radarcape_gps() {
        let receiver = Receiver::new(
            1,
            "test".to_string(),
            None,
            "radarcape_gps",
            [0.0, 0.0, 0.0],
            false,
            "test".to_string(),
            0.0,
        );
        
        assert_eq!(receiver.epoch, Some("gps_midnight".to_string()));
    }
    
    #[test]
    fn test_ecef_distance() {
        // Two points 1000m apart along x-axis
        let pos1 = [1000.0, 0.0, 0.0];
        let pos2 = [2000.0, 0.0, 0.0];
        
        let dist = ecef_distance(pos1, pos2);
        assert!((dist - 1000.0).abs() < 0.001);
        
        // Same point
        let dist_zero = ecef_distance(pos1, pos1);
        assert!(dist_zero.abs() < 0.001);
        
        // 3D distance (3-4-5 triangle scaled by 1000)
        let pos3 = [0.0, 0.0, 0.0];
        let pos4 = [3000.0, 4000.0, 0.0];
        let dist_3d = ecef_distance(pos3, pos4);
        assert!((dist_3d - 5000.0).abs() < 0.001);
    }
    
    #[test]
    fn test_compute_distance_matrix() {
        let mut receivers = HashMap::new();
        
        // Create 3 receivers at different positions
        let r1 = Receiver::new(
            1, "r1".to_string(), None, "dump1090",
            [37.0, -122.0, 0.0], false, "r1".to_string(), 0.0
        );
        let r2 = Receiver::new(
            2, "r2".to_string(), None, "dump1090",
            [38.0, -122.0, 0.0], false, "r2".to_string(), 0.0
        );
        let r3 = Receiver::new(
            3, "r3".to_string(), None, "dump1090",
            [37.0, -121.0, 0.0], false, "r3".to_string(), 0.0
        );
        
        receivers.insert(1, r1);
        receivers.insert(2, r2);
        receivers.insert(3, r3);
        
        compute_distance_matrix(&mut receivers);
        
        // Check that each receiver has distances to all others
        assert_eq!(receivers[&1].distance.len(), 3);
        assert_eq!(receivers[&2].distance.len(), 3);
        assert_eq!(receivers[&3].distance.len(), 3);
        
        // Check self-distance is 0
        assert_eq!(receivers[&1].distance[&1], 0.0);
        assert_eq!(receivers[&2].distance[&2], 0.0);
        assert_eq!(receivers[&3].distance[&3], 0.0);
        
        // Check symmetry
        let dist_12 = receivers[&1].distance[&2];
        let dist_21 = receivers[&2].distance[&1];
        assert!((dist_12 - dist_21).abs() < 0.001);
        
        // Check distances are positive
        assert!(receivers[&1].distance[&2] > 0.0);
        assert!(receivers[&1].distance[&3] > 0.0);
        assert!(receivers[&2].distance[&3] > 0.0);
    }
    
    #[test]
    fn test_update_distance_matrix() {
        let mut receivers = HashMap::new();
        
        // Start with 2 receivers
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
        compute_distance_matrix(&mut receivers);
        
        // Add a third receiver
        let r3 = Receiver::new(
            3, "r3".to_string(), None, "dump1090",
            [37.0, -121.0, 0.0], false, "r3".to_string(), 0.0
        );
        receivers.insert(3, r3);
        update_distance_matrix(&mut receivers, 3);
        
        // Check that all receivers now have distances to all others
        assert_eq!(receivers[&1].distance.len(), 3);
        assert_eq!(receivers[&2].distance.len(), 3);
        assert_eq!(receivers[&3].distance.len(), 3);
        
        // Check new receiver has correct distances
        assert_eq!(receivers[&3].distance[&3], 0.0);
        assert!(receivers[&3].distance[&1] > 0.0);
        assert!(receivers[&3].distance[&2] > 0.0);
        
        // Check symmetry with new receiver
        assert!((receivers[&1].distance[&3] - receivers[&3].distance[&1]).abs() < 0.001);
        assert!((receivers[&2].distance[&3] - receivers[&3].distance[&2]).abs() < 0.001);
    }
    
    #[test]
    fn test_increment_jumps() {
        let mut receiver = Receiver::new(
            1, "test".to_string(), None, "dump1090",
            [0.0, 0.0, 0.0], false, "test".to_string(), 0.0
        );
        
        receiver.sync_peers = [2, 2, 2, 2, 2];  // 10 total peers
        receiver.recent_pair_jumps = 0;
        receiver.bad_syncs = 0.0;
        
        // Increment jumps below threshold
        receiver.increment_jumps();
        assert_eq!(receiver.recent_pair_jumps, 1);
        assert_eq!(receiver.bad_syncs, 0.0);
        
        // Increment to above threshold (>25% of 10 peers = >2.5, and >3)
        receiver.recent_pair_jumps = 3;
        receiver.increment_jumps();  // Now 4 jumps, 40% of 10 peers
        
        // Should reset jumps and increment bad_syncs
        assert_eq!(receiver.recent_pair_jumps, 0);
        assert!((receiver.bad_syncs - 0.1).abs() < 0.001);
    }
    
    #[test]
    fn test_reset_clock() {
        let mut receiver = Receiver::new(
            1, "test".to_string(), None, "dump1090",
            [0.0, 0.0, 0.0], false, "test".to_string(), 1000.0
        );
        
        assert_eq!(receiver.last_clock_reset, 1000.0);
        assert_eq!(receiver.clock_reset_counter, 0);
        
        receiver.reset_clock(2000.0);
        
        assert_eq!(receiver.last_clock_reset, 2000.0);
        assert_eq!(receiver.clock_reset_counter, 1);
    }
    
    #[test]
    fn test_receiver_ordering() {
        let r1 = Receiver::new(
            1, "r1".to_string(), None, "dump1090",
            [0.0, 0.0, 0.0], false, "r1".to_string(), 0.0
        );
        let r2 = Receiver::new(
            2, "r2".to_string(), None, "dump1090",
            [0.0, 0.0, 0.0], false, "r2".to_string(), 0.0
        );
        
        assert!(r1 < r2);
        assert!(r2 > r1);
        assert_eq!(r1, r1);
    }
}
