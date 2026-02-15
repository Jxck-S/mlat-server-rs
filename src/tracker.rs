// Aircraft tracking
// Ported from mlat/tracker.py

use std::collections::HashSet;
use crate::kalman::KalmanState;

/// Constants for MLAT tracking
const FORCE_MLAT_INTERVAL: f64 = 600.0;  // Force MLAT every 10 minutes
const NO_ADSB_MLAT_SECONDS: f64 = 120.0;  // Enable MLAT if no ADS-B for 2 minutes

/// A single tracked aircraft
#[derive(Debug)]
pub struct TrackedAircraft {
    // Identity
    /// ICAO address of this aircraft
    pub icao: u32,
    /// Allow multilateration of this aircraft?
    pub allow_mlat: bool,
    
    // Tracking sets (receiver IDs)
    /// Set of receivers that can see this aircraft
    pub tracking: HashSet<usize>,
    /// Set of receivers using this aircraft for synchronization
    pub sync_interest: HashSet<usize>,
    /// Set of receivers wanting to use this aircraft for multilateration
    pub mlat_interest: HashSet<usize>,
    /// Set of receivers who have seen ADS-B from this aircraft
    pub adsb_seen: HashSet<usize>,
    
    // Timestamps
    /// Last time this aircraft was seen (Unix timestamp)
    pub seen: f64,
    /// Last time we received valid ADS-B position from this aircraft
    pub last_adsb_time: f64,
    /// Last time we forced MLAT for this aircraft
    pub last_force_mlat: f64,
    /// Should we force MLAT for this aircraft?
    pub force_mlat: bool,
    
    // MLAT statistics
    /// Number of MLAT message resolves attempted
    pub mlat_message_count: usize,
    /// Number of MLAT messages that produced valid least-squares results
    pub mlat_result_count: usize,
    /// Number of MLAT messages that produced valid Kalman state updates
    pub mlat_kalman_count: usize,
    
    // Position and altitude
    /// Last reported altitude (feet)
    pub altitude: Option<f64>,
    /// Time of last altitude (Unix timestamp)
    pub last_altitude_time: Option<f64>,
    /// Altitude history: (time, altitude) tuples
    pub alt_history: Vec<(f64, f64)>,
    /// Derived vertical rate (feet per minute)
    pub vrate: Option<i32>,
    /// Time of last vertical rate calculation
    pub vrate_time: Option<f64>,
    
    // Last MLAT result
    /// Last multilateration time (monotonic)
    pub last_result_time: Option<f64>,
    /// Last multilateration ECEF position
    pub last_result_position: Option<[f64; 3]>,
    /// Last multilateration variance
    pub last_result_var: Option<f64>,
    /// Last multilateration degrees of freedom
    pub last_result_dof: Option<usize>,
    /// Last multilateration distinct receivers
    pub last_result_distinct: Option<usize>,
    
    // Kalman filter state
    /// Kalman filter for position tracking
    pub kalman: KalmanState,
    
    // Sync quality metrics
    /// Number of good sync messages
    pub sync_good: usize,
    /// Number of bad sync messages
    pub sync_bad: usize,
    /// Don't use this aircraft for sync?
    pub sync_dont_use: bool,
    /// Percentage of bad sync messages
    pub sync_bad_percent: f64,
    
    // Metadata
    /// Aircraft callsign
    pub callsign: Option<String>,
    /// Squawk code
    pub squawk: Option<u16>,
    /// Should we do MLAT for this aircraft?
    pub do_mlat: bool,
    
    // Internal state
    pub last_resolve_attempt: f64,
    #[allow(dead_code)]
    last_crappy_output: f64,
}

impl TrackedAircraft {
    /// Create a new tracked aircraft
    pub fn new(icao: u32, allow_mlat: bool, now: f64) -> Self {
        // Random jitter for force_mlat to spread load
        let jitter = (icao as f64 * 0.123456789) % 1.0;  // Deterministic "random"
        
        TrackedAircraft {
            icao,
            allow_mlat,
            
            tracking: HashSet::new(),
            sync_interest: HashSet::new(),
            mlat_interest: HashSet::new(),
            adsb_seen: HashSet::new(),
            
            seen: now,
            last_adsb_time: 0.0,
            last_force_mlat: now - FORCE_MLAT_INTERVAL * jitter,
            force_mlat: false,
            
            mlat_message_count: 0,
            mlat_result_count: 0,
            mlat_kalman_count: 0,
            
            altitude: None,
            last_altitude_time: None,
            alt_history: Vec::new(),
            vrate: None,
            vrate_time: None,
            
            last_result_time: None,
            last_result_position: None,
            last_result_var: None,
            last_result_dof: None,
            last_result_distinct: None,
            
            kalman: KalmanState::new(icao),
            
            sync_good: 0,
            sync_bad: 0,
            sync_dont_use: false,
            sync_bad_percent: 0.0,
            
            callsign: None,
            squawk: None,
            do_mlat: false,
            
            last_resolve_attempt: 0.0,
            last_crappy_output: 0.0,
        }
    }
    
    /// Is this aircraft interesting (i.e., are we asking any station to transmit data for it)?
    pub fn interesting(&self) -> bool {
        !self.sync_interest.is_empty() || (self.allow_mlat && !self.mlat_interest.is_empty())
    }
    
    /// Update altitude and calculate vertical rate
    pub fn update_altitude(&mut self, new_altitude: f64, time: f64) {
        // Validate altitude range
        if new_altitude < -1500.0 || new_altitude > 75000.0 {
            return;
        }
        
        // Check if we should update
        let should_update = match self.last_altitude_time {
            None => true,
            Some(last_time) => {
                time > last_time && 
                (time - last_time > 15.0 || 
                 self.altitude.map_or(true, |alt| (alt - new_altitude).abs() < 4000.0))
            }
        };
        
        if !should_update {
            return;
        }
        
        self.altitude = Some(new_altitude);
        self.last_altitude_time = Some(time);
        
        // Prune old altitude history (keep last 20 seconds)
        self.alt_history.retain(|(ts, _)| time - ts < 20.0);
        self.alt_history.push((time, new_altitude));
        
        // Calculate vertical rate if we have enough history
        if let Some((first_time, first_alt)) = self.alt_history.first() {
            let ts_diff = time - first_time;
            if ts_diff > 10.0 {
                // Calculate vertical rate in feet per minute
                let new_vrate = ((new_altitude - first_alt) / (ts_diff / 60.0)) as i32;
                
                // Apply exponential smoothing if we have a recent vrate
                self.vrate = if let (Some(old_vrate), Some(vrate_time)) = (self.vrate, self.vrate_time) {
                    if time - vrate_time < 15.0 {
                        Some((old_vrate as f64 + 0.3 * (new_vrate as f64 - old_vrate as f64)) as i32)
                    } else {
                        Some(new_vrate)
                    }
                } else {
                    Some(new_vrate)
                };
                
                self.vrate_time = Some(time);
            }
        }
    }
    
    /// Update sync quality metrics
    pub fn update_sync_quality(&mut self) {
        let total = self.sync_good + self.sync_bad;
        if total > 0 {
            self.sync_bad_percent = (self.sync_bad as f64 / total as f64) * 100.0;
        }
        
        // Mark as don't use if bad percentage is too high
        self.sync_dont_use = self.sync_bad_percent > 10.0;
    }
}

impl PartialOrd for TrackedAircraft {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for TrackedAircraft {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.icao.cmp(&other.icao)
    }
}

impl PartialEq for TrackedAircraft {
    fn eq(&self, other: &Self) -> bool {
        self.icao == other.icao
    }
}

impl Eq for TrackedAircraft {}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_tracked_aircraft_creation() {
        let ac = TrackedAircraft::new(0xABCDEF, true, 1000.0);
        assert_eq!(ac.icao, 0xABCDEF);
        assert!(ac.allow_mlat);
        assert_eq!(ac.tracking.len(), 0);
        assert!(!ac.interesting());
        assert_eq!(ac.mlat_message_count, 0);
    }
    
    #[test]
    fn test_interesting() {
        let mut ac = TrackedAircraft::new(0x123456, true, 1000.0);
        assert!(!ac.interesting());
        
        // Add sync interest
        ac.sync_interest.insert(1);
        assert!(ac.interesting());
        
        // Clear sync, add MLAT interest
        ac.sync_interest.clear();
        ac.mlat_interest.insert(2);
        assert!(ac.interesting());
        
        // Disable MLAT
        ac.allow_mlat = false;
        assert!(!ac.interesting());
    }
    
    #[test]
    fn test_altitude_update() {
        let mut ac = TrackedAircraft::new(0x123456, true, 1000.0);
        
        // First altitude
        ac.update_altitude(10000.0, 1000.0);
        assert_eq!(ac.altitude, Some(10000.0));
        assert_eq!(ac.alt_history.len(), 1);
        assert!(ac.vrate.is_none());  // Not enough history
        
        // Second altitude after 11 seconds (enough for vrate)
        ac.update_altitude(11000.0, 1011.0);
        assert_eq!(ac.altitude, Some(11000.0));
        assert_eq!(ac.alt_history.len(), 2);
        assert!(ac.vrate.is_some());
        
        // Vertical rate should be ~5454 fpm (1000 ft / 11 sec * 60)
        let vrate = ac.vrate.unwrap();
        assert!((vrate - 5454).abs() < 100);
    }
    
    #[test]
    fn test_altitude_validation() {
        let mut ac = TrackedAircraft::new(0x123456, true, 1000.0);
        
        // Invalid altitudes should be rejected
        ac.update_altitude(-2000.0, 1000.0);
        assert!(ac.altitude.is_none());
        
        ac.update_altitude(80000.0, 1001.0);
        assert!(ac.altitude.is_none());
        
        // Valid altitude
        ac.update_altitude(35000.0, 1002.0);
        assert_eq!(ac.altitude, Some(35000.0));
    }
    
    #[test]
    fn test_sync_quality() {
        let mut ac = TrackedAircraft::new(0x123456, true, 1000.0);
        
        ac.sync_good = 90;
        ac.sync_bad = 10;
        ac.update_sync_quality();
        
        assert_eq!(ac.sync_bad_percent, 10.0);
        assert!(!ac.sync_dont_use);
        
        // Increase bad percentage
        ac.sync_bad = 20;
        ac.update_sync_quality();
        
        assert!((ac.sync_bad_percent - 18.18).abs() < 0.1);
        assert!(ac.sync_dont_use);
    }
    
    #[test]
    fn test_ordering() {
        let ac1 = TrackedAircraft::new(0x111111, true, 1000.0);
        let ac2 = TrackedAircraft::new(0x222222, true, 1000.0);
        
        assert!(ac1 < ac2);
        assert_eq!(ac1, ac1);
    }
}

use std::collections::HashMap;

/// Aircraft tracker - manages all tracked aircraft
pub struct Tracker {
    /// Map of ICAO address -> TrackedAircraft
    aircraft: HashMap<u32, TrackedAircraft>,
    
    /// Partition ID (for distributed systems)
    partition_id: usize,
    /// Total number of partitions
    partition_count: usize,
    
    /// Set of aircraft we want to do MLAT on (cached)
    mlat_wanted: HashSet<u32>,
    /// Timestamp when mlat_wanted was last updated
    mlat_wanted_ts: f64,
}

impl Tracker {
    /// Create a new tracker
    pub fn new(partition_id: usize, partition_count: usize) -> Self {
        Tracker {
            aircraft: HashMap::new(),
            partition_id,
            partition_count,
            mlat_wanted: HashSet::new(),
            mlat_wanted_ts: 0.0,
        }
    }
    
    /// Number of tracked aircraft
    pub fn aircraft_count(&self) -> usize {
        self.aircraft.len()
    }

    /// Check if an ICAO address belongs to this partition
    /// 
    /// Uses a hash function to distribute aircraft across partitions
    pub fn in_local_partition(&self, icao: u32) -> bool {
        if self.partition_count == 1 {
            return true;
        }
        
        // Mix the address using a hash function (from Python)
        let mut h = icao;
        h = (((h >> 16) ^ h).wrapping_mul(0x45d9f3b)) & 0xFFFFFFFF;
        h = (((h >> 16) ^ h).wrapping_mul(0x45d9f3b)) & 0xFFFFFFFF;
        h = (h >> 16) ^ h;
        
        (h as usize % self.partition_count) == self.partition_id
    }
    
    /// Add aircraft to tracking for a receiver
    pub fn add(&mut self, receiver_id: usize, icao_set: &[u32], now: f64) {
        for &icao in icao_set {
            // Pre-calculate partition membership to avoid borrow checker issues
            let allow_mlat = self.in_local_partition(icao);
            
            let ac = self.aircraft
                .entry(icao)
                .or_insert_with(|| TrackedAircraft::new(icao, allow_mlat, now));
            
            ac.tracking.insert(receiver_id);
            ac.seen = now;
        }
    }
    
    /// Remove aircraft from tracking for a receiver
    pub fn remove(&mut self, receiver_id: usize, icao_set: &[u32]) {
        for &icao in icao_set {
            if let Some(ac) = self.aircraft.get_mut(&icao) {
                ac.tracking.remove(&receiver_id);
                
                // Remove aircraft if no receivers are tracking it
                if ac.tracking.is_empty() {
                    self.aircraft.remove(&icao);
                }
            }
        }
    }
    
    /// Remove all aircraft for a receiver
    pub fn remove_all(&mut self, receiver_id: usize) {
        // Collect ICAOs to remove
        let to_remove: Vec<u32> = self.aircraft
            .iter_mut()
            .filter_map(|(&icao, ac)| {
                ac.tracking.remove(&receiver_id);
                ac.sync_interest.remove(&receiver_id);
                ac.adsb_seen.remove(&receiver_id);
                ac.mlat_interest.remove(&receiver_id);
                
                if ac.tracking.is_empty() {
                    Some(icao)
                } else {
                    None
                }
            })
            .collect();
        
        // Remove aircraft with no tracking receivers
        for icao in to_remove {
            self.aircraft.remove(&icao);
        }
    }
    
    /// Update MLAT wanted set
    /// 
    /// This determines which aircraft we want to do MLAT on
    pub fn update_mlat_wanted(&mut self, now: f64) {
        // Only update every 0.1 seconds
        if now - self.mlat_wanted_ts < 0.1 {
            return;
        }
        
        self.mlat_wanted.clear();
        
        for ac in self.aircraft.values_mut() {
            let since_force = now - ac.last_force_mlat;
            
            // Update force_mlat flag
            if !ac.force_mlat && since_force > FORCE_MLAT_INTERVAL - 15.0 {
                ac.force_mlat = true;
            }
            
            if since_force > FORCE_MLAT_INTERVAL + 15.0 {
                // Random jitter for next force interval
                let jitter = (ac.icao as f64 * 0.123456789) % 1.0;
                ac.last_force_mlat = now + jitter;
                ac.force_mlat = false;
            }
            
            // Determine if we want to do MLAT for this aircraft
            let want_mlat = ac.tracking.len() >= 2
                && ac.allow_mlat
                && (now - ac.last_adsb_time > NO_ADSB_MLAT_SECONDS
                    || ac.sync_bad_percent > 10.0
                    || (since_force > FORCE_MLAT_INTERVAL - 15.0 
                        && since_force < FORCE_MLAT_INTERVAL));
            
            if want_mlat {
                self.mlat_wanted.insert(ac.icao);
                ac.do_mlat = true;
            } else {
                ac.do_mlat = false;
            }
        }
        
        self.mlat_wanted_ts = now;
    }
    
    /// Get an aircraft by ICAO address
    pub fn get(&self, icao: u32) -> Option<&TrackedAircraft> {
        self.aircraft.get(&icao)
    }
    
    /// Get a mutable reference to an aircraft by ICAO address
    pub fn get_mut(&mut self, icao: u32) -> Option<&mut TrackedAircraft> {
        self.aircraft.get_mut(&icao)
    }
    
    /// Get the number of tracked aircraft
    pub fn num_aircraft(&self) -> usize {
        self.aircraft.len()
    }

    /// Mutable iterator over (icao, aircraft) for state dump (sync_good/sync_bad decay, etc.)
    pub fn aircraft_iter_mut(&mut self) -> std::collections::hash_map::IterMut<'_, u32, TrackedAircraft> {
        self.aircraft.iter_mut()
    }
    
    /// Get the number of aircraft we want to do MLAT on
    pub fn num_mlat_wanted(&self) -> usize {
        self.mlat_wanted.len()
    }

    /// Count of aircraft with at least one receiver in sync_interest (for status line)
    pub fn num_sync_interest(&self) -> usize {
        self.aircraft.values().filter(|ac| !ac.sync_interest.is_empty()).count()
    }

    /// Update aircraft state from a decoded message.
    /// Only updates aircraft that are already in the tracker (added via "seen" / receiver_tracking_add).
    /// Matches Python: tracker.add is only called from receiver_tracking_add(seen); sync/mlat do not create new aircraft.
    pub fn update_aircraft(
        &mut self,
        icao: u32,
        timestamp: f64,
        source: &crate::modes::message::DecodedMessage,
        receiver_id: usize,
    ) {
        let now = timestamp; // Simplify for now
        
        if !self.in_local_partition(icao) {
            return;
        }
        // Only update existing aircraft; do not create from sync/mlat (match Python behaviour)
        let ac = match self.aircraft.get_mut(&icao) {
            Some(a) => a,
            None => return,
        };
        ac.seen = now;
        ac.tracking.insert(receiver_id);
        
        // Extract message details
        let traj = source.as_trait();
        let _df = traj.df();
        
        // Update altitude if present
        if let Some(alt) = traj.altitude() {
            ac.update_altitude(alt as f64, now);
        }
        
        // Update callsign if present
        if let Some(callsign) = traj.callsign() {
            ac.callsign = Some(callsign.to_string());
        }
        
        // Update squawk if present
        if let Some(squawk) = traj.squawk() {
            if let Ok(sq) = u16::from_str_radix(squawk, 8) {
                ac.squawk = Some(sq);
            }
        }
        
        // Mark as having ADS-B data if message has position
        if let crate::modes::message::DecodedMessage::DF17(df17) = source {
            if matches!(df17.estype, crate::modes::message::ESType::AirbornePosition | crate::modes::message::ESType::SurfacePosition) {
                ac.last_adsb_time = now;
                ac.adsb_seen.insert(receiver_id);
            }
        }
    }

    /// Update interest sets for a receiver
    /// Mirrors Python tracker.update_interest logic.
    pub fn update_interest(
        &mut self,
        receiver_id: usize,
        receivers: &HashMap<usize, crate::receiver::Receiver>,
        now: f64,
        max_sync_ac: usize,
        max_sync_rate: f64,
    ) -> (HashSet<u32>, HashSet<u32>, HashSet<u32>) {
        // 1. Update mlat_wanted every 0.1s
        self.update_mlat_wanted(now);

        let receiver = match receivers.get(&receiver_id) {
            Some(r) => r,
            None => return (HashSet::new(), HashSet::new(), HashSet::new()),
        };

        // 2. Determine mlat_interest (intersection of tracking and mlat_wanted)
        let mut new_mlat = HashSet::new();
        for &icao in &receiver.tracking {
            if self.mlat_wanted.contains(&icao) {
                new_mlat.insert(icao);
            }
        }

        let mut new_adsb = HashSet::new();
        let mut new_sync = HashSet::new();

        // 3. Handle legacy clients or missing rate reports
        if receiver.last_rate_report.is_none() {
            let mut candidates: Vec<u32> = receiver.tracking.iter().copied().collect();
            candidates.sort(); 
            for &icao in candidates.iter().take(max_sync_ac) {
                new_sync.insert(icao);
            }
        } else {
            let rates = receiver.last_rate_report.as_ref().unwrap();
            let mut ratepair_list = Vec::new();
            let mut ac_to_ratepairs = HashMap::new();

            // 4. Build ratepair list (rp, other_receiver_id, icao, internal_rate)
            for (&icao, &rate) in rates {
                let ac = match self.aircraft.get_mut(&icao) {
                    Some(a) => a,
                    None => continue,
                };
                // Python: ac.seen = now (tracker.py L236)
                ac.seen = now;
                new_adsb.insert(icao);

                let alt_factor = if let Some(alt) = ac.altitude {
                    if alt > 0.0 {
                        1.0 + (alt / 20000.0).powf(1.5)
                    } else {
                        1.0
                    }
                } else {
                    1.0
                };

                let mut ac_pairs = Vec::new();
                for &r1_id in &ac.tracking {
                    if r1_id == receiver_id { continue; }

                    let r1 = match receivers.get(&r1_id) {
                        Some(r) => r,
                        None => continue,
                    };

                    let rate1 = if let Some(r1_rates) = &r1.last_rate_report {
                        r1_rates.get(&icao).copied().unwrap_or(0.0)
                    } else {
                        0.8
                    };

                    let mut rp = rate * rate1 / 2.25;
                    rp *= alt_factor;

                    if rp < 0.01 { continue; }

                    let pair = (rp, r1_id, icao, rate);
                    ac_pairs.push(pair);
                    ratepair_list.push(pair);
                }
                ac_to_ratepairs.insert(icao, ac_pairs);
            }

            // 5. Selection Logic (Round 1 & 2)
            ratepair_list.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
            
            let split = ratepair_list.len() / 2;
            // Python: random.shuffle(firstHalf) - shuffle top half before round 1
            let mut first_half: Vec<_> = ratepair_list[..split].to_vec();
            {
                use rand::seq::SliceRandom;
                first_half.shuffle(&mut rand::thread_rng());
            }
            let first_half = &first_half;

            let mut ntotal: HashMap<usize, f64> = HashMap::new();
            let mut total_rate = 0.0;

            // Round 1
            for &(_rp, r1_id, icao, rate) in first_half {
                if new_sync.contains(&icao) { continue; }
                let ac = self.aircraft.get(&icao).unwrap();
                if ac.sync_dont_use { continue; }

                if total_rate > max_sync_rate { break; }

                if *ntotal.get(&r1_id).unwrap_or(&0.0) < 0.3 {
                    new_sync.insert(icao);
                    total_rate += rate;
                    if let Some(pairs) = ac_to_ratepairs.get(&icao) {
                        for &(rp2, r2_id, _, _) in pairs {
                            *ntotal.entry(r2_id).or_insert(0.0) += rp2;
                        }
                    }
                }
            }

            // Round 2 (Python: ntotal.get(r1, 0.0) < 3.5; NO sync_dont_use check in round 2)
            for &(_rp, r1_id, icao, rate) in &ratepair_list {
                if new_sync.contains(&icao) { continue; }

                if total_rate > max_sync_rate { break; }

                if *ntotal.get(&r1_id).unwrap_or(&0.0) < 3.5 {
                    new_sync.insert(icao);
                    total_rate += rate;
                    if let Some(pairs) = ac_to_ratepairs.get(&icao) {
                        for &(rp2, r2_id, _, _) in pairs {
                            *ntotal.entry(r2_id).or_insert(0.0) += rp2;
                        }
                    }
                }
            }

            // addSome: top up to at least MAX_SYNC_AC/4 (Python tracker.py L318-326)
            let target_min = max_sync_ac / 4;
            let mut add_some = target_min.saturating_sub(new_sync.len());
            if add_some > 0 {
                let mut available: Vec<u32> = ac_to_ratepairs.keys()
                    .filter(|&&icao| !new_sync.contains(&icao))
                    .copied()
                    .collect();
                if !available.is_empty() {
                    use rand::seq::SliceRandom;
                    let n = add_some.min(available.len());
                    available.shuffle(&mut rand::thread_rng());
                    for &icao in available.iter().take(n) {
                        new_sync.insert(icao);
                    }
                }
            }
            add_some = target_min.saturating_sub(new_sync.len());
            if add_some > 0 {
                let mut available: Vec<u32> = receiver.tracking.iter()
                    .filter(|&&icao| !new_sync.contains(&icao))
                    .copied()
                    .collect();
                if !available.is_empty() {
                    use rand::seq::SliceRandom;
                    let n = add_some.min(available.len());
                    available.shuffle(&mut rand::thread_rng());
                    for &icao in available.iter().take(n) {
                        new_sync.insert(icao);
                    }
                }
            }
        }

        // 6. Update aircraft interest sets (needed for Sync-MLAT and tracking)
        // First, clear this receiver from all aircraft it was interested in
        for ac in self.aircraft.values_mut() {
            ac.sync_interest.remove(&receiver_id);
            ac.mlat_interest.remove(&receiver_id);
        }

        // Apply new interests
        for &icao in &new_sync {
            if let Some(ac) = self.aircraft.get_mut(&icao) {
                ac.sync_interest.insert(receiver_id);
            }
        }
        for &icao in &new_mlat {
            if let Some(ac) = self.aircraft.get_mut(&icao) {
                ac.mlat_interest.insert(receiver_id);
            }
        }

        (new_sync, new_mlat, new_adsb)
    }
}

#[cfg(test)]
mod tracker_tests {
    use super::*;
    
    #[test]
    fn test_tracker_creation() {
        let tracker = Tracker::new(0, 1);
        assert_eq!(tracker.num_aircraft(), 0);
        assert_eq!(tracker.partition_id, 0);
        assert_eq!(tracker.partition_count, 1);
    }
    
    #[test]
    fn test_partition_hashing() {
        let tracker = Tracker::new(0, 4);
        
        // Test that partition hashing is deterministic
        let icao = 0xABCDEF;
        let partition1 = tracker.in_local_partition(icao);
        let partition2 = tracker.in_local_partition(icao);
        assert_eq!(partition1, partition2);
        
        // Test that single partition accepts all
        let tracker_single = Tracker::new(0, 1);
        assert!(tracker_single.in_local_partition(icao));
    }
    
    #[test]
    fn test_add_remove() {
        let mut tracker = Tracker::new(0, 1);
        let now = 1000.0;
        
        // Add aircraft
        tracker.add(1, &[0x123456, 0x789ABC], now);
        assert_eq!(tracker.num_aircraft(), 2);
        
        let ac = tracker.get(0x123456).unwrap();
        assert!(ac.tracking.contains(&1));
        
        // Remove one aircraft
        tracker.remove(1, &[0x123456]);
        assert_eq!(tracker.num_aircraft(), 1);
        assert!(tracker.get(0x123456).is_none());
        
        // Remove all
        tracker.remove_all(1);
        assert_eq!(tracker.num_aircraft(), 0);
    }
    
    #[test]
    fn test_mlat_wanted() {
        let mut tracker = Tracker::new(0, 1);
        let now = 1000.0;
        
        // Add aircraft with 2 receivers
        tracker.add(1, &[0x123456], now);
        tracker.add(2, &[0x123456], now);
        
        // Set recent ADS-B time
        {
            let ac = tracker.get_mut(0x123456).unwrap();
            ac.last_adsb_time = now - 10.0;  // Recent ADS-B
        }
        
        // Update MLAT wanted
        tracker.update_mlat_wanted(now);
        
        // Should not want MLAT yet (recent ADS-B)
        assert_eq!(tracker.num_mlat_wanted(), 0);
        
        // Simulate no ADS-B for 2 minutes
        {
            let ac = tracker.get_mut(0x123456).unwrap();
            ac.last_adsb_time = now - 130.0;
        }
        
        // Update MLAT wanted again after changing state
        tracker.update_mlat_wanted(now + 0.2);  // Force update by advancing time
        assert_eq!(tracker.num_mlat_wanted(), 1);
    }
    
    #[test]
    fn test_partition_distribution() {
        let tracker1 = Tracker::new(0, 4);
        let tracker2 = Tracker::new(1, 4);
        let tracker3 = Tracker::new(2, 4);
        let tracker4 = Tracker::new(3, 4);
        
        // Test that each ICAO goes to exactly one partition
        for icao in 0x100000..0x100100 {
            let count = [
                tracker1.in_local_partition(icao),
                tracker2.in_local_partition(icao),
                tracker3.in_local_partition(icao),
                tracker4.in_local_partition(icao),
            ].iter().filter(|&&x| x).count();
            
            assert_eq!(count, 1, "ICAO {:06X} should belong to exactly one partition", icao);
        }
    }
}
