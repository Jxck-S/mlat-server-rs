// Clock synchronization tracking
// Ported from mlat/clocktrack.pyx

use std::collections::HashMap;

/// Maximum range for clock synchronization (500 km)
const MAX_RANGE: f64 = 500e3;

/// Clock pairing buffer size
const CP_SIZE: usize = 32;

/// Number of samples needed for stable drift estimation
const DRIFT_N_STABLE: usize = 12;

/// Clock characteristics for a receiver
#[derive(Debug, Clone)]
pub struct Clock {
    /// Clock frequency in Hz
    pub freq: f64,
    /// Maximum expected relative frequency error (e.g., 1e-6 = 1 PPM)
    pub max_freq_error: f64,
    /// Expected jitter of a typical reading, in seconds (standard deviation)
    pub jitter: f64,
    /// Precomputed delay factor: freq / C_air
    pub delay_factor: f64,
    /// Type of clock
    pub clock_type: ClockType,
    /// Epoch timestamp to subtract from incoming timestamps to maintain precision
    pub epoch: f64,
}

/// Clock types supported by receivers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockType {
    RadarcapeGps,
    RadarcapeOld,
    Beast,
    Sbs,
    Dump1090,
    Unknown,
}

impl Clock {
    /// Create a new clock with the given parameters
    pub fn new(freq: f64, max_freq_error: f64, jitter: f64) -> Self {
        const C_AIR: f64 = 299792458.0 / 1.00032; // Speed of light in air
        Clock {
            freq,
            max_freq_error,
            jitter,
            delay_factor: freq / C_AIR,
            clock_type: ClockType::Unknown,
            epoch: 0.0,
        }
    }

    /// Create a new clock with the given parameters and type
    pub fn new_with_type(freq: f64, max_freq_error: f64, jitter: f64, clock_type: ClockType) -> Self {
        const C_AIR: f64 = 299792458.0 / 1.00032; // Speed of light in air
        Clock {
            freq,
            max_freq_error,
            jitter,
            delay_factor: freq / C_AIR,
            clock_type,
            epoch: 0.0,
        }
    }
    
    /// Create a clock from a clock type
    pub fn from_type(clock_type: ClockType) -> Self {
        match clock_type {
            ClockType::RadarcapeGps => {
                // GPS-disciplined 1 GHz clock
                let mut c = Clock::new_with_type(1e9, 1e-6, 15e-9, ClockType::RadarcapeGps);
                // Use a fixed epoch to maintain precision when dealing with nanosecond timestamps
                c.epoch = 1771000000.0;
                c
            }
            ClockType::RadarcapeOld | ClockType::Beast => {
                // 12 MHz crystal
                let t = if matches!(clock_type, ClockType::RadarcapeOld) { ClockType::RadarcapeOld } else { ClockType::Beast };
                Clock::new_with_type(12e6, 5e-6, 83e-9, t)
            }
            ClockType::Sbs => {
                // 20 MHz crystal, poor quality
                Clock::new_with_type(20e6, 100e-6, 500e-9, ClockType::Sbs)
            }
            ClockType::Dump1090 | ClockType::Unknown => {
                // 12 MHz crystal, assume poor quality
                let t = if matches!(clock_type, ClockType::Dump1090) { ClockType::Dump1090 } else { ClockType::Unknown };
                Clock::new_with_type(12e6, 100e-6, 500e-9, t)
            }
        }
    }
}

/// Part of a sync point - data for one receiver
#[derive(Debug, Clone)]
pub struct SyncShard {
    /// Receiver ID (we'll use usize for now, can change to Arc<Receiver> later)
    pub receiver_id: usize,
    /// Timestamp (delay-corrected)
    pub td: f64,
    /// Interval
    pub i: f64,
}

/// A potential clock synchronization point
/// 
/// Clock synchronization points are a pair of DF17 messages,
/// and associated timing info from all receivers that see that pair.
#[derive(Debug, Clone)]
pub struct SyncPoint {
    /// ICAO address of the sync aircraft
    pub address: u32,
    /// ECEF position of the earlier message
    pub pos_a: [f64; 3],
    /// ECEF position of the later message
    pub pos_b: [f64; 3],
    /// Nominal interval (in seconds) between the two messages
    pub interval: f64,
    /// List of receivers that saw this sync point
    pub receivers: Vec<SyncShard>,
    /// Aircraft ID (we'll use usize for now)
    pub aircraft_id: usize,
    /// Last updated (for matching)
    pub last_updated: f64,
}

impl SyncPoint {
    /// Create a new sync point
    pub fn new(
        address: u32,
        pos_a: [f64; 3],
        pos_b: [f64; 3],
        interval: f64,
        aircraft_id: usize,
    ) -> Self {
        SyncPoint {
            address,
            pos_a,
            pos_b,
            interval,
            receivers: Vec::new(),
            aircraft_id: aircraft_id,
            last_updated: 0.0,
        }
    }
}

/// Describes the current relative characteristics of a pair of clocks
#[derive(Debug)]
pub struct ClockPairing {
    /// Is this pairing valid for clock synchronization?
    pub valid: bool,
    /// Last update time (Unix timestamp)
    pub updated: f64,
    /// Variance of recent predictions
    pub variance: f64,
    /// Number of samples in the buffer
    pub n: usize,
    
    // Receiver IDs (base is always < peer for consistent ordering)
    base_id: usize,
    peer_id: usize,
    
    /// Distance category (0-3, based on 50km bins)
    cat: usize,
    
    // Clock information
    base_freq: f64,
    peer_freq: f64,
    relative_freq: f64,      // peer_freq / base_freq
    i_relative_freq: f64,    // base_freq / peer_freq
    
    // Drift tracking
    raw_drift: f64,
    drift: f64,
    i_drift: f64,
    drift_n: usize,
    drift_outliers: usize,
    drift_max: f64,
    drift_max_delta: f64,
    
    // Sample buffers (circular)
    ts_base: [f64; CP_SIZE],
    ts_peer: [f64; CP_SIZE],
    var: [f64; CP_SIZE],
    var_sum: f64,
    
    // Outlier detection
    outliers: usize,
    outlier_threshold: f64,
    outlier_total: f64,
    update_total: f64,
    outlier_reset_cooldown: usize,
    
    // Prediction error tracking
    cumulative_error: f64,
    error: f64,
    
    // Stats
    jumped: usize,
    update_attempted: f64,
    update_last_sync: f64,
    
    // Conversion factors (for optimization)
    pub factor: f64,
    pub i_factor: f64,
    pub base_avg: f64,
    pub peer_avg: f64,
}

impl ClockPairing {
    /// Create a new clock pairing
    pub fn new(
        base_id: usize,
        peer_id: usize,
        base_clock: &Clock,
        peer_clock: &Clock,
        cat: usize,
    ) -> Self {
        let relative_freq = peer_clock.freq / base_clock.freq;
        let i_relative_freq = base_clock.freq / peer_clock.freq;
        let drift_max = 0.75 * (base_clock.max_freq_error + peer_clock.max_freq_error);
        let drift_max_delta = drift_max / 20.0;
        
        // Outlier threshold: 1.5 microseconds
        // (Python comment: "this was about 2.5 us for rtl-sdr receivers")
        let outlier_threshold = 1.5e-6;
        
        ClockPairing {
            valid: false,
            updated: 0.0,
            variance: -1e-6,
            n: 0,
            
            base_id,
            peer_id,
            cat,
            
            base_freq: base_clock.freq,
            peer_freq: peer_clock.freq,
            relative_freq,
            i_relative_freq,
            
            raw_drift: 0.0,
            drift: 0.0,
            i_drift: 0.0,
            drift_n: 0,
            drift_outliers: 0,
            drift_max,
            drift_max_delta,
            
            ts_base: [0.0; CP_SIZE],
            ts_peer: [0.0; CP_SIZE],
            var: [0.0; CP_SIZE],
            var_sum: 0.0,
            
            outliers: 0,
            outlier_threshold,
            outlier_total: 0.0,
            update_total: 1e-3,
            outlier_reset_cooldown: 5,
            
            cumulative_error: 0.0,
            error: -1e-6,
            
            jumped: 0,
            update_attempted: 0.0,
            update_last_sync: 0.0,
            
            factor: 1.0,
            i_factor: 1.0,
            base_avg: 0.0,
            peer_avg: 0.0,
        }
    }
    
    /// Check if this pairing is valid for clock synchronization
    pub fn check_valid(&mut self, now: f64) -> bool {
        if self.n < 2 || self.drift_n < 2 || self.outlier_total / self.update_total > 0.5 {
            self.variance = -1e-6;
            self.error = -1e-6;
            self.valid = false;
            return false;
        }
        
        // Variance of recent predictions
        self.variance = self.var_sum / self.n as f64;
        // Standard error of recent predictions
        self.error = self.variance.sqrt();
        
        // Valid if:
        // - Outlier cooldown expired
        // - Enough samples
        // - Low variance
        // - Recently updated
        self.valid = self.outlier_reset_cooldown < 1
            && self.n > 4
            && self.drift_n > 4
            && self.variance < 16e-12
            && now - self.updated < 35.0;
        
        
        if self.valid && now > self.update_last_sync {
            self.update_last_sync = now + 5.0;
            // TODO: Update receiver last_sync times
        }
        
        self.valid
    }
    
    
    /// Update the pairing with a new sync point
    /// 
    /// Returns true if the update was used, false if it was an outlier
    pub fn update(
        &mut self,
        _address: u32,
        base_ts: f64,
        peer_ts: f64,
        base_interval: f64,
        peer_interval: f64,
        now: f64,
        sync_dont_use: bool,
    ) -> bool {
        let mut outlier = false;
        let mut do_reset = false;
        
        // Clean old data if buffer is full or data is too old
        if self.n > CP_SIZE - 1 || (self.n > 0 && base_ts - self.ts_base[0] > 50.0 * self.base_freq) {
            self.prune_old_data(now);
        }
        
        if !sync_dont_use {
            self.update_total += 1.0;
        }
        self.update_attempted = now;
        
        // Check for monotonicity - clocks must not go backwards
        if self.n > 0 && !outlier {
            let last_idx = self.n - 1;
            if peer_ts <= self.ts_peer[last_idx] || base_ts <= self.ts_base[last_idx] {
                if peer_ts < self.ts_peer[last_idx] && base_ts < self.ts_base[last_idx] {
                    return false;
                }
                if peer_ts == self.ts_peer[last_idx] || base_ts == self.ts_base[last_idx] {
                    return false;
                }
                
                // Clock went backwards - mark as invalid
                self.valid = false;
                self.outliers += 10;
                outlier = true;
                self.outlier_total += 1.0;
                
                if self.outliers <= 10 {
                    return false;
                }
            }
        }
        
        // Predict from existing data and check for outliers
        let mut prediction_error = 0.0;
        if self.n > 0 && !outlier {
            let prediction = self.predict_peer(base_ts);
            prediction_error = (prediction - peer_ts) / self.peer_freq;
            
            let outlier_threshold = if self.n >= 4 {
                self.outlier_threshold
            } else {
                2.0 * self.outlier_threshold
            };
            
            let abs_error = prediction_error.abs();
            
            if abs_error > outlier_threshold {
                outlier = true;
                self.outlier_total += 1.0;
                
                if abs_error > 3.0 * outlier_threshold {
                    self.outliers += 20;
                } else {
                    self.outliers += 6;
                }
                
                if self.outliers <= 77 && abs_error < 5.0 * outlier_threshold {
                    return false;
                }
                
                if abs_error > 3.0 * outlier_threshold {
                    do_reset = true;
                    self.jumped = 1;
                }
            }
        }
        
        if sync_dont_use {
            return false;
        }
        
        // Reset if we have a large outlier
        if outlier && do_reset {
            self.reset_offsets();
            self.outlier_reset_cooldown = 15;
            prediction_error = 0.0;
        }
        
        // Decay outlier counter
        self.outliers = self.outliers.saturating_sub(18);
        
        // Update cumulative error (for drift correction)
        if !outlier {
            self.cumulative_error = (self.cumulative_error + prediction_error).clamp(-50e-6, 50e-6);
        }
        
        self.outlier_reset_cooldown = self.outlier_reset_cooldown.saturating_sub(1);
        
        // Update drift estimate
        if !self.update_drift(base_interval, peer_interval) {
            self.check_valid(now);
            return false;
        }
        
        // Update conversion factors
        self.factor = self.relative_freq * (1.0 + self.drift);
        self.i_factor = self.i_relative_freq * (1.0 + self.i_drift);
        
        // Update offset
        self.update_offset(base_ts, peer_ts, prediction_error);
        
        self.updated = now;
        self.check_valid(now);
        true
    }
    
    /// Prune old data from the circular buffers
    fn prune_old_data(&mut self, now: f64) {
        let mut i = 0;
        
        // Keep at most CP_SIZE - 12 samples
        let new_max = CP_SIZE - 12;
        if self.n > new_max {
            i = self.n - new_max;
        }
        
        // Also prune samples older than 45 seconds
        if self.n > 0 {
            let latest_base_ts = self.ts_base[self.n - 1];
            let limit = 45.0 * self.base_freq;
            while i < self.n && (latest_base_ts - self.ts_base[i]) > limit {
                i += 1;
            }
        }
        
        if i > 0 {
            // Shift arrays
            self.n -= i;
            self.ts_base.copy_within(i.., 0);
            self.ts_peer.copy_within(i.., 0);
            self.var.copy_within(i.., 0);
            
            // Recalculate variance sum
            self.var_sum = self.var[..self.n].iter().sum();
            self.check_valid(now);
        }
    }
    
    /// Update drift estimate based on interval ratio
    /// 
    /// Returns false if the update should be rejected
    fn update_drift(&mut self, base_interval: f64, peer_interval: f64) -> bool {
        // Avoid catastrophic cancellation
        let adjusted_base_interval = base_interval * self.relative_freq;
        let new_drift = (peer_interval - adjusted_base_interval) / adjusted_base_interval;
        
        // Reject if drift is too large
        if new_drift.abs() > self.drift_max {
            return false;
        }
        
        // First sample or too many outliers - trust it outright
        if self.drift_n == 0 || self.drift_outliers > 15 {
            self.raw_drift = new_drift;
            self.drift = new_drift;
            self.i_drift = -self.drift / (1.0 + self.drift);
            self.cumulative_error = 0.0;
            self.drift_outliers = 0;
            self.drift_n = 1;
            return true;
        }
        
        // Check if new drift is too far from current estimate
        let drift_error = new_drift - self.raw_drift;
        if drift_error.abs() > self.drift_max_delta {
            self.drift_outliers += 1;
            return false;
        }
        
        self.drift_outliers = self.drift_outliers.saturating_sub(2);
        
        // PI controller constants
        let mut kp = 0.06;
        let ki = 0.008;
        
        // Allow quicker adjustment for new pairs
        let unstable = (DRIFT_N_STABLE as i32 - self.drift_n as i32).max(0) as usize;
        if unstable > 0 {
            let adjustment_factor = 1.0 + (0.2 / kp) * (unstable as f64 / DRIFT_N_STABLE as f64);
            kp *= adjustment_factor;
        }
        
        self.drift_n += 1;
        
        // Move towards the new value (PI controller)
        self.raw_drift += drift_error * kp;
        self.drift = self.raw_drift - ki * self.cumulative_error;
        self.i_drift = -self.drift / (1.0 + self.drift);
        
        true
    }
    
    /// Reset offset estimates (but keep drift)
    pub fn reset_offsets(&mut self) {
        self.valid = false;
        self.n = 0;
        self.var_sum = 0.0;
        self.error = -1e-6;
        self.variance = -1e-6;
        self.outliers = 0;
        self.cumulative_error = 0.0;
        self.base_avg = 0.0;
        self.peer_avg = 0.0;
    }
    
    /// Update offset based on new timestamp pair
    fn update_offset(&mut self, base_ts: f64, peer_ts: f64, prediction_error: f64) {
        let p_var = prediction_error * prediction_error;
        
        // Add to buffers
        self.ts_base[self.n] = base_ts;
        self.ts_peer[self.n] = peer_ts;
        self.var[self.n] = p_var;
        
        // Update running averages with exponential decay
        if self.n == 0 {
            self.base_avg = base_ts;
            self.peer_avg = peer_ts;
        } else {
            let elapsed = base_ts - self.ts_base[self.n - 1];
            let max_elapsed = 20.0;
            
            if elapsed < max_elapsed {
                let keep = ((1.0 - elapsed / max_elapsed).powi(2)) * 0.75;
                self.base_avg = self.base_avg * keep + base_ts * (1.0 - keep);
                self.peer_avg = self.peer_avg * keep + peer_ts * (1.0 - keep);
            } else {
                self.base_avg = base_ts;
                self.peer_avg = peer_ts;
            }
        }
        
        self.n += 1;
        self.var_sum += p_var;
    }
    
    /// Predict peer timestamp from base timestamp
    /// 
    /// Uses the simplified linear model from Python
    pub fn predict_peer(&self, base_ts: f64) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        self.peer_avg + (base_ts - self.base_avg) * self.factor
    }
    
    /// Predict base timestamp from peer timestamp
    pub fn predict_base(&self, peer_ts: f64) -> f64 {
        if self.n == 0 {
            return 0.0;
        }
        self.base_avg + (peer_ts - self.peer_avg) * self.i_factor
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_clock_creation() {
        let clock = Clock::from_type(ClockType::RadarcapeGps);
        assert_eq!(clock.freq, 1e9);
        assert_eq!(clock.max_freq_error, 1e-6);
        assert_eq!(clock.jitter, 15e-9);
        assert!((clock.delay_factor - 1e9 / (299792458.0 / 1.00032)).abs() < 1e-6);
    }
    
    #[test]
    fn test_clock_pairing_creation() {
        let base_clock = Clock::from_type(ClockType::Beast);
        let peer_clock = Clock::from_type(ClockType::RadarcapeGps);
        
        let pairing = ClockPairing::new(0, 1, &base_clock, &peer_clock, 0);
        
        assert!(!pairing.valid);
        assert_eq!(pairing.n, 0);
        assert_eq!(pairing.base_freq, 12e6);
        assert_eq!(pairing.peer_freq, 1e9);
        assert!((pairing.relative_freq - 1e9 / 12e6).abs() < 1e-6);
    }
    
    #[test]
    fn test_sync_point_creation() {
        let sp = SyncPoint::new(
            0xABCDEF,
            [4000000.0, 0.0, 5000000.0],
            [4000100.0, 0.0, 5000000.0],
            0.5,
            123,
        );
        
        assert_eq!(sp.address, 0xABCDEF);
        assert_eq!(sp.interval, 0.5);
        assert_eq!(sp.receivers.len(), 0);
    }
}

/// Helper function: Get sync peer limit for a distance category
fn get_limit(cat: usize) -> usize {
    match cat {
        0 | 1 => 32,
        2 | 3 => 16,
        _ => 16,
    }
}

/// Helper function: Calculate ECEF distance between two points
fn ecef_distance(p0: &[f64; 3], p1: &[f64; 3]) -> f64 {
    let dx = p0[0] - p1[0];
    let dy = p0[1] - p1[1];
    let dz = p0[2] - p1[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Maximum distance between two positions in the same sync message pair (10km)
const MAX_INTERMESSAGE_RANGE: f64 = 10e3;

/// Speed of light in air (m/s) for propagation delay
#[allow(dead_code)]
const C_AIR: f64 = 299792458.0 / 1.00032;

/// Feet to meters conversion
const FTOM: f64 = 0.3048;

/// Receiver info needed for sync operations (avoids needing full Receiver struct)
#[derive(Debug, Clone)]
pub struct SyncReceiverInfo {
    pub uid: usize,
    pub position: [f64; 3],     // ECEF
    pub clock: Clock,
    pub distance: HashMap<usize, f64>,  // uid -> distance in meters
    pub sync_peers: [usize; 5],
    pub last_sync: f64,
    pub bad_syncs: f64,
    pub dead: bool,
    pub sync_range_exceeded: usize,     // counter of range exceeded errors
}

/// Stored state for a sync point key: (message details, list of sync points, or invalid marker)
#[derive(Debug, Clone)]
enum SyncPointState {
    /// Invalid — should be ignored
    Invalid,
    /// Valid with message details and a list of sync points matching this key
    Valid {
        address: u32,
        pos_a: [f64; 3],
        pos_b: [f64; 3],
        syncpoints: Vec<SyncPoint>,
    },
}

/// Clock tracker - manages all clock pairings and sync points
pub struct ClockTracker {
    /// Map of (receiver_id_0, receiver_id_1) -> ClockPairing
    clock_pairs: HashMap<(usize, usize), ClockPairing>,
    /// Sync point cache: (even_msg_bytes, odd_msg_bytes) -> SyncPointState
    sync_points: HashMap<(Vec<u8>, Vec<u8>), SyncPointState>,
}

impl ClockTracker {
    /// Create a new clock tracker
    pub fn new() -> Self {
        ClockTracker {
            clock_pairs: HashMap::new(),
            sync_points: HashMap::new(),
        }
    }
    
    /// Get or create a clock pairing between two receivers
    pub fn get_or_create_pairing(
        &mut self,
        receiver_id_0: usize,
        receiver_id_1: usize,
        clock_0: &Clock,
        clock_1: &Clock,
        distance_meters: f64,
    ) -> &mut ClockPairing {
        let (base_id, peer_id, base_clock, peer_clock) = if receiver_id_0 < receiver_id_1 {
            (receiver_id_0, receiver_id_1, clock_0, clock_1)
        } else {
            (receiver_id_1, receiver_id_0, clock_1, clock_0)
        };
        
        let cat = ((distance_meters / 50e3) as usize).min(3);
        
        self.clock_pairs
            .entry((base_id, peer_id))
            .or_insert_with(|| ClockPairing::new(base_id, peer_id, base_clock, peer_clock, cat))
    }
    
    /// Get an existing pairing
    pub fn get_pairing(&self, receiver_id_0: usize, receiver_id_1: usize) -> Option<&ClockPairing> {
        let key = if receiver_id_0 < receiver_id_1 {
            (receiver_id_0, receiver_id_1)
        } else {
            (receiver_id_1, receiver_id_0)
        };
        self.clock_pairs.get(&key)
    }
    
    /// Get a mutable reference to an existing pairing
    pub fn get_pairing_mut(&mut self, receiver_id_0: usize, receiver_id_1: usize) -> Option<&mut ClockPairing> {
        let key = if receiver_id_0 < receiver_id_1 {
            (receiver_id_0, receiver_id_1)
        } else {
            (receiver_id_1, receiver_id_0)
        };
        self.clock_pairs.get_mut(&key)
    }
    
    /// Clean up expired pairings
    pub fn cleanup(&mut self, now: f64) {
        self.clock_pairs.retain(|_, pairing| {
            pairing.check_valid(now);
            (now - pairing.updated <= 45.0)
                && (pairing.valid || now - pairing.updated <= 30.0)
        });
    }
    
    /// Clear all sync points (called periodically by coordinator, every ~15s)
    pub fn clear_all_sync_points(&mut self) {
        self.sync_points.clear();
    }
    
    /// Get the number of active pairings
    pub fn num_pairings(&self) -> usize {
        self.clock_pairs.len()
    }
    
    /// Get the number of valid pairings
    pub fn num_valid_pairings(&self) -> usize {
        self.clock_pairs.values().filter(|p| p.valid).count()
    }

    /// Dump receiver state for write_state (sync.json). Returns per-pairing data; coordinator keys by user.
    /// Decays outlier_total/update_total like Python. Each (base_id, peer_id) yields one entry.
    pub fn dump_receiver_state(&mut self, now: f64) -> Vec<((usize, usize), (u32, f64, f64, f64, f64, usize, i64))> {
        let mut out = Vec::new();
        for ((base_id, peer_id), pairing) in self.clock_pairs.iter_mut() {
            if pairing.n < 2 {
                continue;
            }
            let outlier_pct = if pairing.update_total < 4.0 {
                50.0 * pairing.outlier_total / pairing.update_total
            } else {
                100.0 * pairing.outlier_total / pairing.update_total
            };
            pairing.outlier_total /= 2.0;
            pairing.update_total /= 2.0;
            let time_since = (now - pairing.updated).round() as i64;
            let row = (
                pairing.n as u32,
                (pairing.error * 1e6).round() as f64,
                (pairing.drift * 1e6).round() as f64,
                (pairing.i_drift * 1e6).round() as f64,
                (outlier_pct * 10.0).round() / 10.0,
                pairing.jumped,
                time_since,
            );
            out.push(((*base_id, *peer_id), row));
            pairing.jumped = 0;
        }
        out
    }

    /// Reset clock offsets for a receiver (on clock reset / reconnect)
    /// Mirrors Python receiver_clock_reset
    pub fn receiver_clock_reset(&mut self, receiver_id: usize) {
        for ((base_id, peer_id), pairing) in self.clock_pairs.iter_mut() {
            if *base_id == receiver_id || *peer_id == receiver_id {
                pairing.reset_offsets();
            }
        }
    }

    /// Remove all pairings involving a receiver (on disconnect)
    /// Mirrors Python receiver_disconnect
    pub fn receiver_disconnect(&mut self, receiver_id: usize, receivers: &mut HashMap<usize, SyncReceiverInfo>) {
        let to_remove: Vec<(usize, usize)> = self.clock_pairs.keys()
            .filter(|&&(base, peer)| base == receiver_id || peer == receiver_id)
            .copied()
            .collect();
        
        for key in to_remove {
            if let Some(pairing) = self.clock_pairs.remove(&key) {
                // Decrement sync_peers for both receivers
                if let Some(r) = receivers.get_mut(&key.0) {
                    if pairing.cat < r.sync_peers.len() {
                        r.sync_peers[pairing.cat] = r.sync_peers[pairing.cat].saturating_sub(1);
                    }
                }
                if let Some(r) = receivers.get_mut(&key.1) {
                    if pairing.cat < r.sync_peers.len() {
                        r.sync_peers[pairing.cat] = r.sync_peers[pairing.cat].saturating_sub(1);
                    }
                }
            }
        }
    }

    /// Process a sync message from a receiver.
    /// Mirrors Python `ClockTracker.receiver_sync` (clocktrack.pyx L299-496).
    ///
    /// # Arguments
    /// * `receiver` - Info about the reporting receiver
    /// * `even_time` - Timestamp of even message in receiver clock units
    /// * `odd_time` - Timestamp of odd message in receiver clock units
    /// * `even_msg_bytes` - Decoded (not hex) even message bytes
    /// * `odd_msg_bytes` - Decoded (not hex) odd message bytes
    /// * `receivers` - Map of all receivers for sync (mutable for sync_peers updates)
    /// * `now` - Current wall time (seconds since epoch)
    pub fn receiver_sync(
        &mut self,
        receiver: &SyncReceiverInfo,
        even_time: f64,
        odd_time: f64,
        even_msg_bytes: &[u8],
        odd_msg_bytes: &[u8],
        receivers: &mut HashMap<usize, SyncReceiverInfo>,
        now: f64,
    ) {
        // Order messages by timestamp → (tA, tB) where tA < tB
        let (t_a, t_b, key) = if even_time < odd_time {
            (even_time, odd_time, (even_msg_bytes.to_vec(), odd_msg_bytes.to_vec()))
        } else {
            (odd_time, even_time, (odd_msg_bytes.to_vec(), even_msg_bytes.to_vec()))
        };

        // Compute interval in seconds
        let interval = (t_b - t_a) / receiver.clock.freq;

        // Messages must be within 5 seconds of each other
        if interval > 5.0 {
            return;
        }

        // Check if we have an existing entry for this message pair
        if let Some(state) = self.sync_points.get_mut(&key) {
            match state {
                SyncPointState::Invalid => return,
                SyncPointState::Valid { address, pos_a, pos_b, syncpoints } => {
                    // Check if any existing syncpoint has a matching interval (within 0.75ms)
                    let address = *address;
                    let pos_a = *pos_a;
                    let pos_b = *pos_b;
                    let mut found_match = false;

                    for sp in syncpoints.iter_mut() {
                        // Match by interval (same as Python: 0.75 ms) and by recency so we pair
                        // receivers that saw the same sync event.
                        if (sp.interval - interval).abs() < 0.75e-3 && (sp.last_updated - now).abs() < 2.0 {
                            Self::add_to_existing_syncpoint(
                                &mut self.clock_pairs,
                                sp,
                                receiver,
                                t_a,
                                t_b,
                                receivers,
                                now,
                            );
                            found_match = true;
                            break;
                        }
                    }

                    if !found_match {
                        // No matching existing syncpoint — create new syncpoint in the list
                        let mut new_sp = SyncPoint::new(address, pos_a, pos_b, interval, 0);
                        Self::add_to_existing_syncpoint(
                            &mut self.clock_pairs,
                            &mut new_sp,
                            receiver,
                            t_a,
                            t_b,
                            receivers,
                            now,
                        );
                        syncpoints.push(new_sp);
                    }
                    return;
                }
            }
        }

        // No existing entry — decode and validate the messages to create a new sync point

        // Decode even message
        let even_decoded = match crate::modes::message::decode(even_msg_bytes) {
            Some(m) => m,
            None => return,
        };

        // Decode odd message
        let odd_decoded = match crate::modes::message::decode(odd_msg_bytes) {
            Some(m) => m,
            None => return,
        };

        // Both must be DF17 with valid CRC
        let (even_df17, odd_df17) = match (&even_decoded, &odd_decoded) {
            (crate::modes::message::DecodedMessage::DF17(e), crate::modes::message::DecodedMessage::DF17(o)) => (e, o),
            _ => {
                // Not DF17 — mark as invalid so we don't try again
                self.sync_points.insert(key, SyncPointState::Invalid);
                return;
            }
        };

        // CRC check
        if !even_df17.crc_ok || !odd_df17.crc_ok {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // F flag check: even must have F=0, odd must have F=1
        if even_df17.f != Some(0) || odd_df17.f != Some(1) {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // Same aircraft
        if even_df17.address != odd_df17.address {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }
        let address = even_df17.address;

        // Both must be airborne position
        if even_df17.estype != crate::modes::message::ESType::AirbornePosition
            || odd_df17.estype != crate::modes::message::ESType::AirbornePosition
        {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // Need lat/lon from both
        let (even_lat, even_lon, odd_lat, odd_lon) = match (even_df17.lat, even_df17.lon, odd_df17.lat, odd_df17.lon) {
            (Some(el), Some(eo), Some(ol), Some(oo)) => (el, eo, ol, oo),
            _ => {
                self.sync_points.insert(key, SyncPointState::Invalid);
                return;
            }
        };

        // Decode CPR to global positions
        let (rlat_e, rlon_e, rlat_o, rlon_o) = match crate::modes::cpr::decode_cpr(even_lat, even_lon, odd_lat, odd_lon) {
            Ok(pos) => pos,
            Err(_) => {
                self.sync_points.insert(key, SyncPointState::Invalid);
                return;
            }
        };

        // Need altitude from both
        let (even_alt, odd_alt) = match (even_df17.altitude, odd_df17.altitude) {
            (Some(ea), Some(oa)) => (ea, oa),
            _ => {
                self.sync_points.insert(key, SyncPointState::Invalid);
                return;
            }
        };

        // Altitude consistency check
        if (even_alt - odd_alt).unsigned_abs() > 5000 {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // Latitude quality check
        if rlat_e > 85.0 || rlat_e < -85.0 || rlat_e == 0.0 || rlon_e == 0.0 {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // NUC quality check (need NUC >= 6)
        let even_nuc = even_df17.nuc.unwrap_or(0);
        let odd_nuc = odd_df17.nuc.unwrap_or(0);
        if even_nuc < 6 || odd_nuc < 6 {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // Convert to ECEF
        let even_ecef_tuple = crate::geodesy::llh2ecef(rlat_e, rlon_e, even_alt as f64 * FTOM);
        let even_ecef = [even_ecef_tuple.0, even_ecef_tuple.1, even_ecef_tuple.2];

        let odd_ecef_tuple = crate::geodesy::llh2ecef(rlat_o, rlon_o, odd_alt as f64 * FTOM);
        let odd_ecef = [odd_ecef_tuple.0, odd_ecef_tuple.1, odd_ecef_tuple.2];

        // Inter-message range check
        if ecef_distance(&even_ecef, &odd_ecef) > MAX_INTERMESSAGE_RANGE {
            self.sync_points.insert(key, SyncPointState::Invalid);
            return;
        }

        // Order positions by timestamp (pos_a = earlier message position)
        let (pos_a, pos_b) = if even_time < odd_time {
            (even_ecef, odd_ecef)
        } else {
            (odd_ecef, even_ecef)
        };

        // Receiver range check
        if ecef_distance(&even_ecef, &receiver.position) > MAX_RANGE as f64 {
            if let Some(r) = receivers.get_mut(&receiver.uid) {
                r.sync_range_exceeded = r.sync_range_exceeded.saturating_add(1);
            }
            return;
        }

        // Valid! Create new sync point
        let mut syncpoint = SyncPoint::new(address, pos_a, pos_b, interval, 0);

        // Add to existing syncpoint (which also processes clock pairing)
        Self::add_to_existing_syncpoint(
            &mut self.clock_pairs,
            &mut syncpoint,
            receiver,
            t_a,
            t_b,
            receivers,
            now,
        );

        // Store in cache
        self.sync_points.insert(key, SyncPointState::Valid {
            address,
            pos_a,
            pos_b,
            syncpoints: vec![syncpoint],
        });
    }

    /// Add a receiver to an existing sync point and update clock pairings.
    /// Mirrors Python `_add_to_existing_syncpoint` (clocktrack.pyx L111-215).
    fn add_to_existing_syncpoint(
        clock_pairs: &mut HashMap<(usize, usize), ClockPairing>,
        syncpoint: &mut SyncPoint,
        r0: &SyncReceiverInfo,
        t0_a: f64,
        t0_b: f64,
        receivers: &mut HashMap<usize, SyncReceiverInfo>,
        now: f64,
    ) {
        // Compute distances from receiver to sync point positions
        let receiver_dist_a = ecef_distance(&syncpoint.pos_a, &r0.position);
        let receiver_dist_b = ecef_distance(&syncpoint.pos_b, &r0.position);

        // Receiver range check
        if receiver_dist_a > MAX_RANGE as f64 {
            return;
        }

        // Propagation delays in clock units
        let delay_factor = r0.clock.delay_factor;
        let delay0_a = receiver_dist_a * delay_factor;
        let delay0_b = receiver_dist_b * delay_factor;

        // Delay-corrected timestamps
        let td0_a = t0_a - delay0_a;
        let td0_b = t0_b - delay0_b;

        // Compute interval adjusted for transmitter motion
        let i0 = td0_b - td0_a;

        // Update last_updated
        syncpoint.last_updated = now;

        let r0_shard = SyncShard {
            receiver_id: r0.uid,
            td: td0_b,
            i: i0,
        };

        // Try to sync with all receivers already in this syncpoint
        for r1_shard in syncpoint.receivers.iter() {
            let r1_id = r1_shard.receiver_id;
            if r1_id == r0.uid {
                continue;
            }

            let td1_b = r1_shard.td;
            let i1 = r1_shard.i;

            // Get receiver r1 info
            let r1 = match receivers.get(&r1_id) {
                Some(r) => r,
                None => continue,
            };
            if r1.dead {
                continue;
            }

            // Order the clock pair (base < peer)
            let (base_id, peer_id) = if r0.uid < r1_id {
                (r0.uid, r1_id)
            } else {
                (r1_id, r0.uid)
            };
            let pair_key = (base_id, peer_id);

            // Get distance between receivers
            let distance = match r0.distance.get(&r1_id) {
                Some(&d) => d,
                None => {
                    continue;
                }
            };
            let cat = (distance / 50e3).min(3.0) as usize;

            // Get or create pairing
            let pairing = clock_pairs.entry(pair_key).or_insert_with(|| {
                let _limit = (0.7 * get_limit(cat) as f64) as usize;
                let _p0 = r0.sync_peers[cat];
                let _p1 = r1.sync_peers[cat];

                // Check peer limits before creating
                // (we can't easily increment sync_peers here because of borrow checker,
                //  so we do it after the loop — slight deviation from Python)
                let (base_clock, peer_clock) = if r0.uid < r1_id {
                    (&r0.clock, &r1.clock)
                } else {
                    (&r1.clock, &r0.clock)
                };
                ClockPairing::new(base_id, peer_id, base_clock, peer_clock, cat)
            });

            // Rate limit existing pairings
            if pairing.n > 8 && now - pairing.update_attempted < 0.2 {
                continue;
            }

            // Call update with properly ordered timestamps
            if r0.uid < r1_id {
                pairing.update(syncpoint.address, td0_b, td1_b, i0, i1, now, false);
            } else {
                pairing.update(syncpoint.address, td1_b, td0_b, i1, i0, now, false);
            }
        }

        // Add this receiver's shard to the syncpoint
        syncpoint.receivers.push(r0_shard);
    }

    /// Backward-compatible wrapper: process a sync message from a receiver.
    /// Prefer `receiver_sync` which is the full implementation.
    #[allow(dead_code)]
    pub fn process_message(
        &mut self,
        _receiver_id: usize,
        _even_ts: f64,
        _odd_ts: f64,
        _even_msg: &[u8],
        _odd_msg: &[u8],
    ) {
        // Deprecated: use receiver_sync() instead.
        // This method cannot perform proper sync because it lacks receiver position/clock info.
    }
    
    /// Normalize timestamps from multiple receivers into a common timescale.
    ///
    /// Given a map of receiver_id -> [(timestamp, utc), ...], returns a list of
    /// components where timestamps are normalized to a common base within each component.
    ///
    /// Each component is a HashMap of receiver_id -> (variance, [(normalized_timestamp, utc), ...])
    ///
    /// This implements the normalize2 algorithm from Python clocktrack.pyx lines 1270-1441.
    /// It builds a weighted graph of valid clock pairings, finds the MST, picks a central
    /// node, and converts all timestamps through the spanning tree edges.
    pub fn normalize2(
        &self,
        timestamp_map: &HashMap<usize, Vec<(f64, f64)>>,
        receiver_clocks: &HashMap<usize, Clock>,
    ) -> Vec<HashMap<usize, (f64, Vec<(f64, f64)>)>> {
        let receivers: Vec<usize> = {
            let mut r: Vec<usize> = timestamp_map.keys().copied().collect();
            r.sort();
            r
        };
        let reclen = receivers.len();
        if reclen < 3 {
            return vec![];
        }
        
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();

        // Build index map for fast lookup: receiver_id -> index
        let _index_of: HashMap<usize, usize> = receivers.iter().enumerate()
            .map(|(i, &r)| (r, i))
            .collect();

        // Build adjacency list with predictor data
        // For each valid pairing, create forward and reverse predictors
        let mut edges: Vec<(usize, usize, f64)> = Vec::new(); // (idx_i, idx_j, variance)
        let mut predictor_map: HashMap<(usize, usize), Predictor> = HashMap::new();
        
        for (i, &si) in receivers.iter().enumerate() {
            for (j, &sj) in receivers.iter().enumerate() {
                if si < sj {
                    if let Some(predictors) = make_predictors(&self.clock_pairs, si, sj, now) {
                        predictor_map.insert((si, sj), predictors.0.clone());
                        predictor_map.insert((sj, si), predictors.1.clone());
                        edges.push((i, j, predictors.0.variance));
                    } else {
                        // Check for GPS sync (RadarcapeGps)
                        let ci = receiver_clocks.get(&si);
                        let cj = receiver_clocks.get(&sj);
                        if let (Some(c_i), Some(c_j)) = (ci, cj) {
                            if matches!(c_i.clock_type, ClockType::RadarcapeGps) && matches!(c_j.clock_type, ClockType::RadarcapeGps) {
                                // Both are GPS-synced, assume perfect sync (variance 1ns^2)
                                let p0 = Predictor {
                                    target_ref: 0.0,
                                    source_ref: 0.0,
                                    factor: 1.0,
                                    variance: 1e-18,
                                };
                                let p1 = Predictor {
                                    target_ref: 0.0,
                                    source_ref: 0.0,
                                    factor: 1.0,
                                    variance: 1e-18,
                                };
                                predictor_map.insert((si, sj), p0);
                                predictor_map.insert((sj, si), p1);
                                edges.push((i, j, 1e-18));
                            }
                        }
                    }
                }
            }
        }
        
        if edges.len() < 2 {
            return vec![];
        }

        // Build MST using Prim's algorithm
        let mst_edges = prim_mst(reclen, &edges);

        // Find connected components in the MST
        let components = find_connected_components(reclen, &mst_edges);
        
        // Find the largest component
        let mut comp_sizes: HashMap<usize, Vec<usize>> = HashMap::new();
        for (node, &label) in components.iter().enumerate() {
            comp_sizes.entry(label).or_default().push(node);
        }
        let mut comp_list: Vec<Vec<usize>> = comp_sizes.into_values().collect();
        comp_list.sort_by(|a, b| b.len().cmp(&a.len()));
        
        if comp_list.is_empty() || comp_list[0].len() < 3 {
            return vec![];
        }
        
        let big_comp = &comp_list[0];
        let big_comp_set: std::collections::HashSet<usize> = big_comp.iter().copied().collect();

        // Build adjacency list for MST restricted to biggest component
        let mut adj: Vec<Vec<(usize, f64)>> = vec![vec![]; reclen];
        for &(u, v, w) in &mst_edges {
            if big_comp_set.contains(&u) && big_comp_set.contains(&v) {
                adj[u].push((v, w));
                adj[v].push((u, w));
            }
        }

        // Find central node: minimizes the maximum path cost to any node
        let root = big_comp[0];
        let heights = label_heights(&adj, root);
        
        // Find two tallest branches from root
        let tall1 = tallest_branch(&adj, root, &heights, None);
        let tall2 = tallest_branch(&adj, root, &heights, tall1.1);
        
        let target = (tall1.0 + tall2.0) / 2.0;
        let mut central = root;
        let mut step = tall1.1;
        while let Some(s) = step {
            if (heights[&central] - target).abs() <= (heights[&s] - target).abs() {
                break;
            }
            central = s;
            let tb = tallest_branch(&adj, central, &heights, Some(central));
            step = tb.1;
        }

        // Convert timestamps: walk MST from central node
        // The final conversion adds a frequency->wallclock step
        let central_recv = receivers[central];
        let central_clock = receiver_clocks.get(&central_recv)
            .cloned()
            .unwrap_or(Clock::from_type(crate::clocktrack::ClockType::Beast));
        
        let factor = 1.0 / central_clock.freq;
        let initial_pred = Predictor {
            target_ref: 0.0,
            source_ref: 0.0,
            factor,
            variance: central_clock.jitter * central_clock.jitter,
        };

        let mut results: HashMap<usize, (f64, Vec<(f64, f64)>)> = HashMap::new();
        convert_timestamps(
            &adj,
            timestamp_map,
            &predictor_map,
            &receivers,
            central,
            &mut results,
            &[initial_pred],
            central_clock.jitter * central_clock.jitter,
        );

        if results.is_empty() {
            vec![]
        } else {
            vec![results]
        }
    }
}

/// Predictor for converting timestamps between two clocks
/// Mirrors Python _Predictor class (clocktrack.pyx lines 1025-1037)
#[derive(Debug, Clone)]
pub struct Predictor {
    pub target_ref: f64,
    pub source_ref: f64,
    pub factor: f64,
    pub variance: f64,
}

/// Create forward and reverse predictors for a pair of receivers
/// Mirrors Python _make_predictors (clocktrack.pyx lines 1044-1085)
fn make_predictors(
    clock_pairs: &HashMap<(usize, usize), ClockPairing>,
    station0: usize,
    station1: usize,
    now: f64,
) -> Option<(Predictor, Predictor)> {
    let key = if station0 < station1 {
        (station0, station1)
    } else {
        (station1, station0)
    };
    
    let pairing = clock_pairs.get(&key)?;
    if !pairing.valid {
        return None;
    }
    
    let mut variance = pairing.variance;
    let stale = now - pairing.updated;
    
    // Increase variance for stale pairings
    variance *= 1.0 + stale / 60.0;
    
    // Increase variance for pairings with fewer sync points
    if pairing.n < 10 {
        variance *= 1.0 + (10 - pairing.n) as f64 * 0.05;
    }
    
    let base_predictor = Predictor {
        target_ref: pairing.base_avg,
        source_ref: pairing.peer_avg,
        factor: pairing.i_factor,
        variance,
    };
    let peer_predictor = Predictor {
        target_ref: pairing.peer_avg,
        source_ref: pairing.base_avg,
        factor: pairing.factor,
        variance,
    };
    
    if station0 < station1 {
        Some((peer_predictor, base_predictor))
    } else {
        Some((base_predictor, peer_predictor))
    }
}

/// Prim's MST algorithm on adjacency specified as edge list
fn prim_mst(n: usize, edges: &[(usize, usize, f64)]) -> Vec<(usize, usize, f64)> {
    if n == 0 || edges.is_empty() {
        return vec![];
    }
    
    // Build adjacency list
    let mut adj: Vec<Vec<(usize, f64)>> = vec![vec![]; n];
    for &(u, v, w) in edges {
        adj[u].push((v, w));
        adj[v].push((u, w));
    }
    
    let mut in_mst = vec![false; n];
    let mut mst_edges = Vec::new();
    
    // Find a node that has edges
    let start = edges.iter().map(|e| e.0).next().unwrap_or(0);
    in_mst[start] = true;
    
    // Simple Prim's: at each step pick the minimum weight edge crossing the cut
    // For the sizes we deal with (<100 receivers), O(V*E) is fine
    loop {
        let mut best: Option<(usize, usize, f64)> = None;
        
        for u in 0..n {
            if !in_mst[u] { continue; }
            for &(v, w) in &adj[u] {
                if in_mst[v] { continue; }
                if best.is_none() || w < best.unwrap().2 {
                    best = Some((u, v, w));
                }
            }
        }
        
        match best {
            Some((u, v, w)) => {
                in_mst[v] = true;
                mst_edges.push((u, v, w));
            }
            None => break,
        }
    }
    
    mst_edges
}

/// Find connected components in the MST
fn find_connected_components(n: usize, edges: &[(usize, usize, f64)]) -> Vec<usize> {
    let mut labels = vec![usize::MAX; n];
    let mut adj: Vec<Vec<usize>> = vec![vec![]; n];
    
    for &(u, v, _) in edges {
        adj[u].push(v);
        adj[v].push(u);
    }
    
    let mut label = 0;
    for start in 0..n {
        if labels[start] != usize::MAX { continue; }
        // Check that this node has edges or was part of the graph
        if adj[start].is_empty() {
            labels[start] = label;
            label += 1;
            continue;
        }
        // BFS
        let mut queue = std::collections::VecDeque::new();
        queue.push_back(start);
        labels[start] = label;
        while let Some(node) = queue.pop_front() {
            for &neighbor in &adj[node] {
                if labels[neighbor] == usize::MAX {
                    labels[neighbor] = label;
                    queue.push_back(neighbor);
                }
            }
        }
        label += 1;
    }
    
    labels
}

/// Label each node with its height (longest path to any leaf)
/// Mirrors Python _label_heights (clocktrack.pyx lines 1087-1099)
fn label_heights(adj: &[Vec<(usize, f64)>], root: usize) -> HashMap<usize, f64> {
    let mut heights: HashMap<usize, f64> = HashMap::new();
    label_heights_recursive(adj, root, &mut heights);
    heights
}

fn label_heights_recursive(adj: &[Vec<(usize, f64)>], node: usize, heights: &mut HashMap<usize, f64>) {
    heights.insert(node, 0.0);
    for &(neighbor, weight) in &adj[node] {
        if !heights.contains_key(&neighbor) {
            label_heights_recursive(adj, neighbor, heights);
            let mn = heights[&neighbor] + weight;
            if mn > heights[&node] {
                heights.insert(node, mn);
            }
        }
    }
}

/// Find the tallest branch from a node, optionally ignoring one neighbor
/// Returns (height, Some(neighbor)) or (0, None)
/// Mirrors Python _tallest_branch (clocktrack.pyx lines 1102-1116)
fn tallest_branch(
    adj: &[Vec<(usize, f64)>],
    node: usize,
    heights: &HashMap<usize, f64>,
    ignore: Option<usize>,
) -> (f64, Option<usize>) {
    let mut tallest = (0.0, None);
    
    for &(neighbor, weight) in &adj[node] {
        if Some(neighbor) == ignore {
            continue;
        }
        let eh = heights.get(&neighbor).copied().unwrap_or(0.0) + weight;
        if eh > tallest.0 {
            tallest = (eh, Some(neighbor));
        }
    }
    
    tallest
}

/// Walk the MST from a root node, converting all timestamps through
/// the chain of predictors.
/// Mirrors Python _convert_timestamps (clocktrack.pyx lines 1119-1146)
fn convert_timestamps(
    adj: &[Vec<(usize, f64)>],
    timestamp_map: &HashMap<usize, Vec<(f64, f64)>>,
    predictor_map: &HashMap<(usize, usize), Predictor>,
    receivers: &[usize],
    node: usize,
    results: &mut HashMap<usize, (f64, Vec<(f64, f64)>)>,
    conversion_chain: &[Predictor],
    variance: f64,
) {
    let receiver_id = receivers[node];
    
    // Convert our own timestamps using the provided chain
    let mut converted = Vec::new();
    if let Some(timestamps) = timestamp_map.get(&receiver_id) {
        for &(ts, utc) in timestamps {
            let mut converted_ts = ts;
            for pred in conversion_chain {
                converted_ts = pred.target_ref + (converted_ts - pred.source_ref) * pred.factor;
            }
            converted.push((converted_ts, utc));
        }
    }
    results.insert(receiver_id, (variance, converted));
    
    // Convert all reachable unvisited neighbors
    for &(neighbor, _) in &adj[node] {
        let neighbor_id = receivers[neighbor];
        if results.contains_key(&neighbor_id) {
            continue;
        }
        
        // Get predictor for neighbor -> node conversion
        if let Some(pred) = predictor_map.get(&(neighbor_id, receiver_id)) {
            let mut new_chain = vec![pred.clone()];
            new_chain.extend_from_slice(conversion_chain);
            convert_timestamps(
                adj,
                timestamp_map,
                predictor_map,
                receivers,
                neighbor,
                results,
                &new_chain,
                variance + pred.variance,
            );
        }
    }
}

impl Default for ClockTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tracker_tests {
    use super::*;
    
    #[test]
    fn test_clock_tracker_creation() {
        let tracker = ClockTracker::new();
        assert_eq!(tracker.num_pairings(), 0);
        assert_eq!(tracker.num_valid_pairings(), 0);
    }
    
    #[test]
    fn test_clock_tracker_pairing() {
        let mut tracker = ClockTracker::new();
        let clock1 = Clock::from_type(ClockType::Beast);
        let clock2 = Clock::from_type(ClockType::RadarcapeGps);
        
        let pairing = tracker.get_or_create_pairing(1, 2, &clock1, &clock2, 100e3);
        assert!(!pairing.valid);
        assert_eq!(tracker.num_pairings(), 1);
        
        let pairing2 = tracker.get_pairing(2, 1);
        assert!(pairing2.is_some());
        
        let pairing3 = tracker.get_pairing(1, 3);
        assert!(pairing3.is_none());
    }
    
    #[test]
    fn test_ecef_distance() {
        let p0 = [0.0, 0.0, 0.0];
        let p1 = [3.0, 4.0, 0.0];
        assert!((ecef_distance(&p0, &p1) - 5.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_get_limit() {
        assert_eq!(get_limit(0), 32);
        assert_eq!(get_limit(1), 32);
        assert_eq!(get_limit(2), 16);
        assert_eq!(get_limit(3), 16);
        assert_eq!(get_limit(99), 16);
    }
}
