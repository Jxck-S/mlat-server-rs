// Kalman filter state for aircraft tracking
// Ported from mlat/kalman.py

use nalgebra as na;
use na::{DMatrix, DVector};
use std::f64::consts::PI;

use crate::geodesy;
use super::unscented::{Moments, moments2points, unscented_filter_predict, unscented_filter_correct};

// Constants from mlat/constants.py
const DTOR: f64 = PI / 180.0;
const C_AIR: f64 = 299792458.0 / 1.00029; // Speed of light in air

/// Kalman filter state for a single aircraft
///
/// Ported from Python's KalmanStateCV (Constant Velocity model)
#[derive(Debug, Clone)]
pub struct KalmanState {
    // Aircraft identifier
    pub icao: u32,
    
    // Filter state
    mean: Option<DVector<f64>>,  // State vector: [x, y, z, vx, vy, vz]
    cov: Option<DMatrix<f64>>,   // Covariance matrix (6x6)
    acquiring: bool,              // True if still acquiring initial lock
    outliers: usize,              // Count of consecutive outliers
    pub last_update: f64,         // Time of last update (UTC seconds)
    
    // Filter validity
    pub valid: bool,
    
    // Derived values (ECEF frame)
    pub position: Option<[f64; 3]>,        // ECEF position [x, y, z]
    pub velocity: Option<[f64; 3]>,        // ECEF velocity [vx, vy, vz]
    pub position_error: Option<f64>,       // Position error in meters
    pub velocity_error: Option<f64>,       // Velocity error in m/s
    
    // Derived values (geographic frame)
    pub position_llh: Option<[f64; 3]>,    // [lat, lon, alt]
    pub velocity_enu: Option<[f64; 3]>,    // [east, north, up]
    pub heading: Option<f64>,              // Heading in degrees
    pub ground_speed: Option<f64>,         // Ground speed in m/s
    pub vertical_speed: Option<f64>,       // Vertical speed in m/s
    
    // Configuration parameters
    min_acquiring_dof: usize,
    min_tracking_dof: usize,
    outlier_mahalanobis_distance: f64,
    min_acquiring_position_error: f64,
    min_acquiring_velocity_error: f64,
    max_tracking_position_error: f64,
    max_tracking_velocity_error: f64,
    process_noise: f64,
}

impl KalmanState {
    /// Create a new Kalman filter state for an aircraft
    pub fn new(icao: u32) -> Self {
        Self {
            icao,
            mean: None,
            cov: None,
            acquiring: true,
            outliers: 0,
            last_update: 0.0,
            valid: false,
            position: None,
            velocity: None,
            position_error: None,
            velocity_error: None,
            position_llh: None,
            velocity_enu: None,
            heading: None,
            ground_speed: None,
            vertical_speed: None,
            // Default configuration from Python
            min_acquiring_dof: 1,
            min_tracking_dof: 0,
            outlier_mahalanobis_distance: 15.0,
            min_acquiring_position_error: 3e3,
            min_acquiring_velocity_error: 50.0,
            max_tracking_position_error: 5e3,
            max_tracking_velocity_error: 75.0,
            process_noise: 0.10,
        }
    }
    
    /// Reset the filter state
    fn reset(&mut self) {
        self.mean = None;
        self.cov = None;
        self.acquiring = true;
        self.outliers = 0;
        self.last_update = 0.0;
        self.valid = false;
        self.position = None;
        self.velocity = None;
        self.position_error = None;
        self.velocity_error = None;
        self.position_llh = None;
        self.velocity_enu = None;
        self.heading = None;
        self.ground_speed = None;
        self.vertical_speed = None;
    }
    
    /// Set initial state from least-squares position estimate
    ///
    /// State is: [x, y, z, vx, vy, vz] (Constant Velocity model)
    fn set_initial_state(&mut self, leastsquares_position: &[f64; 3], leastsquares_cov: &DMatrix<f64>) {
        // Initialize mean: position from least squares, velocity = 0
        self.mean = Some(DVector::from_vec(vec![
            leastsquares_position[0],
            leastsquares_position[1],
            leastsquares_position[2],
            0.0, 0.0, 0.0,
        ]));
        
        // Initialize covariance
        let mut cov = DMatrix::zeros(6, 6);
        // Position covariance from least squares (scaled by 4)
        for i in 0..3 {
            for j in 0..3 {
                cov[(i, j)] = leastsquares_cov[(i, j)] * 4.0;
            }
        }
        // Velocity covariance (large initial uncertainty)
        cov[(3, 3)] = 200.0 * 200.0;
        cov[(4, 4)] = 200.0 * 200.0;
        cov[(5, 5)] = 200.0 * 200.0;
        
        self.cov = Some(cov);
    }
    
    /// Transition function for constant-velocity model
    ///
    /// x' = x + vx*dt
    /// vx' = vx (constant velocity)
    fn transition_function(&self, state: &DVector<f64>, dt: f64) -> DVector<f64> {
        let x = state[0];
        let y = state[1];
        let z = state[2];
        let vx = state[3];
        let vy = state[4];
        let vz = state[5];
        
        DVector::from_vec(vec![
            x + vx * dt,
            y + vy * dt,
            z + vz * dt,
            vx,
            vy,
            vz,
        ])
    }
    
    /// Transition covariance for constant-velocity model
    ///
    /// Models process noise (acceleration uncertainty)
    fn transition_covariance(&self, dt: f64) -> DMatrix<f64> {
        let mut trans_covar = DMatrix::zeros(6, 6);
        
        // Position-position covariance
        trans_covar[(0, 0)] = 0.25 * dt.powi(4);
        trans_covar[(1, 1)] = 0.25 * dt.powi(4);
        trans_covar[(2, 2)] = 0.25 * dt.powi(4);
        
        // Velocity-velocity covariance
        trans_covar[(3, 3)] = dt.powi(2);
        trans_covar[(4, 4)] = dt.powi(2);
        trans_covar[(5, 5)] = dt.powi(2);
        
        // Position-velocity cross-covariance
        trans_covar[(0, 3)] = 0.5 * dt.powi(3);
        trans_covar[(3, 0)] = 0.5 * dt.powi(3);
        trans_covar[(1, 4)] = 0.5 * dt.powi(3);
        trans_covar[(4, 1)] = 0.5 * dt.powi(3);
        trans_covar[(2, 5)] = 0.5 * dt.powi(3);
        trans_covar[(5, 2)] = 0.5 * dt.powi(3);
        
        // Scale by process noise (white noise, so scale by dt not dt^2)
        trans_covar * self.process_noise.powi(2) * dt
    }
    
    /// Observation function without altitude
    ///
    /// Returns N-1 pseudorange differences relative to first receiver
    #[allow(dead_code)]
    fn observation_function_without_altitude(
        &self,
        state: &DVector<f64>,
        positions: &[[f64; 3]],
    ) -> DVector<f64> {
        let x = state[0];
        let y = state[1];
        let z = state[2];
        
        let n = positions.len();
        let mut obs = DVector::zeros(n - 1);
        
        // Distance to first receiver
        let rx0 = positions[0][0];
        let ry0 = positions[0][1];
        let rz0 = positions[0][2];
        let zero_range = ((rx0 - x).powi(2) + (ry0 - y).powi(2) + (rz0 - z).powi(2)).sqrt();
        
        // Pseudorange differences
        for i in 1..n {
            let rx = positions[i][0];
            let ry = positions[i][1];
            let rz = positions[i][2];
            let range = ((rx - x).powi(2) + (ry - y).powi(2) + (rz - z).powi(2)).sqrt();
            obs[i - 1] = range - zero_range;
        }
        
        obs
    }
    
    /// Observation function with altitude
    ///
    /// Returns altitude + N-1 pseudorange differences
    #[allow(dead_code)]
    fn observation_function_with_altitude(
        &self,
        state: &DVector<f64>,
        positions: &[[f64; 3]],
    ) -> DVector<f64> {
        let x = state[0];
        let y = state[1];
        let z = state[2];
        
        let n = positions.len();
        let mut obs = DVector::zeros(n);
        
        // First observation is altitude
        let (_, _, alt) = geodesy::ecef2llh(x, y, z);
        obs[0] = alt;
        
        // Distance to first receiver
        let rx0 = positions[0][0];
        let ry0 = positions[0][1];
        let rz0 = positions[0][2];
        let zero_range = ((rx0 - x).powi(2) + (ry0 - y).powi(2) + (rz0 - z).powi(2)).sqrt();
        
        // Pseudorange differences
        for i in 1..n {
            let rx = positions[i][0];
            let ry = positions[i][1];
            let rz = positions[i][2];
            let range = ((rx - x).powi(2) + (ry - y).powi(2) + (rz - z).powi(2)).sqrt();
            obs[i] = range - zero_range;
        }
        
        obs
    }
    
    /// Update derived values from filter state
    fn update_derived(&mut self) {
        let mean = self.mean.as_ref().unwrap();
        let cov = self.cov.as_ref().unwrap();
        
        // Extract position and velocity
        self.position = Some([mean[0], mean[1], mean[2]]);
        self.velocity = Some([mean[3], mean[4], mean[5]]);
        
        // Calculate position error (sqrt of trace of position covariance)
        let pe = cov[(0, 0)] + cov[(1, 1)] + cov[(2, 2)];
        self.position_error = Some(if pe < 0.0 { 1e6 } else { pe.sqrt() });
        
        // Calculate velocity error (sqrt of trace of velocity covariance)
        let ve = cov[(3, 3)] + cov[(4, 4)] + cov[(5, 5)];
        self.velocity_error = Some(if ve < 0.0 { 1e6 } else { ve.sqrt() });
        
        // Convert position to LLH
        let (lat, lon, alt) = geodesy::ecef2llh(mean[0], mean[1], mean[2]);
        self.position_llh = Some([lat, lon, alt]);
        
        // Rotate velocity into local tangent plane (ENU frame)
        let lat_r = lat * DTOR;
        let lon_r = lon * DTOR;
        
        // Rotation matrix from ECEF to ENU
        let c_lat = lat_r.cos();
        let s_lat = lat_r.sin();
        let c_lon = lon_r.cos();
        let s_lon = lon_r.sin();
        
        let east = -s_lon * mean[3] + c_lon * mean[4];
        let north = -s_lat * c_lon * mean[3] - s_lat * s_lon * mean[4] + c_lat * mean[5];
        let up = c_lat * c_lon * mean[3] + c_lat * s_lon * mean[4] + s_lat * mean[5];
        
        self.velocity_enu = Some([east, north, up]);
        
        // Calculate heading and speeds
        let mut heading = east.atan2(north) * 180.0 / PI;
        if heading < 0.0 {
            heading += 360.0;
        }
        self.heading = Some(heading);
        self.ground_speed = Some((north.powi(2) + east.powi(2)).sqrt());
        self.vertical_speed = Some(up);
        
        self.valid = true;
    }
    
    /// Update the filter with new measurements
    ///
    /// # Arguments
    /// * `position_time` - Time of measurements (UTC seconds)
    /// * `measurements` - List of (receiver_position, timestamp, variance) tuples
    /// * `altitude` - Reported altitude in meters (or None)
    /// * `altitude_error` - Altitude error in meters (or None)
    /// * `leastsquares_position` - ECEF position from least-squares solver
    /// * `leastsquares_cov` - Covariance of least-squares position
    /// * `dof` - Degrees of freedom in the solution
    ///
    /// # Returns
    /// True if filter was updated and is valid
    pub fn update(
        &mut self,
        position_time: f64,
        measurements: &[([f64; 3], f64, f64)],  // (position, timestamp, variance)
        altitude: Option<f64>,
        altitude_error: Option<f64>,
        leastsquares_position: &[f64; 3],
        leastsquares_cov: &DMatrix<f64>,
        dof: usize,
    ) -> bool {
        let dt = position_time - self.last_update;
        
        // Check for time going backwards
        if dt < 0.0 {
            return false;
        }
        
        // Reset if too much time has passed
        if dt > 300.0 {
            self.reset();
        }
        
        // Don't update if not enough DOF while acquiring
        if self.acquiring && dof < self.min_acquiring_dof {
            return false;
        }
        
        // Initialize filter if needed
        if self.mean.is_none() {
            self.last_update = position_time;
            self.set_initial_state(leastsquares_position, leastsquares_cov);
            return false;
        }
        
        // Don't update if not enough DOF while tracking
        if dof < self.min_tracking_dof {
            return false;
        }
        
        // Prepare observations
        let zero_pr = measurements[0].1 * C_AIR;
        let positions: Vec<[f64; 3]> = measurements.iter().map(|(pos, _, _)| *pos).collect();
        
        let n = measurements.len();
        let (obs, obs_var) = if altitude.is_none() {
            // Without altitude
            let mut obs = DVector::zeros(n - 1);
            let mut obs_var = DVector::zeros(n - 1);
            
            for i in 1..n {
                let (_, timestamp, variance) = measurements[i];
                obs[i - 1] = timestamp * C_AIR - zero_pr;
                obs_var[i - 1] = (variance + measurements[0].2) * C_AIR.powi(2);
            }
            
            (obs, obs_var)
        } else {
            // With altitude
            let mut obs = DVector::zeros(n);
            let mut obs_var = DVector::zeros(n);
            
            obs[0] = altitude.unwrap();
            obs_var[0] = altitude_error.unwrap().powi(2);
            
            for i in 1..n {
                let (_, timestamp, variance) = measurements[i];
                obs[i] = timestamp * C_AIR - zero_pr;
                obs_var[i] = (variance + measurements[0].2) * C_AIR.powi(2);
            }
            
            (obs, obs_var)
        };
        
        let obs_covar = DMatrix::from_diagonal(&obs_var);
        
        // Run unscented Kalman filter update
        let result = self.ukf_update(dt, &obs, &obs_covar, &positions, altitude.is_some(), position_time);
        
        result
    }
    
    /// Unscented Kalman Filter update step
    fn ukf_update(
        &mut self,
        dt: f64,
        obs: &DVector<f64>,
        obs_covar: &DMatrix<f64>,
        positions: &[[f64; 3]],
        has_altitude: bool,
        position_time: f64,
    ) -> bool {
        let mean = self.mean.clone().unwrap();
        let cov = self.cov.clone().unwrap();
        
        // Generate sigma points
        let moments_state = Moments {
            mean,
            covariance: cov,
        };
        let points_state = moments2points(&moments_state, 1.0, 0.0, 0.0);
        
        // Predict step
        let trans_covar = self.transition_covariance(dt);
        let transition_fn = |state: &DVector<f64>| self.transition_function(state, dt);
        
        let (points_pred, moments_pred) = unscented_filter_predict(
            transition_fn,
            &points_state,
            &trans_covar,
        );
        
        // Create observation function (standalone to avoid borrow issues)
        let positions_vec = positions.to_vec();
        let obs_fn_with_alt = |state: &DVector<f64>| {
            observation_with_altitude(state, &positions_vec)
        };
        let obs_fn_without_alt = |state: &DVector<f64>| {
            observation_without_altitude(state, &positions_vec)
        };
        
        // Correct step with outlier detection
        let (_, obs_moments_pred) = if has_altitude {
            super::unscented::unscented_transform(
                &points_pred,
                obs_fn_with_alt,
                Some(obs_covar),
            )
        } else {
            super::unscented::unscented_transform(
                &points_pred,
                obs_fn_without_alt,
                Some(obs_covar),
            )
        };
        
        // Calculate Mahalanobis distance for outlier detection
        let innovation = obs - &obs_moments_pred.mean;
        let obs_cov_inv = obs_moments_pred.covariance.clone()
            .pseudo_inverse(1e-10)
            .unwrap_or_else(|_| obs_moments_pred.covariance.clone());
        
        let md = (innovation.transpose() * &obs_cov_inv * &innovation)[(0, 0)].sqrt();
        
        // Check for outliers
        if md > self.outlier_mahalanobis_distance {
            self.outliers += 1;
            if self.outliers < 3 || (position_time - self.last_update) < 15.0 {
                return false;
            }
            self.reset();
            return false;
        }
        
        self.outliers = 0;
        
        // Apply correction
        let moments_filt = if has_altitude {
            unscented_filter_correct(
                obs_fn_with_alt,
                &moments_pred,
                &points_pred,
                obs,
                obs_covar,
            )
        } else {
            unscented_filter_correct(
                obs_fn_without_alt,
                &moments_pred,
                &points_pred,
                obs,
                obs_covar,
            )
        };
        
        self.mean = Some(moments_filt.mean);
        self.cov = Some(moments_filt.covariance);
        self.last_update = position_time;
        
        // Update derived values
        self.update_derived();
        
        // Check if we've converged enough to start reporting
        let pos_err = self.position_error.unwrap();
        let vel_err = self.velocity_error.unwrap();
        
        if self.acquiring && pos_err < self.min_acquiring_position_error && vel_err < self.min_acquiring_velocity_error {
            self.acquiring = false;
        } else if !self.acquiring && (pos_err > self.max_tracking_position_error || vel_err > self.max_tracking_velocity_error) {
            self.acquiring = true;
        }
        
        self.valid = !self.acquiring;
        self.valid
    }
}

// Standalone observation functions (to avoid borrow checker issues)

/// Observation function without altitude (standalone)
fn observation_without_altitude(
    state: &DVector<f64>,
    positions: &[[f64; 3]],
) -> DVector<f64> {
    let x = state[0];
    let y = state[1];
    let z = state[2];
    
    let n = positions.len();
    let mut obs = DVector::zeros(n - 1);
    
    // Distance to first receiver
    let rx0 = positions[0][0];
    let ry0 = positions[0][1];
    let rz0 = positions[0][2];
    let zero_range = ((rx0 - x).powi(2) + (ry0 - y).powi(2) + (rz0 - z).powi(2)).sqrt();
    
    // Pseudorange differences
    for i in 1..n {
        let rx = positions[i][0];
        let ry = positions[i][1];
        let rz = positions[i][2];
        let range = ((rx - x).powi(2) + (ry - y).powi(2) + (rz - z).powi(2)).sqrt();
        obs[i - 1] = range - zero_range;
    }
    
    obs
}

/// Observation function with altitude (standalone)
fn observation_with_altitude(
    state: &DVector<f64>,
    positions: &[[f64; 3]],
) -> DVector<f64> {
    let x = state[0];
    let y = state[1];
    let z = state[2];
    
    let n = positions.len();
    let mut obs = DVector::zeros(n);
    
    // First observation is altitude
    let (_, _, alt) = geodesy::ecef2llh(x, y, z);
    obs[0] = alt;
    
    // Distance to first receiver
    let rx0 = positions[0][0];
    let ry0 = positions[0][1];
    let rz0 = positions[0][2];
    let zero_range = ((rx0 - x).powi(2) + (ry0 - y).powi(2) + (rz0 - z).powi(2)).sqrt();
    
    // Pseudorange differences
    for i in 1..n {
        let rx = positions[i][0];
        let ry = positions[i][1];
        let rz = positions[i][2];
        let range = ((rx - x).powi(2) + (ry - y).powi(2) + (rz - z).powi(2)).sqrt();
        obs[i] = range - zero_range;
    }
    
    obs
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_kalman_state_creation() {
        let state = KalmanState::new(0xABCDEF);
        assert_eq!(state.icao, 0xABCDEF);
        assert!(!state.valid);
        assert!(state.mean.is_none());
    }
    
    #[test]
    fn test_transition_function() {
        let state = KalmanState::new(0x123456);
        let s = DVector::from_vec(vec![1000.0, 2000.0, 3000.0, 100.0, 200.0, 300.0]);
        let dt = 1.0;
        
        let s_next = state.transition_function(&s, dt);
        
        // Position should be updated by velocity * dt
        assert!((s_next[0] - 1100.0).abs() < 1e-10);
        assert!((s_next[1] - 2200.0).abs() < 1e-10);
        assert!((s_next[2] - 3300.0).abs() < 1e-10);
        
        // Velocity should remain constant
        assert!((s_next[3] - 100.0).abs() < 1e-10);
        assert!((s_next[4] - 200.0).abs() < 1e-10);
        assert!((s_next[5] - 300.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_transition_covariance() {
        let state = KalmanState::new(0x123456);
        let dt = 1.0;
        
        let cov = state.transition_covariance(dt);
        
        // Should be 6x6
        assert_eq!(cov.nrows(), 6);
        assert_eq!(cov.ncols(), 6);
        
        // Should be symmetric
        for i in 0..6 {
            for j in 0..6 {
                assert!((cov[(i, j)] - cov[(j, i)]).abs() < 1e-10);
            }
        }
    }
}
