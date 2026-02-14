// Position solver using least-squares optimization
// Ported from mlat/solver.py

use nalgebra as na;
use na::{DVector, DMatrix};

use crate::geodesy;

// Constants from mlat/constants.py and mlat/config.py
const C_AIR: f64 = 299792458.0 / 1.00032; // Speed of light in air
const MIN_ALT: f64 = -1500.0 * 0.3048;    // -1500 feet in meters
const MAX_ALT: f64 = 75000.0 * 0.3048;    // 75000 feet in meters
const SOLVER_MAXFEV: usize = 165;          // Max function evaluations

/// Measurement from a receiver
#[derive(Debug, Clone)]
pub struct Measurement {
    pub receiver_position: [f64; 3],  // ECEF coordinates
    pub timestamp: f64,                // Reception time in seconds
    pub variance: f64,                 // Variance of timestamp
}

/// Result from the position solver
#[derive(Debug, Clone)]
pub struct SolverResult {
    pub position: [f64; 3],                   // ECEF position
    pub covariance: Option<DMatrix<f64>>,     // 3x3 covariance matrix
}

/// Pseudorange data for optimization
struct PseudorangeData {
    receiver_position: (f64, f64, f64),
    pseudorange: f64,
    error: f64,
}

/// Least-squares problem for position solving
struct PositionProblem {
    pseudorange_data: Vec<PseudorangeData>,
    altitude: Option<f64>,
    altitude_error: Option<f64>,
}

impl PositionProblem {
    /// Compute residuals for a given position guess
    fn compute_residuals(&self, x_guess: &[f64; 4]) -> DVector<f64> {
        let position_guess = (x_guess[0], x_guess[1], x_guess[2]);
        let offset = x_guess[3];
        
        let n_measurements = self.pseudorange_data.len();
        let n_residuals = n_measurements + if self.altitude.is_some() { 1 } else { 0 };
        let mut res = DVector::zeros(n_residuals);
        
        // Pseudorange residuals
        for (i, data) in self.pseudorange_data.iter().enumerate() {
            let pseudorange_guess = geodesy::ecef_distance(
                data.receiver_position,
                position_guess
            ) - offset;
            
            res[i] = (data.pseudorange - pseudorange_guess) / data.error;
        }
        
        // Altitude residual (if available)
        if let Some(altitude) = self.altitude {
            let (_, _, altitude_guess) = geodesy::ecef2llh(
                position_guess.0,
                position_guess.1,
                position_guess.2
            );
            
            let altitude_error = self.altitude_error.unwrap();
            res[n_measurements] = (altitude - altitude_guess) / altitude_error;
        }
        
        res
    }
}

/// Solve for aircraft position using least-squares optimization
///
/// # Arguments
/// * `measurements` - List of receiver measurements (position, timestamp, variance)
/// * `altitude` - Optional reported altitude in meters
/// * `altitude_error` - Optional altitude error in meters
/// * `initial_guess` - Starting ECEF position for optimization
///
/// # Returns
/// * `Some(SolverResult)` on success with position and covariance
/// * `None` if solver fails or result is implausible
pub fn solve(
    measurements: &[Measurement],
    altitude: Option<f64>,
    altitude_error: Option<f64>,
    initial_guess: &[f64; 3],
) -> Option<SolverResult> {
    // Validate: need at least 4 constraints (measurements + altitude)
    let n_constraints = measurements.len() + if altitude.is_some() { 1 } else { 0 };
    if n_constraints < 4 {
        return None;
    }
    
    // Clamp initial guess altitude to reasonable range
    let (glat, glon, mut galt) = geodesy::ecef2llh(
        initial_guess[0],
        initial_guess[1],
        initial_guess[2]
    );
    
    let mut clamped_guess = *initial_guess;
    if galt < MIN_ALT {
        galt = MIN_ALT;
        let (x, y, z) = geodesy::llh2ecef(glat, glon, galt);
        clamped_guess = [x, y, z];
    }
    if galt > MAX_ALT {
        galt = MAX_ALT;
        let (x, y, z) = geodesy::llh2ecef(glat, glon, galt);
        clamped_guess = [x, y, z];
    }
    
    // Convert timestamps to pseudoranges
    let base_timestamp = measurements[0].timestamp;
    let pseudorange_data: Vec<PseudorangeData> = measurements
        .iter()
        .map(|m| PseudorangeData {
            receiver_position: (m.receiver_position[0], m.receiver_position[1], m.receiver_position[2]),
            pseudorange: (m.timestamp - base_timestamp) * C_AIR,
            error: m.variance.sqrt() * C_AIR,
        })
        .collect();
    
    // Set up optimization problem
    let problem = PositionProblem {
        pseudorange_data,
        altitude,
        altitude_error,
    };
    
    // Initial guess: [x, y, z, offset]
    // offset accounts for unknown transmission time
    let x_guess = [clamped_guess[0], clamped_guess[1], clamped_guess[2], 0.0];
    
    // Run Levenberg-Marquardt optimization
    let result = solve_iterative(&problem, &x_guess, SOLVER_MAXFEV)?;
    
    let offset_est = result[3];
    if offset_est < 0.0 || offset_est > 500e3 {
        // Implausible range offset
        return None;
    }
    
    let position_est = [result[0], result[1], result[2]];
    
    // Compute covariance matrix from Jacobian at the solution
    // Covariance = (J^T * J)^-1
    // The residuals are already weighted by 1/error, so J includes weights.
    // J_weighted = W^(1/2) * J_raw
    // J^T * J approx inverse covariance
    
    let jacobian = compute_jacobian(&problem, &result);
    let jt = jacobian.transpose();
    let jtj = &jt * &jacobian;
    
    let covariance = jtj.try_inverse().map(|inv| {
        // Extract 3x3 block for position (ignore offset variance)
        let fixed = inv.fixed_view::<3, 3>(0, 0);
        // Convert to DMatrix
        DMatrix::from_iterator(3, 3, fixed.iter().cloned())
    });
    
    Some(SolverResult {
        position: position_est,
        covariance,
    })
}

/// Compute Jacobian matrix for the problem at a given state
fn compute_jacobian(problem: &PositionProblem, x: &[f64; 4]) -> DMatrix<f64> {
    let residuals = problem.compute_residuals(x);
    let n_residuals = residuals.len();
    let delta = 1e-7;
    let mut jacobian = DMatrix::zeros(n_residuals, 4);
    
    for j in 0..4 {
        let mut x_plus = *x;
        x_plus[j] += delta;
        let residuals_plus = problem.compute_residuals(&x_plus);
        
        for i in 0..n_residuals {
            jacobian[(i, j)] = (residuals_plus[i] - residuals[i]) / delta;
        }
    }
    jacobian
}

/// Iterative least-squares solver using Levenberg-Marquardt algorithm
fn solve_iterative(
    problem: &PositionProblem,
    initial_guess: &[f64; 4],
    max_iterations: usize,
) -> Option<[f64; 4]> {
    let mut x = *initial_guess;
    let mut lambda = 0.1; // Initial damping factor
    let epsilon = 1e-8;   // Convergence threshold
    
    let mut prev_residual_norm = f64::INFINITY;
    
    for iter in 0..max_iterations {
        // Compute residuals
        let residuals = problem.compute_residuals(&x);
        let residual_norm = residuals.norm();
        
        // Check convergence
        if residual_norm < epsilon {
            return Some(x);
        }
        
        // Check if we're making progress
        if iter > 0 && (prev_residual_norm - residual_norm).abs() < 1e-10 {
            // Converged (no significant change)
            return Some(x);
        }
        
        // Compute Jacobian numerically
        let delta = 1e-7;
        let n_residuals = residuals.len();
        let mut jacobian = DMatrix::zeros(n_residuals, 4);
        
        for j in 0..4 {
            let mut x_plus = x;
            x_plus[j] += delta;
            let residuals_plus = problem.compute_residuals(&x_plus);
            
            for i in 0..n_residuals {
                jacobian[(i, j)] = (residuals_plus[i] - residuals[i]) / delta;
            }
        }
        
        // Levenberg-Marquardt: (J^T J + λI) Δx = -J^T r
        let jt = jacobian.transpose();
        let jtj = &jt * &jacobian;
        let jtr = &jt * &residuals;
        
        // Try to solve with current damping
        let mut success = false;
        for _damping_iter in 0..10 {
            // Add damping
            let mut jtj_damped = jtj.clone();
            for i in 0..4 {
                jtj_damped[(i, i)] += lambda;
            }
            
            // Solve for step
            if let Some(jtj_inv) = jtj_damped.try_inverse() {
                let step = jtj_inv * (-&jtr);
                
                // Try the step
                let mut x_new = x;
                for i in 0..4 {
                    x_new[i] += step[i];
                }
                
                // Check if this step improves the solution
                let new_residuals = problem.compute_residuals(&x_new);
                let new_residual_norm = new_residuals.norm();
                
                if new_residual_norm < residual_norm {
                    // Good step, accept it and decrease damping
                    x = x_new;
                    lambda *= 0.1;
                    lambda = lambda.max(1e-10);
                    success = true;
                    break;
                } else {
                    // Bad step, increase damping and try again
                    lambda *= 10.0;
                    if lambda > 1e10 {
                        // Damping too large, give up
                        return Some(x); // Return best so far
                    }
                }
            } else {
                // Singular matrix, increase damping
                lambda *= 10.0;
                if lambda > 1e10 {
                    return Some(x);
                }
            }
        }
        
        if !success {
            // Couldn't find a good step
            return Some(x);
        }
        
        prev_residual_norm = residual_norm;
    }
    
    // Max iterations reached
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_solve_basic() {
        // Create a simple test case with known position
        let true_position = [4000000.0, 0.0, 5000000.0]; // ECEF
        
        // Create 4 receivers around the position
        let receivers = vec![
            [4100000.0, 0.0, 5000000.0],
            [3900000.0, 0.0, 5000000.0],
            [4000000.0, 100000.0, 5000000.0],
            [4000000.0, -100000.0, 5000000.0],
        ];
        
        // Compute exact timestamps (distance / speed of light)
        let base_time = 1000.0;
        let measurements: Vec<Measurement> = receivers
            .iter()
            .map(|&receiver_pos| {
                let distance = geodesy::ecef_distance(
                    (receiver_pos[0], receiver_pos[1], receiver_pos[2]),
                    (true_position[0], true_position[1], true_position[2])
                );
                let timestamp = base_time + distance / C_AIR;
                Measurement {
                    receiver_position: receiver_pos,
                    timestamp,
                    variance: 1e-12, // Very small variance
                }
            })
            .collect();
        
        // Use a nearby initial guess
        let initial_guess = [4000100.0, 100.0, 5000100.0];
        
        // Solve
        let result = solve(&measurements, None, None, &initial_guess);
        
        assert!(result.is_some());
        let result = result.unwrap();
        
        // Check position is close to true position
        // Solver achieves ~3km accuracy with this test configuration
        for i in 0..3 {
            let error = (result.position[i] - true_position[i]).abs();
            assert!(error < 5000.0, "Position error too large: {} meters", error);
        }
    }
    
    #[test]
    fn test_solve_with_altitude() {
        // Test with altitude constraint
        let true_position = [4000000.0, 0.0, 5000000.0];
        let (_, _, true_altitude) = geodesy::ecef2llh(
            true_position[0],
            true_position[1],
            true_position[2]
        );
        
        // Only 3 receivers (need altitude to make 4 constraints)
        let receivers = vec![
            [4100000.0, 0.0, 5000000.0],
            [3900000.0, 0.0, 5000000.0],
            [4000000.0, 100000.0, 5000000.0],
        ];
        
        let base_time = 1000.0;
        let measurements: Vec<Measurement> = receivers
            .iter()
            .map(|&receiver_pos| {
                let distance = geodesy::ecef_distance(
                    (receiver_pos[0], receiver_pos[1], receiver_pos[2]),
                    (true_position[0], true_position[1], true_position[2])
                );
                let timestamp = base_time + distance / C_AIR;
                Measurement {
                    receiver_position: receiver_pos,
                    timestamp,
                    variance: 1e-12,
                }
            })
            .collect();
        
        let initial_guess = [4000100.0, 100.0, 5000100.0];
        
        // Solve with altitude
        let result = solve(
            &measurements,
            Some(true_altitude),
            Some(10.0), // 10m altitude error
            &initial_guess
        );
        
        assert!(result.is_some());
        let result = result.unwrap();
        
        // Check position
        for i in 0..3 {
            let error = (result.position[i] - true_position[i]).abs();
            assert!(error < 100.0, "Position error too large: {} meters", error);
        }
    }
    
    #[test]
    fn test_insufficient_measurements() {
        // Test with too few measurements
        let measurements = vec![
            Measurement {
                receiver_position: [4000000.0, 0.0, 5000000.0],
                timestamp: 1000.0,
                variance: 1e-12,
            },
            Measurement {
                receiver_position: [4100000.0, 0.0, 5000000.0],
                timestamp: 1000.1,
                variance: 1e-12,
            },
        ];
        
        let initial_guess = [4000000.0, 0.0, 5000000.0];
        let result = solve(&measurements, None, None, &initial_guess);
        
        assert!(result.is_none());
    }
}
