// Unscented Kalman Filter core functions
// Ported from pykalman/unscented.py
//
// Implements the Unscented Transform and related functions for
// state estimation in nonlinear systems.

use nalgebra as na;
use na::{DMatrix, DVector};

/// Represents mean and covariance of a multivariate normal distribution
#[derive(Debug, Clone)]
pub struct Moments {
    pub mean: DVector<f64>,
    pub covariance: DMatrix<f64>,
}

/// Represents a collection of sigma points and their associated weights
#[derive(Debug, Clone)]
pub struct SigmaPoints {
    /// Sigma points, one per row
    pub points: DMatrix<f64>,
    /// Weights for mean calculation
    pub weights_mean: DVector<f64>,
    /// Weights for covariance calculation
    pub weights_covariance: DVector<f64>,
}

/// Calculate sigma points from mean and covariance (Unscented Transform)
///
/// Ported from pykalman's moments2points function
///
/// # Arguments
/// * `moments` - Mean and covariance of a multivariate normal
/// * `alpha` - Spread of the sigma points (typically 1e-3)
/// * `beta` - Prior knowledge parameter (2 is optimal for Gaussian)
/// * `kappa` - Secondary scaling parameter
///
/// # Returns
/// Sigma points (2*n_dim+1 points) with associated weights
pub fn moments2points(
    moments: &Moments,
    alpha: f64,
    beta: f64,
    kappa: f64,
) -> SigmaPoints {
    let n_dim = moments.mean.len();
    let mu = &moments.mean;
    let sigma = &moments.covariance;
    
    // Calculate scaling parameters
    let lambda = (alpha * alpha) * (n_dim as f64 + kappa) - n_dim as f64;
    let c = n_dim as f64 + lambda;
    
    // Compute sqrt(sigma) using Cholesky decomposition
    let sigma_chol = sigma.clone().cholesky()
        .expect("Covariance matrix must be positive definite")
        .l()
        .transpose();
    
    let scale = c.sqrt();
    
    // Create sigma points:
    // - First point: mu
    // - Next n_dim points: mu + each column of sigma_chol * sqrt(c)
    // - Last n_dim points: mu - each column of sigma_chol * sqrt(c)
    let n_points = 2 * n_dim + 1;
    let mut points = DMatrix::zeros(n_points, n_dim);
    
    // First point is the mean
    points.row_mut(0).copy_from(&mu.transpose());
    
    // Positive sigma points
    for i in 0..n_dim {
        let offset = sigma_chol.column(i) * scale;
        points.row_mut(i + 1).copy_from(&(mu + offset).transpose());
    }
    
    // Negative sigma points
    for i in 0..n_dim {
        let offset = sigma_chol.column(i) * scale;
        points.row_mut(n_dim + i + 1).copy_from(&(mu - offset).transpose());
    }
    
    // Calculate weights
    let mut weights_mean = DVector::from_element(n_points, 0.5 / c);
    weights_mean[0] = lambda / c;
    
    let mut weights_cov = weights_mean.clone();
    weights_cov[0] = lambda / c + (1.0 - alpha * alpha + beta);
    
    SigmaPoints {
        points,
        weights_mean,
        weights_covariance: weights_cov,
    }
}

/// Calculate mean and covariance from sigma points
///
/// Ported from pykalman's points2moments function
///
/// # Arguments
/// * `points` - Sigma points with weights
/// * `sigma_noise` - Optional additive noise covariance
///
/// # Returns
/// Estimated mean and covariance
pub fn points2moments(
    points: &SigmaPoints,
    sigma_noise: Option<&DMatrix<f64>>,
) -> Moments {
    let n_points = points.points.nrows();
    let n_dim = points.points.ncols();
    
    // Calculate weighted mean
    let mut mean = DVector::zeros(n_dim);
    for i in 0..n_points {
        mean += points.points.row(i).transpose() * points.weights_mean[i];
    }
    
    // Calculate weighted covariance
    let mut covariance = DMatrix::zeros(n_dim, n_dim);
    for i in 0..n_points {
        let diff = points.points.row(i).transpose() - &mean;
        covariance += diff.clone() * diff.transpose() * points.weights_covariance[i];
    }
    
    // Add noise covariance if provided
    if let Some(noise) = sigma_noise {
        covariance += noise;
    }
    
    Moments { mean, covariance }
}

/// Apply the Unscented Transform to a set of points
///
/// Ported from pykalman's unscented_transform function
///
/// Apply f to points, then approximate the resulting mean and covariance.
///
/// # Arguments
/// * `points` - Sigma points to transform
/// * `f` - Transition function to apply
/// * `sigma_noise` - Optional additive noise covariance
///
/// # Returns
/// Transformed sigma points and their moments
pub fn unscented_transform<F>(
    points: &SigmaPoints,
    f: F,
    sigma_noise: Option<&DMatrix<f64>>,
) -> (SigmaPoints, Moments)
where
    F: Fn(&DVector<f64>) -> DVector<f64>,
{
    let n_points = points.points.nrows();
    
    // Propagate points through f
    let transformed_points: Vec<DVector<f64>> = (0..n_points)
        .map(|i| {
            let point = points.points.row(i).transpose();
            f(&point)
        })
        .collect();
    
    // Stack transformed points into matrix (one point per row)
    let n_dim_out = transformed_points[0].len();
    let mut points_pred = DMatrix::zeros(n_points, n_dim_out);
    for (i, point) in transformed_points.iter().enumerate() {
        points_pred.row_mut(i).copy_from(&point.transpose());
    }
    
    let points_pred = SigmaPoints {
        points: points_pred,
        weights_mean: points.weights_mean.clone(),
        weights_covariance: points.weights_covariance.clone(),
    };
    
    // Calculate approximate mean and covariance
    let moments_pred = points2moments(&points_pred, sigma_noise);
    
    (points_pred, moments_pred)
}

/// Predict next state distribution using the Unscented Transform
///
/// Ported from pykalman's unscented_filter_predict function
///
/// # Arguments
/// * `transition_function` - Function describing state evolution
/// * `points_state` - Sigma points for current state
/// * `sigma_transition` - Transition noise covariance (additive)
///
/// # Returns
/// Predicted sigma points and moments
pub fn unscented_filter_predict<F>(
    transition_function: F,
    points_state: &SigmaPoints,
    sigma_transition: &DMatrix<f64>,
) -> (SigmaPoints, Moments)
where
    F: Fn(&DVector<f64>) -> DVector<f64>,
{
    unscented_transform(points_state, transition_function, Some(sigma_transition))
}

/// Correct predicted state estimates with an observation
///
/// Ported from pykalman's unscented_correct function
///
/// # Arguments
/// * `cross_sigma` - Cross-covariance between state and observation
/// * `moments_pred` - Predicted state mean and covariance
/// * `obs_moments_pred` - Predicted observation mean and covariance
/// * `observation` - Actual observation
///
/// # Returns
/// Corrected (filtered) state moments
pub fn unscented_correct(
    cross_sigma: &DMatrix<f64>,
    moments_pred: &Moments,
    obs_moments_pred: &Moments,
    observation: &DVector<f64>,
) -> Moments {
    // Calculate Kalman gain
    let obs_cov_inv = obs_moments_pred.covariance.clone()
        .pseudo_inverse(1e-10)
        .expect("Observation covariance must be invertible");
    
    let kalman_gain = cross_sigma * obs_cov_inv;
    
    // Correct mean and covariance
    let innovation = observation - &obs_moments_pred.mean;
    let mean_filt = &moments_pred.mean + &kalman_gain * innovation;
    let covariance_filt = &moments_pred.covariance - &kalman_gain * cross_sigma.transpose();
    
    Moments {
        mean: mean_filt,
        covariance: covariance_filt,
    }
}

/// Integrate new observation to correct state estimates
///
/// Ported from pykalman's unscented_filter_correct function
///
/// # Arguments
/// * `observation_function` - Function mapping state to observation
/// * `moments_pred` - Predicted state moments
/// * `points_pred` - Predicted state sigma points
/// * `observation` - Actual observation
/// * `sigma_observation` - Observation noise covariance (additive)
///
/// # Returns
/// Corrected (filtered) state moments
pub fn unscented_filter_correct<F>(
    observation_function: F,
    moments_pred: &Moments,
    points_pred: &SigmaPoints,
    observation: &DVector<f64>,
    sigma_observation: &DMatrix<f64>,
) -> Moments
where
    F: Fn(&DVector<f64>) -> DVector<f64>,
{
    // Calculate predicted observation mean and covariance
    let (obs_points_pred, obs_moments_pred) = unscented_transform(
        points_pred,
        observation_function,
        Some(sigma_observation),
    );
    
    // Calculate cross-covariance between state and observation
    let n_points = points_pred.points.nrows();
    let n_dim_state = points_pred.points.ncols();
    let n_dim_obs = obs_points_pred.points.ncols();
    
    let mut cross_sigma = DMatrix::zeros(n_dim_state, n_dim_obs);
    for i in 0..n_points {
        let state_diff = points_pred.points.row(i).transpose() - &moments_pred.mean;
        let obs_diff = obs_points_pred.points.row(i).transpose() - &obs_moments_pred.mean;
        cross_sigma += state_diff * obs_diff.transpose() * points_pred.weights_mean[i];
    }
    
    // Correct using Kalman update
    unscented_correct(&cross_sigma, moments_pred, &obs_moments_pred, observation)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_moments2points_basic() {
        // Simple 2D case
        let mean = DVector::from_vec(vec![1.0, 2.0]);
        let cov = DMatrix::from_row_slice(2, 2, &[
            1.0, 0.0,
            0.0, 1.0,
        ]);
        
        let moments = Moments {
            mean,
            covariance: cov,
        };
        
        let points = moments2points(&moments, 1.0, 0.0, 1.0);
        
        // Should have 2*2+1 = 5 points
        assert_eq!(points.points.nrows(), 5);
        assert_eq!(points.points.ncols(), 2);
        
        // First point should be the mean
        assert!((points.points[(0, 0)] - 1.0).abs() < 1e-10);
        assert!((points.points[(0, 1)] - 2.0).abs() < 1e-10);
        
        // Weights should sum to 1
        let weight_sum: f64 = points.weights_mean.iter().sum();
        assert!((weight_sum - 1.0).abs() < 1e-10);
    }
    
    #[test]
    fn test_points2moments_roundtrip() {
        // Create moments, convert to points, convert back
        let mean = DVector::from_vec(vec![1.0, 2.0, 3.0]);
        let cov = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 2.0, 3.0]));
        
        let moments = Moments {
            mean: mean.clone(),
            covariance: cov.clone(),
        };
        
        let points = moments2points(&moments, 1.0, 0.0, 0.0);
        let recovered = points2moments(&points, None);
        
        // Mean should be recovered exactly
        for i in 0..3 {
            assert!((recovered.mean[i] - mean[i]).abs() < 1e-10);
        }
        
        // Covariance should be close (not exact due to numerical precision)
        for i in 0..3 {
            for j in 0..3 {
                assert!((recovered.covariance[(i, j)] - cov[(i, j)]).abs() < 1e-8);
            }
        }
    }
    
    #[test]
    fn test_unscented_transform_linear() {
        // For a linear function, unscented transform should give exact results
        let mean = DVector::from_vec(vec![1.0, 2.0]);
        let cov = DMatrix::from_diagonal(&DVector::from_vec(vec![1.0, 1.0]));
        
        let moments = Moments { mean, covariance: cov };
        let points = moments2points(&moments, 1.0, 0.0, 0.0);
        
        // Linear transformation: y = 2*x
        let f = |x: &DVector<f64>| x * 2.0;
        
        let (_, transformed_moments) = unscented_transform(&points, f, None);
        
        // Mean should be 2 * original mean
        assert!((transformed_moments.mean[0] - 2.0).abs() < 1e-10);
        assert!((transformed_moments.mean[1] - 4.0).abs() < 1e-10);
        
        // Covariance should be 4 * original covariance (for linear scaling)
        assert!((transformed_moments.covariance[(0, 0)] - 4.0).abs() < 1e-8);
        assert!((transformed_moments.covariance[(1, 1)] - 4.0).abs() < 1e-8);
    }
}
