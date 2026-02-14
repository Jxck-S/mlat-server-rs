// Geodesy module - coordinate transformations and distance calculations
// Ported from mlat/geodesy.pyx
//
// Provides conversions between:
// - LLH (Latitude/Longitude/Height) in degrees and meters
// - ECEF (Earth-Centered Earth-Fixed) in meters
//
// Uses WGS84 ellipsoid model for Earth

use std::f64::consts::PI;

/// Degrees to radians conversion factor
const DTOR: f64 = PI / 180.0;

/// Radians to degrees conversion factor
const RTOD: f64 = 180.0 / PI;

/// WGS84 ellipsoid semi-major axis (equatorial radius) in meters
const WGS84_A: f64 = 6378137.0;

/// WGS84 ellipsoid flattening factor
const WGS84_F: f64 = 1.0 / 298.257223563;

/// WGS84 ellipsoid semi-minor axis (polar radius) in meters
const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);

/// WGS84 ellipsoid eccentricity squared
const WGS84_ECC_SQ: f64 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);

/// Average radius for spherical Earth approximation in meters
const SPHERICAL_R: f64 = 6371e3;

// Helper function to compute derived constants at compile time where possible
// For constants requiring sqrt, we compute at first use
#[inline]
fn wgs84_ep() -> f64 {
    let a_sq = WGS84_A * WGS84_A;
    let b_sq = WGS84_B * WGS84_B;
    ((a_sq - b_sq) / b_sq).sqrt()
}

#[inline]
fn wgs84_ep2_b() -> f64 {
    let ep = wgs84_ep();
    ep * ep * WGS84_B
}

#[inline]
fn wgs84_e2_a() -> f64 {
    WGS84_ECC_SQ * WGS84_A
}

/// Converts from WGS84 lat/lon/height to ellipsoid-earth ECEF coordinates
///
/// # Arguments
/// * `lat` - Latitude in degrees
/// * `lon` - Longitude in degrees
/// * `alt` - Altitude in meters above WGS84 ellipsoid
///
/// # Returns
/// ECEF coordinates (x, y, z) in meters
///
/// # Example
/// ```
/// let (x, y, z) = llh2ecef(51.5, -0.1, 100.0);
/// ```
pub fn llh2ecef(lat: f64, lon: f64, alt: f64) -> (f64, f64, f64) {
    // Convert to radians
    let lat_rad = lat * DTOR;
    let lon_rad = lon * DTOR;
    
    // Precompute trig functions
    let slat = lat_rad.sin();
    let slon = lon_rad.sin();
    let clat = lat_rad.cos();
    let clon = lon_rad.cos();
    
    // Radius of curvature in prime vertical
    let d = (1.0 - (slat * slat * WGS84_ECC_SQ)).sqrt();
    let rn = WGS84_A / d;
    
    // ECEF coordinates
    let x = (rn + alt) * clat * clon;
    let y = (rn + alt) * clat * slon;
    let z = (rn * (1.0 - WGS84_ECC_SQ) + alt) * slat;
    
    (x, y, z)
}

/// Converts from ECEF to WGS84 lat/lon/height
///
/// Uses iterative method for accurate conversion
///
/// # Arguments
/// * `x` - ECEF X coordinate in meters
/// * `y` - ECEF Y coordinate in meters
/// * `z` - ECEF Z coordinate in meters
///
/// # Returns
/// (latitude, longitude, altitude) where lat/lon are in degrees and altitude in meters
///
/// # Example
/// ```
/// let (lat, lon, alt) = ecef2llh(4000000.0, 0.0, 5000000.0);
/// ```
pub fn ecef2llh(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    // Longitude is straightforward
    let lon = y.atan2(x);
    
    // Latitude requires iterative calculation
    let p = (x * x + y * y).sqrt();
    let th = (WGS84_A * z).atan2(WGS84_B * p);
    
    let sin_th = th.sin();
    let cos_th = th.cos();
    
    let ep2_b = wgs84_ep2_b();
    let e2_a = wgs84_e2_a();
    
    let lat = (z + ep2_b * sin_th * sin_th * sin_th)
        .atan2(p - e2_a * cos_th * cos_th * cos_th);
    
    // Calculate altitude
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = WGS84_A / (1.0 - WGS84_ECC_SQ * sin_lat * sin_lat).sqrt();
    let alt = p / cos_lat - n;
    
    (lat * RTOD, lon * RTOD, alt)
}

/// Returns great-circle distance in meters between two LLH points
///
/// **Assumes spherical Earth and ignores altitude**. Accuracy is ~1% for most purposes.
/// For higher accuracy, convert to ECEF and use ecef_distance.
///
/// # Arguments
/// * `lat0`, `lon0` - First point (latitude, longitude) in degrees
/// * `lat1`, `lon1` - Second point (latitude, longitude) in degrees
///
/// # Returns
/// Distance in meters
///
/// # Example
/// ```
/// let distance = greatcircle(51.5, -0.1, 48.8, 2.3); // London to Paris
/// ```
pub fn greatcircle(lat0: f64, lon0: f64, lat1: f64, lon1: f64) -> f64 {
    let lat0_rad = lat0 * DTOR;
    let lon0_rad = lon0 * DTOR;
    let lat1_rad = lat1 * DTOR;
    let lon1_rad = lon1 * DTOR;
    
    SPHERICAL_R * (
        lat0_rad.sin() * lat1_rad.sin() +
        lat0_rad.cos() * lat1_rad.cos() * (lon0_rad - lon1_rad).abs().cos()
    ).acos()
}

/// Returns straight-line (Euclidean) distance in meters between two ECEF points
///
/// This is much faster than scipy or numpy-based approaches (10-20x speedup in Python).
/// In Rust, it's just a simple calculation with no overhead.
///
/// # Arguments
/// * `p0` - First point (x, y, z) in meters
/// * `p1` - Second point (x, y, z) in meters
///
/// # Returns
/// Distance in meters
///
/// # Example
/// ```
/// let dist = ecef_distance((4000000.0, 0.0, 5000000.0), (4100000.0, 0.0, 5000000.0));
/// ```
#[inline]
pub fn ecef_distance(p0: (f64, f64, f64), p1: (f64, f64, f64)) -> f64 {
    let dx = p0.0 - p1.0;
    let dy = p0.1 - p1.1;
    let dz = p0.2 - p1.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    
    const EPSILON: f64 = 1e-6; // Tolerance for floating point comparisons
    
    #[test]
    fn test_llh2ecef_equator() {
        // Point on equator at prime meridian
        let (x, y, z) = llh2ecef(0.0, 0.0, 0.0);
        
        // Should be approximately (WGS84_A, 0, 0)
        assert!((x - WGS84_A).abs() < EPSILON);
        assert!(y.abs() < EPSILON);
        assert!(z.abs() < EPSILON);
    }
    
    #[test]
    fn test_llh2ecef_north_pole() {
        // North pole
        let (x, y, z) = llh2ecef(90.0, 0.0, 0.0);
        
        // Should be approximately (0, 0, WGS84_B)
        assert!(x.abs() < EPSILON);
        assert!(y.abs() < EPSILON);
        assert!((z - WGS84_B).abs() < 1.0); // Within 1 meter
    }
    
    #[test]
    fn test_ecef2llh_roundtrip() {
        // Test round-trip conversion
        let test_points = vec![
            (51.5, -0.1, 100.0),    // London
            (40.7, -74.0, 50.0),    // New York
            (-33.9, 18.4, 20.0),    // Cape Town
            (35.7, 139.7, 40.0),    // Tokyo
        ];
        
        for (lat, lon, alt) in test_points {
            let (x, y, z) = llh2ecef(lat, lon, alt);
            let (lat2, lon2, alt2) = ecef2llh(x, y, z);
            
            assert!((lat - lat2).abs() < 1e-9, "Latitude mismatch: {} vs {}", lat, lat2);
            assert!((lon - lon2).abs() < 1e-9, "Longitude mismatch: {} vs {}", lon, lon2);
            assert!((alt - alt2).abs() < 1e-3, "Altitude mismatch: {} vs {} (diff: {}m)", alt, alt2, (alt - alt2).abs());
        }
    }
    
    #[test]
    fn test_greatcircle_london_paris() {
        // London to Paris
        let dist = greatcircle(51.5074, -0.1278, 48.8566, 2.3522);
        
        // Should be approximately 344 km
        assert!((dist - 344000.0).abs() < 5000.0, "Distance: {} meters", dist);
    }
    
    #[test]
    fn test_greatcircle_same_point() {
        // Same point should have zero distance
        let dist = greatcircle(51.5, -0.1, 51.5, -0.1);
        assert!(dist.abs() < EPSILON);
    }
    
    #[test]
    fn test_ecef_distance() {
        // Two points 100km apart in X direction
        let p0 = (4000000.0, 0.0, 5000000.0);
        let p1 = (4100000.0, 0.0, 5000000.0);
        
        let dist = ecef_distance(p0, p1);
        assert!((dist - 100000.0).abs() < EPSILON);
    }
    
    #[test]
    fn test_ecef_distance_same_point() {
        let p = (4000000.0, 1000000.0, 5000000.0);
        let dist = ecef_distance(p, p);
        assert!(dist.abs() < EPSILON);
    }
    
    #[test]
    fn test_constants() {
        // Verify WGS84 constants match Python values
        assert!((WGS84_A - 6378137.0).abs() < EPSILON);
        assert!((WGS84_F - 1.0/298.257223563).abs() < 1e-15);
        
        // Verify derived constants
        let expected_b = WGS84_A * (1.0 - WGS84_F);
        assert!((WGS84_B - expected_b).abs() < EPSILON);
    }
}
