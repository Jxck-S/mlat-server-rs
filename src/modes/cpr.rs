// CPR (Compact Position Reporting) decoder
// Ported from modes/cpr.py and modes_cython/message.pyx

use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum CprError {
    LatitudeOutOfRange,
    DifferentLatitudeZones,
}

impl fmt::Display for CprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CprError::LatitudeOutOfRange => write!(f, "latitude out of range"),
            CprError::DifferentLatitudeZones => write!(f, "messages lie in different latitude zones"),
        }
    }
}

impl std::error::Error for CprError {}

/// Perform globally unambiguous position decoding for a pair of airborne CPR messages.
///
/// # Arguments
/// * `lat_e` - Raw latitude value of the even message (17 bits)
/// * `lon_e` - Raw longitude value of the even message (17 bits)
/// * `lat_o` - Raw latitude value of the odd message (17 bits)
/// * `lon_o` - Raw longitude value of the odd message (17 bits)
///
/// # Returns
/// A tuple of `(even_lat, even_lon, odd_lat, odd_lon)` in degrees
///
/// # Errors
/// Returns `CprError` if the messages do not produce a useful position.
pub fn decode_cpr(lat_e: u32, lon_e: u32, lat_o: u32, lon_o: u32) -> Result<(f64, f64, f64, f64), CprError> {
    // Compute the Latitude Index "j"
    let j = ((59.0 * lat_e as f64 - 60.0 * lat_o as f64) / 131072.0 + 0.5).floor() as i32;
    let mut rlat_e = (360.0 / 60.0) * (mod_int(j, 60) as f64 + lat_e as f64 / 131072.0);
    let mut rlat_o = (360.0 / 59.0) * (mod_int(j, 59) as f64 + lat_o as f64 / 131072.0);

    // Adjust for southern hemisphere values, which are in the range (270,360)
    if rlat_e >= 270.0 {
        rlat_e -= 360.0;
    }
    if rlat_o >= 270.0 {
        rlat_o -= 360.0;
    }

    // Check to see that the latitude is in range: -90 .. +90
    if rlat_e < -90.0 || rlat_e > 90.0 || rlat_o < -90.0 || rlat_o > 90.0 {
        return Err(CprError::LatitudeOutOfRange);
    }

    // Find latitude zone, abort if the two positions are not in the same zone
    let nl_e = nl(rlat_e);
    if nl_e != nl(rlat_o) {
        return Err(CprError::DifferentLatitudeZones);
    }

    // Compute n(i)
    let n_e = nl_e;
    let n_o = i32::max(1, nl_e - 1);

    // Compute the Longitude Index "m"
    let m = ((lon_e as f64 * (nl_e - 1) as f64 - lon_o as f64 * nl_e as f64) / 131072.0 + 0.5).floor() as i32;

    // Compute global longitudes
    let mut rlon_e = (360.0 / n_e as f64) * (mod_int(m, n_e) as f64 + lon_e as f64 / 131072.0);
    let mut rlon_o = (360.0 / n_o as f64) * (mod_int(m, n_o) as f64 + lon_o as f64 / 131072.0);

    // Renormalize to -180 .. +180
    rlon_e -= ((rlon_e + 180.0) / 360.0).floor() * 360.0;
    rlon_o -= ((rlon_o + 180.0) / 360.0).floor() * 360.0;

    Ok((rlat_e, rlon_e, rlat_o, rlon_o))
}

/// Modulo operation that always returns positive result
#[inline]
fn mod_int(a: i32, b: i32) -> i32 {
    let r = a % b;
    if r < 0 {
        r + b
    } else {
        r
    }
}

/// Number of longitude zones for a given latitude
/// Optimized binary search tree implementation from Cython version
#[inline]
fn nl(lat: f64) -> i32 {
    let lat = lat.abs();

    if lat < 33.53993436 {
        if lat < 10.47047130 { return 59; }
        if lat < 14.82817437 { return 58; }
        if lat < 18.18626357 { return 57; }
        if lat < 21.02939493 { return 56; }
        if lat < 23.54504487 { return 55; }
        if lat < 25.82924707 { return 54; }
        if lat < 27.93898710 { return 53; }
        if lat < 29.91135686 { return 52; }
        if lat < 31.77209708 { return 51; }
        return 50;
    }
    if lat < 45.54626723 {
        if lat < 35.22899598 { return 49; }
        if lat < 36.85025108 { return 48; }
        if lat < 38.41241892 { return 47; }
        if lat < 39.92256684 { return 46; }
        if lat < 41.38651832 { return 45; }
        if lat < 42.80914012 { return 44; }
        if lat < 44.19454951 { return 43; }
        return 42;
    }
    if lat < 56.59318756 {
        if lat < 46.86733252 { return 41; }
        if lat < 48.16039128 { return 40; }
        if lat < 49.42776439 { return 39; }
        if lat < 50.67150166 { return 38; }
        if lat < 51.89342469 { return 37; }
        if lat < 53.09516153 { return 36; }
        if lat < 54.27817472 { return 35; }
        if lat < 55.44378444 { return 34; }
        return 33;
    }
    if lat < 66.36171008 {
        if lat < 57.72747354 { return 32; }
        if lat < 58.84763776 { return 31; }
        if lat < 59.95459277 { return 30; }
        if lat < 61.04917774 { return 29; }
        if lat < 62.13216659 { return 28; }
        if lat < 63.20427479 { return 27; }
        if lat < 64.26616523 { return 26; }
        if lat < 65.31845310 { return 25; }
        return 24;
    }
    if lat < 75.42056257 {
        if lat < 67.39646774 { return 23; }
        if lat < 68.42322022 { return 22; }
        if lat < 69.44242631 { return 21; }
        if lat < 70.45451075 { return 20; }
        if lat < 71.45986473 { return 19; }
        if lat < 72.45884545 { return 18; }
        if lat < 73.45177442 { return 17; }
        if lat < 74.43893416 { return 16; }
        return 15;
    }
    if lat < 76.39684391 { return 14; }
    if lat < 77.36789461 { return 13; }
    if lat < 78.33374083 { return 12; }
    if lat < 79.29428225 { return 11; }
    if lat < 80.24923213 { return 10; }
    if lat < 81.19801349 { return 9; }
    if lat < 82.13956981 { return 8; }
    if lat < 83.07199445 { return 7; }
    if lat < 83.99173563 { return 6; }
    if lat < 84.89166191 { return 5; }
    if lat < 85.75541621 { return 4; }
    if lat < 86.53536998 { return 3; }
    if lat <= 87.00000000 { return 2; }  // CRITICAL: Use <= to match Python bisect_left behavior
    1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nl_function() {
        // Test NL function for various latitudes - values verified against Python
        assert_eq!(nl(0.0), 59);
        assert_eq!(nl(10.0), 59);
        assert_eq!(nl(20.0), 56);
        assert_eq!(nl(30.0), 51);  // Python returns 51
        assert_eq!(nl(40.0), 45);  // Python returns 45, not 42
        assert_eq!(nl(50.0), 38);
        assert_eq!(nl(60.0), 29);
        assert_eq!(nl(70.0), 20);
        assert_eq!(nl(80.0), 10);
        assert_eq!(nl(87.0), 2);  // Python returns 2 (bisect_left behavior)
        assert_eq!(nl(90.0), 1);
        
        // Test negative latitudes (should use absolute value)
        assert_eq!(nl(-30.0), 51);
        assert_eq!(nl(-60.0), 29);
    }

    #[test]
    fn test_mod_int() {
        assert_eq!(mod_int(5, 3), 2);
        assert_eq!(mod_int(-1, 3), 2);
        assert_eq!(mod_int(-5, 3), 1);
        assert_eq!(mod_int(0, 5), 0);
    }

    #[test]
    fn test_decode_cpr_valid() {
        // Test with known CPR position pair
        // These values should decode to a valid position
        let lat_e = 93000;
        let lon_e = 51372;
        let lat_o = 74158;
        let lon_o = 50194;
        
        let result = decode_cpr(lat_e, lon_e, lat_o, lon_o);
        assert!(result.is_ok());
        
        let (rlat_e, rlon_e, rlat_o, rlon_o) = result.unwrap();
        
        // Verify latitudes and longitudes are in valid ranges
        assert!(rlat_e >= -90.0 && rlat_e <= 90.0);
        assert!(rlat_o >= -90.0 && rlat_o <= 90.0);
        assert!(rlon_e >= -180.0 && rlon_e <= 180.0);
        assert!(rlon_o >= -180.0 && rlon_o <= 180.0);
    }

    // TODO: Add proper out-of-range test with verified values
    // The current test values don't actually produce an out-of-range error
}
