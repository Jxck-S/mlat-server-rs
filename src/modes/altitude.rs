// Altitude decoding for Mode S messages
// Ported from modes/altitude.py and modes_cython/message.pyx

const BAD_ALT: i32 = -999999;

/// Decodes a Mode S 13-bit altitude field.
///
/// The expected ordering is as specified in §3.1.2.6.5.4 of Annex 10:
///   C1, A1, C2, A2, C4, A4, (M), B1, (Q), B2, D2, B4, D4
///
/// Returns signed altitude in feet, or None if not decodable.
pub fn decode_ac13(ac13: u16) -> Option<i32> {
    let alt = ALT_TABLE[(ac13 & 0x1fff) as usize];
    if alt == BAD_ALT {
        None
    } else {
        Some(alt)
    }
}

/// Decode a 12-bit AC altitude field from an extended squitter.
///
/// The expected ordering is as specified in Doc 9871 Table A-2-5:
///  the altitude code (AC) as specified in §3.1.2.6.5.4 of Annex 10,
///  but with the M-bit removed
///
/// Returns signed altitude in feet, or None if not a valid altitude.
pub fn decode_ac12(ac12: u16) -> Option<i32> {
    let alt = ALT_TABLE[(((ac12 & 0x0fc0) << 1) | (ac12 & 0x003f)) as usize];
    if alt == BAD_ALT {
        None
    } else {
        Some(alt)
    }
}

/// Internal function to decode AC13 field
const fn decode_ac13_internal(ac13: u16) -> i32 {
    if ac13 == 0 {
        return BAD_ALT; // no data
    }
    if ac13 & 0x0040 != 0 {
        return BAD_ALT; // M bit set
    }
    if ac13 & 0x0010 != 0 {
        // Q bit set - 25ft increments
        let n = ((ac13 & 0x1f80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000f);
        return (n as i32) * 25 - 1000;
    }

    // convert from Gillham code
    if (ac13 & 0x1500) == 0 {
        return BAD_ALT; // illegal C bits
    }

    let mut h = 0;
    if ac13 & 0x1000 != 0 {
        h ^= 7; // C1
    }
    if ac13 & 0x0400 != 0 {
        h ^= 3; // C2
    }
    if ac13 & 0x0100 != 0 {
        h ^= 1; // C4
    }
    if h & 5 != 0 {
        h ^= 5;
    }
    if h > 5 {
        return BAD_ALT; // illegal C bits
    }

    let mut f = 0;
    if ac13 & 0x0010 != 0 {
        f ^= 0x1ff; // D1
    }
    if ac13 & 0x0004 != 0 {
        f ^= 0x0ff; // D2
    }
    if ac13 & 0x0001 != 0 {
        f ^= 0x07f; // D4
    }
    if ac13 & 0x0800 != 0 {
        f ^= 0x03f; // A1
    }
    if ac13 & 0x0200 != 0 {
        f ^= 0x01f; // A2
    }
    if ac13 & 0x0080 != 0 {
        f ^= 0x00f; // A4
    }
    if ac13 & 0x0020 != 0 {
        f ^= 0x007; // B1
    }
    if ac13 & 0x0008 != 0 {
        f ^= 0x003; // B2
    }
    if ac13 & 0x0002 != 0 {
        f ^= 0x001; // B4
    }

    if f & 1 != 0 {
        h = 6 - h;
    }

    let a = 500 * f + 100 * h - 1300;
    if a < -1200 {
        return BAD_ALT; // illegal value
    }

    a
}

/// Precomputed altitude lookup table (8192 entries for 13-bit values)
const ALT_TABLE: [i32; 8192] = generate_alt_table();

/// Generate altitude lookup table at compile time
const fn generate_alt_table() -> [i32; 8192] {
    let mut table = [BAD_ALT; 8192];
    let mut i = 0;
    while i < 8192 {
        table[i] = decode_ac13_internal(i as u16);
        i += 1;
    }
    table
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_ac13_q_bit() {
        // Q-bit encoding from Python: n = ((ac13 & 0x1f80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000f)
        // altitude = n * 25 - 1000
        
        // For n=40: altitude = 40*25-1000 = 0 feet
        // Encode: ac13 = ((n << 2) & 0x1f80) | ((n << 1) & 0x0020) | (n & 0x000f) | 0x0010 (Q bit)
        let n = 40u16;
        let ac13 = ((n << 2) & 0x1f80) | ((n << 1) & 0x0020) | (n & 0x000f) | 0x0010;
        assert_eq!(decode_ac13(ac13), Some(0));
    }

    #[test]
    fn test_decode_ac13_m_bit() {
        // M bit set should return None
        let ac13 = 0x0040; // M bit at position 6
        assert_eq!(decode_ac13(ac13), None);
    }

    #[test]
    fn test_decode_ac13_zero() {
        // Zero should return None
        assert_eq!(decode_ac13(0), None);
    }

    #[test]
    fn test_decode_ac12() {
        // AC12 is like AC13 but without M bit
        // Test Q-bit encoding for n=40: altitude = 40*25-1000 = 0 feet
        // But AC12 encoding is different - it removes the M bit from AC13
        // So we need to test with a value that works for AC12
        let n = 40u16;
        let ac12 = ((n << 2) & 0x1f80) | ((n << 1) & 0x0020) | (n & 0x000f) | 0x0010;
        // Python returns 800 for this encoding, not 0
        assert_eq!(decode_ac12(ac12), Some(800));
    }

    #[test]
    fn test_altitude_range() {
        // Test various altitudes with Q-bit encoding
        for alt in [-1000, 0, 1000, 10000, 30000, 40000] {
            let n = (alt + 1000) / 25;
            if n >= 0 && n < 2048 {
                // Use Python's encoding: n = ((ac13 & 0x1f80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000f)
                let ac13 = (((n as u16) << 2) & 0x1f80) | (((n as u16) << 1) & 0x0020) | ((n as u16) & 0x000f) | 0x0010;
                let decoded = decode_ac13(ac13);
                assert!(decoded.is_some(), "Failed to decode altitude {} (n={})", alt, n);
                // Allow for rounding in division
                assert!((decoded.unwrap() - alt).abs() < 25, "Altitude mismatch: expected {}, got {}", alt, decoded.unwrap());
            }
        }
    }
}
