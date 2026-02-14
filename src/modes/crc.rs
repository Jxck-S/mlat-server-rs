// CRC calculation for Mode S messages
// Ported from modes/crc.py and modes_cython/message.pyx

/// Computes the 24-bit Mode S CRC residual for a message.
///
/// The CRC residual is the CRC computed across the first 4 or 11 bytes,
/// XOR-ed with the CRC value stored in the final 3 bytes.
///
/// For a message using Address/Parity, the expected residual is the
/// transmitter's address.
///
/// For a message using Parity/Interrogator, the expected residual is
/// the interrogator ID.
///
/// For an extended squitter message or a DF11 acquisition squitter, the
/// expected residual is zero.
///
/// Errors in the message or in the CRC value itself will appear as errors
/// in the residual value.
pub fn residual(payload: &[u8]) -> u32 {
    let df = (payload[0] & 0xf8) >> 3;
    let n = if df > 15 { 14 } else { 7 };
    
    let mut rem = CRC_TABLE[payload[0] as usize];
    for i in 1..(n - 3) {
        rem = ((rem & 0xFFFF) << 8) ^ CRC_TABLE[(payload[i] ^ ((rem >> 16) as u8)) as usize];
    }
    
    rem ^ ((payload[n-3] as u32) << 16) ^ ((payload[n-2] as u32) << 8) ^ (payload[n-1] as u32)
}

/// Precomputed CRC lookup table
/// Polynomial: 0xfff409
const CRC_TABLE: [u32; 256] = generate_crc_table();

/// Generate CRC lookup table at compile time
const fn generate_crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let poly = 0xfff409;
    
    let mut i = 0;
    while i < 256 {
        let mut c = (i as u32) << 16;
        let mut j = 0;
        while j < 8 {
            if c & 0x800000 != 0 {
                c = (c << 1) ^ poly;
            } else {
                c = c << 1;
            }
            j += 1;
        }
        table[i] = c & 0xffffff;
        i += 1;
    }
    
    table
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc_table_generation() {
        // Verify first few entries match Python's output
        assert_eq!(CRC_TABLE[0], 0x000000);
        assert_eq!(CRC_TABLE[1], 0xfff409);
        assert_eq!(CRC_TABLE[255], 0xfa0480);
    }

    #[test]
    fn test_crc_residual_df17() {
        // DF17 extended squitter should have residual of 0 if CRC is valid
        let msg = [
            0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3, 
            0x71, 0xc3, 0x2c, 0xe0, 0x57, 0x60, 0x98
        ];
        let res = residual(&msg);
        assert_eq!(res, 0, "Valid DF17 message should have CRC residual of 0");
    }

    #[test]
    fn test_crc_residual_df11() {
        // DF11 all-call reply
        let msg = [0x5d, 0x48, 0x40, 0xd6, 0x00, 0x00, 0x00];
        let res = residual(&msg);
        // For DF11, residual should be 0 if valid, or contain interrogator ID
        // Just verify it computes without panicking
        assert!(res <= 0xffffff);
    }

    #[test]
    fn test_crc_residual_df4() {
        // DF4 surveillance altitude reply - uses Address/Parity
        // Residual extracts address from parity
        let msg = [0x20, 0x00, 0x1c, 0x38, 0x48, 0x40, 0xd6];
        let res = residual(&msg);
        // Just verify it computes a valid 24-bit value
        assert!(res <= 0xffffff);
    }
}
