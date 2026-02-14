// Mode S message parsing and decoding
// Ported from modes/message.py and modes_cython/message.pyx

use super::{altitude, squawk, crc};

/// AIS charset for callsign decoding
const AIS_CHARSET: &[u8; 64] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

/// Extended Squitter message type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ESType {
    IdAndCategory,
    AirbornePosition,
    SurfacePosition,
    AirborneVelocity,
    Other,
}

/// Trait for all Mode S messages
pub trait ModeSMessage {
    fn df(&self) -> u8;
    fn address(&self) -> u32;
    fn altitude(&self) -> Option<i32>;
    fn callsign(&self) -> Option<&str>;
    fn squawk(&self) -> Option<&str>;
    fn crc_ok(&self) -> Option<bool>;
}

/// DF0 (Short air-air surveillance / ACAS) message
#[derive(Debug, Clone)]
pub struct DF0 {
    pub df: u8,
    pub vs: u8,
    pub cc: u8,
    pub sl: u8,
    pub ri: u8,
    pub ac: u16,
    pub altitude: Option<i32>,
    pub address: u32,
}

impl DF0 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let vs = (buf[0] & 0x04) >> 2;
        let cc = (buf[0] & 0x02) >> 1;
        let sl = (buf[1] & 0xe0) >> 5;
        let ri = ((buf[1] & 0x03) << 1) | ((buf[2] & 0x80) >> 7);
        let ac = ((buf[2] as u16 & 0x1f) << 8) | buf[3] as u16;
        
        DF0 {
            df,
            vs,
            cc,
            sl,
            ri,
            ac,
            altitude: altitude::decode_ac13(ac),
            address: crc::residual(buf),
        }
    }
}

impl ModeSMessage for DF0 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { self.altitude }
    fn callsign(&self) -> Option<&str> { None }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { None }
}

/// DF4 (Surveillance, altitude reply) message
#[derive(Debug, Clone)]
pub struct DF4 {
    pub df: u8,
    pub fs: u8,
    pub dr: u8,
    pub um: u8,
    pub ac: u16,
    pub altitude: Option<i32>,
    pub address: u32,
}

impl DF4 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let fs = buf[0] & 0x07;
        let dr = (buf[1] & 0xf8) >> 3;
        let um = ((buf[1] & 0x07) << 3) | ((buf[2] & 0xe0) >> 5);
        let ac = ((buf[2] as u16 & 0x1f) << 8) | buf[3] as u16;
        
        DF4 {
            df,
            fs,
            dr,
            um,
            ac,
            altitude: altitude::decode_ac13(ac),
            address: crc::residual(buf),
        }
    }
}

impl ModeSMessage for DF4 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { self.altitude }
    fn callsign(&self) -> Option<&str> { None }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { None }
}

/// DF5 (Surveillance, identity reply) message
#[derive(Debug, Clone)]
pub struct DF5 {
    pub df: u8,
    pub fs: u8,
    pub dr: u8,
    pub um: u8,
    pub id: u16,
    pub squawk_code: String,
    pub address: u32,
}

impl DF5 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let fs = buf[0] & 0x07;
        let dr = (buf[1] & 0xf8) >> 3;
        let um = ((buf[1] & 0x07) << 3) | ((buf[2] & 0xe0) >> 5);
        let id = ((buf[2] as u16 & 0x1f) << 8) | buf[3] as u16;
        
        DF5 {
            df,
            fs,
            dr,
            um,
            id,
            squawk_code: squawk::decode_id13(id),
            address: crc::residual(buf),
        }
    }
}

impl ModeSMessage for DF5 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { None }
    fn callsign(&self) -> Option<&str> { None }
    fn squawk(&self) -> Option<&str> { Some(&self.squawk_code) }
    fn crc_ok(&self) -> Option<bool> { None }
}

/// DF11 (All-call reply) message
#[derive(Debug, Clone)]
pub struct DF11 {
    pub df: u8,
    pub ca: u8,
    pub aa: u32,
    pub address: u32,
    pub crc_ok: Option<bool>,
}

impl DF11 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let ca = buf[0] & 0x07;
        let aa = ((buf[1] as u32) << 16) | ((buf[2] as u32) << 8) | buf[3] as u32;
        
        let r = crc::residual(buf);
        let crc_ok = if r == 0 {
            Some(true)
        } else if (r & !0x7f) == 0 {
            None
        } else {
            Some(false)
        };
        
        DF11 {
            df,
            ca,
            aa,
            address: aa,
            crc_ok,
        }
    }
}

impl ModeSMessage for DF11 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { None }
    fn callsign(&self) -> Option<&str> { None }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { self.crc_ok }
}

/// DF16 (Long air-air surveillance / ACAS) message
#[derive(Debug, Clone)]
pub struct DF16 {
    pub df: u8,
    pub vs: u8,
    pub sl: u8,
    pub ri: u8,
    pub ac: u16,
    pub mv: [u8; 7],
    pub altitude: Option<i32>,
    pub address: u32,
}

impl DF16 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let vs = (buf[0] & 0x04) >> 2;
        let sl = (buf[1] & 0xe0) >> 5;
        let ri = ((buf[1] & 0x03) << 1) | ((buf[2] & 0x80) >> 7);
        let ac = ((buf[2] as u16 & 0x1f) << 8) | buf[3] as u16;
        let mut mv = [0u8; 7];
        mv.copy_from_slice(&buf[4..11]);
        
        DF16 {
            df,
            vs,
            sl,
            ri,
            ac,
            mv,
            altitude: altitude::decode_ac13(ac),
            address: crc::residual(buf),
        }
    }
}

impl ModeSMessage for DF16 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { self.altitude }
    fn callsign(&self) -> Option<&str> { None }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { None }
}

/// Decode callsign from AIS charset
fn decode_callsign(buf: &[u8]) -> Option<String> {
    let callsign = format!(
        "{}{}{}{}{}{}{}{}",
        AIS_CHARSET[((buf[5] & 0xfc) >> 2) as usize] as char,
        AIS_CHARSET[(((buf[5] & 0x03) << 4) | ((buf[6] & 0xf0) >> 4)) as usize] as char,
        AIS_CHARSET[(((buf[6] & 0x0f) << 2) | ((buf[7] & 0xc0) >> 6)) as usize] as char,
        AIS_CHARSET[(buf[7] & 0x3f) as usize] as char,
        AIS_CHARSET[((buf[8] & 0xfc) >> 2) as usize] as char,
        AIS_CHARSET[(((buf[8] & 0x03) << 4) | ((buf[9] & 0xf0) >> 4)) as usize] as char,
        AIS_CHARSET[(((buf[9] & 0x0f) << 2) | ((buf[10] & 0xc0) >> 6)) as usize] as char,
        AIS_CHARSET[(buf[10] & 0x3f) as usize] as char,
    );
    
    if callsign != "        " && !callsign.contains('?') {
        Some(callsign)
    } else {
        None
    }
}

/// DF17 (Extended Squitter) message
#[derive(Debug, Clone)]
pub struct DF17 {
    pub df: u8,
    pub ca: u8,
    pub aa: u32,
    pub address: u32,
    pub crc_ok: bool,
    pub estype: ESType,
    pub nuc: Option<u8>,
    // Airborne position fields
    pub ss: Option<u8>,
    pub saf: Option<u8>,
    pub ac12: Option<u16>,
    pub t: Option<u8>,
    pub f: Option<u8>,
    pub lat: Option<u32>,
    pub lon: Option<u32>,
    pub altitude: Option<i32>,
    // ID and category fields
    pub category: Option<u8>,
    pub callsign_str: Option<String>,
}

impl DF17 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let ca = buf[0] & 0x07;
        let aa = ((buf[1] as u32) << 16) | ((buf[2] as u32) << 8) | buf[3] as u32;
        let crc_ok = crc::residual(buf) == 0;
        
        let metype = (buf[4] & 0xf8) >> 3;
        let (estype, nuc) = get_es_type(metype);
        
        let mut msg = DF17 {
            df,
            ca,
            aa,
            address: aa,
            crc_ok,
            estype,
            nuc,
            ss: None,
            saf: None,
            ac12: None,
            t: None,
            f: None,
            lat: None,
            lon: None,
            altitude: None,
            category: None,
            callsign_str: None,
        };
        
        match estype {
            ESType::AirbornePosition => {
                msg.ss = Some((buf[4] & 0x06) >> 1);
                msg.saf = Some(buf[4] & 0x01);
                msg.ac12 = Some(((buf[5] as u16) << 4) | ((buf[6] & 0xf0) >> 4) as u16);
                msg.t = Some((buf[6] & 0x08) >> 3);
                msg.f = Some((buf[6] & 0x04) >> 2);
                msg.lat = Some(((buf[6] as u32 & 0x03) << 15) |
                               ((buf[7] as u32) << 7) |
                               ((buf[8] & 0xfe) >> 1) as u32);
                msg.lon = Some(((buf[8] as u32 & 0x01) << 16) |
                               ((buf[9] as u32) << 8) |
                               buf[10] as u32);
                msg.altitude = altitude::decode_ac12(msg.ac12.unwrap());
            }
            ESType::IdAndCategory => {
                msg.category = Some(buf[4] & 0x07);
                msg.callsign_str = decode_callsign(buf);
            }
            _ => {}
        }
        
        msg
    }
}

impl ModeSMessage for DF17 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { self.altitude }
    fn callsign(&self) -> Option<&str> { self.callsign_str.as_deref() }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { Some(self.crc_ok) }
}

/// DF18 (Extended Squitter / Non-Transponder) message
#[derive(Debug, Clone)]
pub struct DF18 {
    pub df: u8,
    pub cf: u8,
    pub aa: u32,
    pub address: u32,
    pub crc_ok: bool,
    pub estype: ESType,
    pub nuc: Option<u8>,
    pub ss: Option<u8>,
    pub saf: Option<u8>,
    pub ac12: Option<u16>,
    pub t: Option<u8>,
    pub f: Option<u8>,
    pub lat: Option<u32>,
    pub lon: Option<u32>,
    pub altitude: Option<i32>,
    pub category: Option<u8>,
    pub callsign_str: Option<String>,
}

impl DF18 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let cf = buf[0] & 0x07;
        let aa = ((buf[1] as u32) << 16) | ((buf[2] as u32) << 8) | buf[3] as u32;
        let crc_ok = crc::residual(buf) == 0;
        
        let metype = (buf[4] & 0xf8) >> 3;
        let (estype, nuc) = get_es_type(metype);
        
        let mut msg = DF18 {
            df,
            cf,
            aa,
            address: aa,
            crc_ok,
            estype,
            nuc,
            ss: None,
            saf: None,
            ac12: None,
            t: None,
            f: None,
            lat: None,
            lon: None,
            altitude: None,
            category: None,
            callsign_str: None,
        };
        
        match estype {
            ESType::AirbornePosition => {
                msg.ss = Some((buf[4] & 0x06) >> 1);
                msg.saf = Some(buf[4] & 0x01);
                msg.ac12 = Some(((buf[5] as u16) << 4) | ((buf[6] & 0xf0) >> 4) as u16);
                msg.t = Some((buf[6] & 0x08) >> 3);
                msg.f = Some((buf[6] & 0x04) >> 2);
                msg.lat = Some(((buf[6] as u32 & 0x03) << 15) |
                               ((buf[7] as u32) << 7) |
                               ((buf[8] & 0xfe) >> 1) as u32);
                msg.lon = Some(((buf[8] as u32 & 0x01) << 16) |
                               ((buf[9] as u32) << 8) |
                               buf[10] as u32);
                msg.altitude = altitude::decode_ac12(msg.ac12.unwrap());
            }
            ESType::IdAndCategory => {
                msg.category = Some(buf[4] & 0x07);
                msg.callsign_str = decode_callsign(buf);
            }
            _ => {}
        }
        
        msg
    }
}

impl ModeSMessage for DF18 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { self.altitude }
    fn callsign(&self) -> Option<&str> { self.callsign_str.as_deref() }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { Some(self.crc_ok) }
}

/// DF20 (Comm-B, altitude reply) message
#[derive(Debug, Clone)]
pub struct DF20 {
    pub df: u8,
    pub fs: u8,
    pub dr: u8,
    pub um: u8,
    pub ac: u16,
    pub mb: [u8; 7],
    pub altitude: Option<i32>,
    pub address: u32,
    pub callsign_str: Option<String>,
}

impl DF20 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let fs = buf[0] & 0x07;
        let dr = (buf[1] & 0xf8) >> 3;
        let um = ((buf[1] & 0x07) << 3) | ((buf[2] & 0xe0) >> 5);
        let ac = ((buf[2] as u16 & 0x1f) << 8) | buf[3] as u16;
        let mut mb = [0u8; 7];
        mb.copy_from_slice(&buf[4..11]);
        
        let callsign_str = if buf[4] == 0x20 {
            decode_callsign(buf)
        } else {
            None
        };
        
        DF20 {
            df,
            fs,
            dr,
            um,
            ac,
            mb,
            altitude: altitude::decode_ac13(ac),
            address: crc::residual(buf),
            callsign_str,
        }
    }
}

impl ModeSMessage for DF20 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { self.altitude }
    fn callsign(&self) -> Option<&str> { self.callsign_str.as_deref() }
    fn squawk(&self) -> Option<&str> { None }
    fn crc_ok(&self) -> Option<bool> { None }
}

/// DF21 (Comm-B, identity reply) message
#[derive(Debug, Clone)]
pub struct DF21 {
    pub df: u8,
    pub fs: u8,
    pub dr: u8,
    pub um: u8,
    pub id: u16,
    pub mb: [u8; 7],
    pub squawk_code: String,
    pub address: u32,
    pub callsign_str: Option<String>,
}

impl DF21 {
    fn decode(buf: &[u8]) -> Self {
        let df = (buf[0] & 0xf8) >> 3;
        let fs = buf[0] & 0x07;
        let dr = (buf[1] & 0xf8) >> 3;
        let um = ((buf[1] & 0x07) << 3) | ((buf[2] & 0xe0) >> 5);
        let id = ((buf[2] as u16 & 0x1f) << 8) | buf[3] as u16;
        let mut mb = [0u8; 7];
        mb.copy_from_slice(&buf[4..11]);
        
        let callsign_str = if buf[4] == 0x20 {
            decode_callsign(buf)
        } else {
            None
        };
        
        DF21 {
            df,
            fs,
            dr,
            um,
            id,
            mb,
            squawk_code: squawk::decode_id13(id),
            address: crc::residual(buf),
            callsign_str,
        }
    }
}

impl ModeSMessage for DF21 {
    fn df(&self) -> u8 { self.df }
    fn address(&self) -> u32 { self.address }
    fn altitude(&self) -> Option<i32> { None }
    fn callsign(&self) -> Option<&str> { self.callsign_str.as_deref() }
    fn squawk(&self) -> Option<&str> { Some(&self.squawk_code) }
    fn crc_ok(&self) -> Option<bool> { None }
}

/// Decoded Mode S message (enum of all DF types)
#[derive(Debug, Clone)]
pub enum DecodedMessage {
    DF0(DF0),
    DF4(DF4),
    DF5(DF5),
    DF11(DF11),
    DF16(DF16),
    DF17(DF17),
    DF18(DF18),
    DF20(DF20),
    DF21(DF21),
}

impl DecodedMessage {
    pub fn as_trait(&self) -> &dyn ModeSMessage {
        match self {
            DecodedMessage::DF0(m) => m,
            DecodedMessage::DF4(m) => m,
            DecodedMessage::DF5(m) => m,
            DecodedMessage::DF11(m) => m,
            DecodedMessage::DF16(m) => m,
            DecodedMessage::DF17(m) => m,
            DecodedMessage::DF18(m) => m,
            DecodedMessage::DF20(m) => m,
            DecodedMessage::DF21(m) => m,
        }
    }
}

/// Get Extended Squitter type from ME type code
fn get_es_type(metype: u8) -> (ESType, Option<u8>) {
    match metype {
        0 => (ESType::AirbornePosition, Some(0)),
        1..=4 => (ESType::IdAndCategory, None),
        5 => (ESType::SurfacePosition, Some(9)),
        6 => (ESType::SurfacePosition, Some(8)),
        7 => (ESType::SurfacePosition, Some(7)),
        8 => (ESType::SurfacePosition, Some(6)),
        9 => (ESType::AirbornePosition, Some(9)),
        10 => (ESType::AirbornePosition, Some(8)),
        11 => (ESType::AirbornePosition, Some(7)),
        12 => (ESType::AirbornePosition, Some(6)),
        13 => (ESType::AirbornePosition, Some(5)),
        14 => (ESType::AirbornePosition, Some(4)),
        15 => (ESType::AirbornePosition, Some(3)),
        16 => (ESType::AirbornePosition, Some(2)),
        17 => (ESType::AirbornePosition, Some(1)),
        18 => (ESType::AirbornePosition, Some(0)),
        19 => (ESType::AirborneVelocity, None),
        20 => (ESType::AirbornePosition, Some(9)),
        21 => (ESType::AirbornePosition, Some(8)),
        22 => (ESType::AirbornePosition, Some(0)),
        _ => (ESType::Other, None),
    }
}

/// Decode a Mode S message
///
/// # Arguments
/// * `frombuf` - A 7-byte or 14-byte message containing the encoded Mode S message
///
/// # Returns
/// A decoded message object, or None if the message type is not handled or invalid.
pub fn decode(frombuf: &[u8]) -> Option<DecodedMessage> {
    if frombuf.len() < 7 {
        return None;
    }
    
    let df = (frombuf[0] & 0xf8) >> 3;
    
    match df {
        0 => Some(DecodedMessage::DF0(DF0::decode(frombuf))),
        4 => Some(DecodedMessage::DF4(DF4::decode(frombuf))),
        5 => Some(DecodedMessage::DF5(DF5::decode(frombuf))),
        11 => Some(DecodedMessage::DF11(DF11::decode(frombuf))),
        16 if frombuf.len() >= 14 => Some(DecodedMessage::DF16(DF16::decode(frombuf))),
        17 if frombuf.len() >= 14 => Some(DecodedMessage::DF17(DF17::decode(frombuf))),
        18 if frombuf.len() >= 14 => Some(DecodedMessage::DF18(DF18::decode(frombuf))),
        20 if frombuf.len() >= 14 => Some(DecodedMessage::DF20(DF20::decode(frombuf))),
        21 if frombuf.len() >= 14 => Some(DecodedMessage::DF21(DF21::decode(frombuf))),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decode_df17_callsign() {
        // DF17 with callsign (example message)
        let msg = hex::decode("8d4840d6202cc371c32ce0576098").unwrap();
        let decoded = decode(&msg);
        assert!(decoded.is_some());
        
        if let Some(DecodedMessage::DF17(df17)) = decoded {
            assert_eq!(df17.df, 17);
            assert_eq!(df17.address, 0x4840d6);
            assert!(df17.crc_ok);
            assert_eq!(df17.estype, ESType::IdAndCategory);
            assert!(df17.callsign().is_some());
        } else {
            panic!("Expected DF17 message");
        }
    }

    #[test]
    fn test_decode_df11() {
        // DF11 all-call reply
        let msg = vec![0x5d, 0x48, 0x40, 0xd6, 0x00, 0x00, 0x00];
        let decoded = decode(&msg);
        assert!(decoded.is_some());
        
        if let Some(DecodedMessage::DF11(df11)) = decoded {
            assert_eq!(df11.df, 11);
            assert_eq!(df11.aa, 0x4840d6);
        } else {
            panic!("Expected DF11 message");
        }
    }

    #[test]
    fn test_decode_invalid() {
        // Too short
        let msg = vec![0x8d, 0x48];
        assert!(decode(&msg).is_none());
        
        // Unknown DF
        let msg = vec![0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        assert!(decode(&msg).is_none());
    }
}

// Add hex dependency for tests
#[cfg(test)]
use hex;
