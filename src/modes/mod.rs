// Mode S message parsing and decoding
// Ported from Python/Cython implementation

pub mod crc;
pub mod altitude;
pub mod squawk;
pub mod cpr;
pub mod message;

// Re-export main types
pub use message::{decode, DecodedMessage, ModeSMessage, ESType};
pub use cpr::decode_cpr;
