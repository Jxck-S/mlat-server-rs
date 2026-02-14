// JSON message type definitions
// Defines client ↔ server message protocol

use std::collections::HashMap;
use serde::{Deserialize, Serialize};

/// Initial handshake as sent by clients (flat JSON, no "type" field).
/// Matches Python mlat-client and test client format.
#[derive(Debug, Deserialize)]
pub struct HandshakeRequest {
    pub version: u32,
    pub user: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    #[serde(default = "default_compress")]
    pub compress: Vec<String>,
    #[serde(default)]
    pub clock_type: Option<String>,
    #[serde(default)]
    pub uuid: Option<String>,
    #[serde(default)]
    pub privacy: bool,
    #[serde(default)]
    pub connection_info: Option<String>,
}

fn default_compress() -> Vec<String> {
    vec!["none".to_string()]
}

/// Wrapped sync message as sent by clients: {"sync": {"et": ..., "ot": ..., "em": ..., "om": ...}}
#[derive(Debug, Deserialize)]
pub struct SyncPayload {
    pub et: f64,
    pub ot: f64,
    pub em: String,
    pub om: String,
}

/// Messages sent from client to server (post-handshake messages use "type" tag)
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum ClientMessage {
    /// Handshake with explicit type (alternative to HandshakeRequest)
    Handshake {
        version: u32,
        user: String,
        lat: f64,
        lon: f64,
        alt: f64,
        #[serde(default)]
        clock_type: Option<String>,
        #[serde(default)]
        uuid: Option<String>,
        #[serde(default)]
        privacy: bool,
        #[serde(default)]
        connection_info: Option<String>,
    },
    
    /// MLAT message with timestamp
    Mlat {
        timestamp: f64,
        message: String,  // Hex-encoded Mode S message
        #[serde(default)]
        utc: Option<f64>,
    },
    
    /// Sync message (CPR even/odd pair)
    Sync {
        even_timestamp: f64,
        odd_timestamp: f64,
        even_message: String,
        odd_message: String,
    },
    /// Keep-alive heartbeat
    Heartbeat {},
    
    /// Rate report for tracked aircraft
    RateReport {
        #[serde(flatten)]
        rates: HashMap<u32, f64>,
    },
}

/// Messages sent from server to client
#[derive(Debug, Serialize, Clone)]
#[serde(tag = "type")]
pub enum ServerMessage {
    /// Handshake response
    #[serde(rename = "handshake")]
    Handshake {
        motd: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        reconnect_in: Option<u32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        compress: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        selective_traffic: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        heartbeat: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        return_results: Option<bool>,
    },
    
    /// Traffic filter request
    #[serde(rename = "mlat_request")]
    TrafficRequest {
        #[serde(skip_serializing_if = "Vec::is_empty")]
        start: Vec<String>,  // ICAO addresses to start tracking
        #[serde(skip_serializing_if = "Vec::is_empty")]
        stop: Vec<String>,   // ICAO addresses to stop tracking
    },
    
    /// MLAT result
    #[serde(rename = "mlat_result")]
    MlatResult {
        icao: String,
        lat: f64,
        lon: f64,
        alt: f64,
        #[serde(skip_serializing_if = "Option::is_none")]
        callsign: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        squawk: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        hdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        vdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        tdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        gdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        nstations: Option<usize>,
    },
    
    /// Heartbeat message
    #[serde(rename = "heartbeat")]
    Heartbeat {
        server_time: f64,
    },
    
    /// MLAT result (for receiver clock tuning)
    #[serde(rename = "position")]
    Position {
        #[serde(rename = "@")]
        timestamp: f64,
        addr: String,
        lat: f64,
        lon: f64,
        alt: f64,
        #[serde(skip_serializing_if = "Option::is_none")]
        callsign: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        squawk: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        hdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        vdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        tdop: Option<f64>,
        #[serde(skip_serializing_if = "Option::is_none")]
        gdop: Option<f64>,
        nstations: usize,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_deserialize_handshake_flat() {
        // Client sends flat format (no "type" field) - HandshakeRequest
        let json = r#"{
            "version": 2,
            "user": "test_user",
            "lat": 37.5,
            "lon": -122.0,
            "alt": 100.0,
            "compress": ["none"],
            "clock_type": "dump1090"
        }"#;

        let req: HandshakeRequest = serde_json::from_str(json).unwrap();
        assert_eq!(req.version, 2);
        assert_eq!(req.user, "test_user");
        assert_eq!(req.lat, 37.5);
        assert_eq!(req.lon, -122.0);
        assert_eq!(req.alt, 100.0);
        assert_eq!(req.clock_type.as_deref(), Some("dump1090"));
        assert_eq!(req.compress, vec!["none"]);
    }

    #[test]
    fn test_deserialize_handshake_with_type() {
        // Optional: client can send with "type": "handshake" (ClientMessage)
        let json = r#"{
            "type": "handshake",
            "version": 2,
            "user": "test_user",
            "lat": 37.5,
            "lon": -122.0,
            "alt": 100.0,
            "clock_type": "dump1090"
        }"#;

        let msg: ClientMessage = serde_json::from_str(json).unwrap();
        match msg {
            ClientMessage::Handshake { version, user, lat, lon, alt, clock_type, .. } => {
                assert_eq!(version, 2);
                assert_eq!(user, "test_user");
                assert_eq!(lat, 37.5);
                assert_eq!(lon, -122.0);
                assert_eq!(alt, 100.0);
                assert_eq!(clock_type, Some("dump1090".to_string()));
            }
            _ => panic!("Expected Handshake"),
        }
    }
    
    #[test]
    fn test_deserialize_mlat() {
        let json = r#"{
            "type": "mlat",
            "timestamp": 123456.789,
            "message": "8D4840D58B...",
            "utc": 1234567890.123
        }"#;
        
        let msg: ClientMessage = serde_json::from_str(json).unwrap();
        
        match msg {
            ClientMessage::Mlat { timestamp, message, utc, .. } => {
                assert_eq!(timestamp, 123456.789);
                assert_eq!(message, "8D4840D58B...");
                assert_eq!(utc, Some(1234567890.123));
            }
            _ => panic!("Expected Mlat"),
        }
    }
    
    #[test]
    fn test_serialize_handshake_response() {
        let msg = ServerMessage::Handshake {
            motd: "Welcome!".to_string(),
            reconnect_in: Some(300),
            compress: Some("zlib".to_string()),
            selective_traffic: Some(true),
            heartbeat: Some(true),
            return_results: Some(true),
        };
        
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("\"type\":\"handshake\""));
        assert!(json.contains("\"motd\":\"Welcome!\""));
        assert!(json.contains("\"reconnect_in\":300"));
    }
    
    #[test]
    fn test_serialize_traffic_request() {
        let msg = ServerMessage::TrafficRequest {
            start: vec!["ABC123".to_string(), "DEF456".to_string()],
            stop: vec!["789GHI".to_string()],
        };
        
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("\"type\":\"mlat_request\""));
        assert!(json.contains("\"start\""));
        assert!(json.contains("ABC123"));
    }
}
