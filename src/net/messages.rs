//! JSON message types and wire format for the client protocol.
//!
//! **Wire format must match the Python server exactly** (python-mlat-server/mlat/jsonclient.py).
//! All server→client messages are flat JSON objects with no `"type"` field.
//!
//! Server sends:
//! - Handshake: `{ "compress", "reconnect_in", "selective_traffic", "heartbeat", "return_results", "return_stats", "rate_reports", "motd" [, "udp_transport": [host, port, key] ] }`
//! - Handshake error: `{ "deny": ["message"], "reconnect_in": N }`
//! - Heartbeat: `{ "heartbeat": { "server_time": N } }`
//! - Traffic: `{ "start_sending": ["icao6", ...] }` and/or `{ "stop_sending": ["icao6", ...] }`
//! - Result: `{ "result": { "@", "addr", "lat", "lon", "alt", "callsign", "squawk", "hdop", "vdop", "tdop", "gdop", "nstations" } }`
//!
//! Client sends (we accept): handshake (flat), then `sync`, `mlat`, `seen`, `lost`, `rate_report`, `heartbeat`, etc. (wrapped or tag format).

use std::collections::HashMap;
use serde::Deserialize;

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
    /// If true, client wants stats message every 15s (Python: return_stats).
    #[serde(default)]
    pub return_stats: Option<bool>,
    /// Client requests UDP transport when == 2 (Python: hs.get('udp_transport', 0) == 2). Server only advertises udp_transport in response when this is Some(2).
    #[serde(default)]
    pub udp_transport: Option<u32>,
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

/// Messages sent from server to client (internal enum).
/// Wire format must match Python: flat JSON objects with no "type" field.
/// See server_message_to_wire() for the exact bytes sent.
#[derive(Debug, Clone)]
pub enum ServerMessage {
    /// Handshake response (sent as flat JSON in json_client, not via this enum on wire)
    Handshake {
        motd: String,
        reconnect_in: Option<u32>,
        compress: Option<String>,
        selective_traffic: Option<bool>,
        heartbeat: Option<bool>,
        return_results: Option<bool>,
        return_stats: Option<bool>,
        rate_reports: Option<bool>,
        udp_transport: Option<(String, u16, u32)>,
    },
    /// Traffic filter: on wire sent as {"start_sending": [...]} and/or {"stop_sending": [...]} (Python keys)
    TrafficRequest {
        start: Vec<String>,
        stop: Vec<String>,
    },
    /// MLAT result (legacy path); on wire sent as {"result": {"@", "addr", "lat", ...}} (Python keys)
    MlatResult {
        icao: String,
        lat: f64,
        lon: f64,
        alt: f64,
        callsign: Option<String>,
        squawk: Option<String>,
        hdop: Option<f64>,
        vdop: Option<f64>,
        tdop: Option<f64>,
        gdop: Option<f64>,
        nstations: Option<usize>,
    },
    /// Heartbeat: on wire sent as {"heartbeat": {"server_time": ...}}
    Heartbeat { server_time: f64 },
    /// Stats (every 15s when client asked for return_stats). Python: send(stats=statistics)
    Stats {
        peer_count: usize,
        bad_sync_timeout: i64,
        outlier_percent: f64,
    },
    /// MLAT result for forward_results: on wire sent as {"result": {"@", "addr", "lat", "lon", "alt", ...}}
    Position {
        timestamp: f64,
        addr: String,
        lat: f64,
        lon: f64,
        alt: f64,
        callsign: Option<String>,
        squawk: Option<String>,
        hdop: Option<f64>,
        vdop: Option<f64>,
        tdop: Option<f64>,
        gdop: Option<f64>,
        nstations: usize,
    },
}

/// Convert ServerMessage to the exact flat JSON lines sent to the client (match Python wire format).
/// Returns one or more JSON objects (e.g. TrafficRequest can yield two lines: start_sending, stop_sending).
pub fn server_message_to_wire(msg: &ServerMessage) -> Vec<serde_json::Value> {
    use ServerMessage::*;
    let mut out = Vec::new();
    match msg {
        TrafficRequest { start, stop } => {
            if !start.is_empty() {
                out.push(serde_json::json!({ "start_sending": start }));
            }
            if !stop.is_empty() {
                out.push(serde_json::json!({ "stop_sending": stop }));
            }
        }
        Heartbeat { server_time } => {
            let t = (server_time * 1000.0).round() / 1000.0;
            out.push(serde_json::json!({ "heartbeat": { "server_time": t } }));
        }
        Stats { peer_count, bad_sync_timeout, outlier_percent } => {
            out.push(serde_json::json!({
                "stats": {
                    "peer_count": peer_count,
                    "bad_sync_timeout": bad_sync_timeout,
                    "outlier_percent": outlier_percent,
                }
            }));
        }
        Position {
            timestamp,
            addr,
            lat,
            lon,
            alt,
            callsign,
            squawk,
            hdop,
            vdop,
            tdop,
            gdop,
            nstations,
        } => {
            let result = serde_json::json!({
                "@": (timestamp * 1000.0).round() / 1000.0,
                "addr": addr,
                "lat": (lat * 100000.0).round() / 100000.0,
                "lon": (lon * 100000.0).round() / 100000.0,
                "alt": alt.round(),
                "callsign": callsign,
                "squawk": squawk,
                "hdop": hdop.unwrap_or(0.0),
                "vdop": vdop.unwrap_or(0.0),
                "tdop": tdop.unwrap_or(0.0),
                "gdop": gdop.unwrap_or(0.0),
                "nstations": nstations,
            });
            out.push(serde_json::json!({ "result": result }));
        }
        MlatResult {
            icao,
            lat,
            lon,
            alt,
            callsign,
            squawk,
            hdop,
            vdop,
            tdop,
            gdop,
            nstations,
        } => {
            let n = nstations.unwrap_or(0);
            let result = serde_json::json!({
                "@": 0_f64,
                "addr": icao,
                "lat": (lat * 100000.0).round() / 100000.0,
                "lon": (lon * 100000.0).round() / 100000.0,
                "alt": alt.round(),
                "callsign": callsign,
                "squawk": squawk,
                "hdop": hdop.unwrap_or(0.0),
                "vdop": vdop.unwrap_or(0.0),
                "tdop": tdop.unwrap_or(0.0),
                "gdop": gdop.unwrap_or(0.0),
                "nstations": n,
            });
            out.push(serde_json::json!({ "result": result }));
        }
        Handshake { .. } => {
            // Handshake is sent manually as flat JSON in process_handshake, not via channel
        }
    }
    out
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
    fn test_wire_handshake_not_sent_via_enum() {
        // Handshake is sent as flat JSON in process_handshake; server_message_to_wire returns empty for Handshake
        let msg = ServerMessage::Handshake {
            motd: "Welcome!".to_string(),
            reconnect_in: Some(300),
            compress: Some("zlib".to_string()),
            selective_traffic: Some(true),
            heartbeat: Some(true),
            return_results: Some(true),
            return_stats: Some(false),
            rate_reports: Some(true),
            udp_transport: None,
        };
        let wire = server_message_to_wire(&msg);
        assert!(wire.is_empty());
    }

    #[test]
    fn test_wire_traffic_request_flat_format() {
        let msg = ServerMessage::TrafficRequest {
            start: vec!["abc123".to_string(), "def456".to_string()],
            stop: vec!["789abc".to_string()],
        };
        let wire = server_message_to_wire(&msg);
        assert_eq!(wire.len(), 2);
        assert!(wire[0].get("start_sending").is_some());
        assert!(wire[1].get("stop_sending").is_some());
        let json0 = serde_json::to_string(&wire[0]).unwrap();
        let json1 = serde_json::to_string(&wire[1]).unwrap();
        assert!(!json0.contains("\"type\""));
        assert!(json0.contains("abc123"));
        assert!(json1.contains("789abc"));
    }

    #[test]
    fn test_wire_heartbeat_flat_format() {
        let msg = ServerMessage::Heartbeat { server_time: 123456.789 };
        let wire = server_message_to_wire(&msg);
        assert_eq!(wire.len(), 1);
        assert!(wire[0].get("heartbeat").is_some());
        assert_eq!(wire[0]["heartbeat"]["server_time"], 123456.789);
    }

    #[test]
    fn test_wire_result_flat_format() {
        let msg = ServerMessage::Position {
            timestamp: 1000.5,
            addr: "abc123".to_string(),
            lat: 37.5,
            lon: -122.0,
            alt: 1000.0,
            callsign: None,
            squawk: None,
            hdop: None,
            vdop: None,
            tdop: None,
            gdop: None,
            nstations: 4,
        };
        let wire = server_message_to_wire(&msg);
        assert_eq!(wire.len(), 1);
        let result = &wire[0]["result"];
        assert_eq!(result["@"], 1000.5);
        assert_eq!(result["addr"], "abc123");
        assert_eq!(result["nstations"], 4);
        assert!(!wire[0].to_string().contains("\"type\""));
    }
}
