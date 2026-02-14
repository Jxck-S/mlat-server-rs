use std::time::UNIX_EPOCH;
use tokio::sync::broadcast;
use crate::mlattrack::MlatResult;

/// Trait for output handlers
pub trait OutputHandler: Send + Sync {
    /// Handle a new MLAT result
    fn handle_result(&mut self, result: &MlatResult);
}

/// Beast binary format output
pub struct BeastOutput {
    tx: Option<broadcast::Sender<Vec<u8>>>,
}

impl BeastOutput {
    pub fn new(tx: Option<broadcast::Sender<Vec<u8>>>) -> Self {
        BeastOutput { tx }
    }

    /// Encode MLAT result as a Series 2 DF18 extended squitter message in AVR/Beast format
    fn encode(&self, result: &MlatResult) -> Vec<u8> {
        // 1. Create synthetic DF18 extended squitter (TIS-B)
        let mut msg = [0u8; 14];
        
        // DF=18, CF=0 (ADS-B), AA=ICAO
        let icao = result.icao;
        msg[0] = 0x90 | ((icao >> 16) as u8 & 0x07); // DF=18 (10010), CA=0 (000) -> 10010000 = 0x90
        msg[1] = (icao >> 16) as u8;
        msg[2] = (icao >> 8) as u8;
        msg[3] = icao as u8;
        
        // Construct a Beast frame around the raw timestamp
        let mut frame = Vec::with_capacity(20);
        frame.push(0x1a); // Escape
        frame.push(0x32); // Mode 2 (Mode S Short) or 3 (Long)
        
        // 12MHz timestamp (6 bytes)
        let ticks = (result.timestamp * 12e6) as u64;
        frame.push((ticks >> 40) as u8);
        frame.push((ticks >> 32) as u8);
        frame.push((ticks >> 24) as u8);
        frame.push((ticks >> 16) as u8);
        frame.push((ticks >> 8) as u8);
        frame.push(ticks as u8);
        
        frame.push(0xFF); // Signal level (max)
        
        // Payload (Mode S Message)
        frame.extend_from_slice(&msg);
        
        frame
    }
}

impl OutputHandler for BeastOutput {
    fn handle_result(&mut self, result: &MlatResult) {
        let bytes = self.encode(result);
        if let Some(tx) = &self.tx {
            let _ = tx.send(bytes);
        }
    }
}

/// SBS (Basestation) format output
pub struct SbsOutput {
    tx: Option<broadcast::Sender<Vec<u8>>>,
}

impl SbsOutput {
    pub fn new(tx: Option<broadcast::Sender<Vec<u8>>>) -> Self {
        SbsOutput { tx }
    }
    
    fn format_date_time(timestamp: f64) -> (String, String) {
        let secs = timestamp as i64;
        let nanos = ((timestamp - secs as f64) * 1e9) as u32;
        if let Some(tm) = UNIX_EPOCH.checked_add(std::time::Duration::new(secs as u64, nanos)) {
             // Basic implementation using time crate or similar would be better
             // but for minimal dependencies, we assume UTC
             let datetime = chrono::DateTime::<chrono::Utc>::from(tm);
             return (
                 datetime.format("%Y/%m/%d").to_string(), 
                 datetime.format("%H:%M:%S.%3f").to_string()
             );
        }
        ("".to_string(), "".to_string())
    }
}

impl OutputHandler for SbsOutput {
    fn handle_result(&mut self, result: &MlatResult) {
        // MSG,3,1,1,Hex,1,Date,Time,Date,Time,,Alt,,,Lat,Lon,,,0,0,0,0
        
        let (date, time) = Self::format_date_time(result.timestamp);
        
        let (lat, lon, alt) = crate::geodesy::ecef2llh(
            result.position[0],
            result.position[1],
            result.position[2]
        );
        
        let sbs = format!(
            "MSG,3,1,1,{:06X},1,{},{},{},{},,{},,,{:.5},{:.5},,,0,0,0,0\r\n",
            result.icao,
            date, time, date, time,
            (alt * 3.28084) as i32, // Altitude in feet
            lat,
            lon
        );

        if let Some(tx) = &self.tx {
            let _ = tx.send(sbs.into_bytes());
        }
    }
}

/// CSV format output
pub struct CsvOutput {
    writer: Option<std::sync::Arc<std::sync::Mutex<std::io::BufWriter<std::fs::File>>>>,
}

impl CsvOutput {
    pub fn new(path: &str) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)?;
        let writer = std::io::BufWriter::new(file);
        Ok(CsvOutput {
            writer: Some(std::sync::Arc::new(std::sync::Mutex::new(writer))),
        })
    }
}

impl OutputHandler for CsvOutput {
    fn handle_result(&mut self, result: &MlatResult) {
        if let Some(writer) = &self.writer {
            if let Ok(mut w) = writer.lock() {
                use std::io::Write;
                let (lat, lon, alt) = crate::geodesy::ecef2llh(
                    result.position[0],
                    result.position[1],
                    result.position[2]
                );
                
                // Format: timestamp,icao,lat,lon,alt_ft
                if let Err(e) = writeln!(
                    w,
                    "{:.6},{:06X},{:.5},{:.5},{:.0}",
                    result.timestamp,
                    result.icao,
                    lat,
                    lon,
                    alt * 3.28084
                ) {
                    eprintln!("Failed to write CSV: {}", e);
                }
            }
        }
    }
}

/// JSON format output
pub struct JsonOutput {
    // tx: Option<broadcast::Sender<Vec<u8>>>, // Not used yet in main
}

impl JsonOutput {
    pub fn new() -> Self {
        JsonOutput {}
    }
    
    // Helper functionality for testing
    pub fn format_json(result: &MlatResult) -> String {
         let (lat, lon, alt) = crate::geodesy::ecef2llh(
            result.position[0],
            result.position[1],
            result.position[2]
        );
        
        format!(
            "{{\"icao\": \"{:06X}\", \"lat\": {:.5}, \"lon\": {:.5}, \"alt\": {:.0}, \"ts\": {:.6}}}",
            result.icao, lat, lon, alt * 3.28084, result.timestamp
        )
    }
}

impl OutputHandler for JsonOutput {
    fn handle_result(&mut self, result: &MlatResult) {
        let json = Self::format_json(result);
        // Currently implies logging or future usage
        let _ = json;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_beast_output_encode() {
        let output = BeastOutput::new(None);
        let result = MlatResult {
            icao: 0x4840D6,
            position: [0.0, 0.0, 0.0], // ECEF
            timestamp: 100.0,
            covariance: None,
            distinct: 4,
            dof: 3,
            error: 10.0,
            receivers: vec![1, 2, 3, 4],
        };
        
        let bytes = output.encode(&result);
        assert_eq!(bytes[0], 0x1a); 
        // ... (rest of checks)
    }

    #[test]
    fn test_sbs_format_date_time() {
        // Need chrono
        let (date, time) = SbsOutput::format_date_time(1672531200.0); // 2023-01-01 00:00:00 UTC
        assert_eq!(date, "2023/01/01");
        assert_eq!(time, "00:00:00.000");
    }
    
    #[test]
    fn test_json_formatting() {
        let ecef = crate::geodesy::llh2ecef(37.5, -122.0, 1000.0);
        let result = MlatResult {
            icao: 0x4840D6,
            position: [ecef.0, ecef.1, ecef.2], 
            timestamp: 100.0,
            covariance: None,
            distinct: 4,
            dof: 3,
            error: 10.0,
            receivers: vec![1, 2, 3, 4],
        };
        
        let json = JsonOutput::format_json(&result);
        assert!(json.contains("\"icao\": \"4840D6\""));
    }
}
