use std::time::UNIX_EPOCH;
use tokio::sync::broadcast;
use crate::constants::{MTOF, MS_TO_KTS, MS_TO_FPM};
use crate::mlattrack::MlatResult;

/// Optional aircraft-derived context for output (callsign, squawk, altitude, vrate).
/// Matches Python BasestationClient's use of coordinator.tracker.aircraft[address].
#[derive(Clone, Default)]
pub struct OutputContext {
    pub callsign: Option<String>,
    pub squawk: Option<u16>,
    pub altitude_ft: Option<f64>,
    pub last_altitude_time: Option<f64>,
    pub vrate_fpm: Option<i32>,
    pub vrate_time: Option<f64>,
}

/// CSV-quote a string for SBS/CSV (Python output.csv_quote)
fn csv_quote(s: &str) -> String {
    if s.find('\n').is_none() && !s.contains('"') && !s.contains(',') {
        s.to_string()
    } else {
        format!("\"{}\"", s.replace('"', "\"\""))
    }
}

/// Trait for output handlers
pub trait OutputHandler: Send + Sync {
    /// Handle a new MLAT result (context from aircraft when available)
    fn handle_result(&mut self, result: &MlatResult, context: Option<&OutputContext>);
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
    fn handle_result(&mut self, result: &MlatResult, context: Option<&OutputContext>) {
        // Python BasestationClient TEMPLATE: MSG,mtype,1,1,addr,1,rcv_date,rcv_time,now_date,now_time,callsign,altitude,speed,heading,lat,lon,vrate,squawk,fs,emerg,ident,aog
        let (rcv_date, rcv_time) = Self::format_date_time(result.timestamp);
        let now = std::time::SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(result.timestamp);
        let (now_date, now_time) = Self::format_date_time(now);

        let (lat, lon, _alt_ecef) = crate::geodesy::ecef2llh(
            result.position[0],
            result.position[1],
            result.position[2],
        );
        // Python: round(lat, 6), round(lon, 6)
        let lat = (lat * 1e6).round() / 1e6;
        let lon = (lon * 1e6).round() / 1e6;

        let callsign = context
            .and_then(|c| c.callsign.as_deref())
            .unwrap_or("");
        let callsign = csv_quote(callsign);

        // Python: never use MLAT altitude; use ac.altitude only when last_altitude_time within 5s
        let altitude = match context {
            Some(c) if c.last_altitude_time.map_or(false, |t| result.timestamp - t < 5.0) => {
                c.altitude_ft.map(|a| (a as i32).to_string()).unwrap_or_default()
            }
            _ => String::new(),
        };

        let speed_str = result
            .ground_speed
            .map(|v| (v * MS_TO_KTS).round() as i32)
            .map(|v| v.to_string())
            .unwrap_or_default();

        // vrate: Python sets ac.vrate when vrate_time within 5s, else vrate = '' (Kalman vrate is overwritten)
        let vrate_str = context
            .and_then(|c| {
                c.vrate_time
                    .filter(|&t| result.timestamp - t < 5.0)
                    .and_then(|_| c.vrate_fpm.map(|v| v.to_string()))
            })
            .unwrap_or_default();

        let squawk = context
            .and_then(|c| c.squawk)
            .map(|s| csv_quote(&format!("{:04o}", s)))
            .unwrap_or_else(|| csv_quote(""));

        let fs = result.receivers.len();
        let emerg = if result.error >= 0.0 {
            (result.error as i64).to_string()
        } else {
            String::new()
        };

        // ident and aog empty per Python TEMPLATE
        let sbs = format!(
            "MSG,3,1,1,{:06X},1,{},{},{},{},{},{},{},,{:.6},{:.6},{},{},{},{},,\n",
            result.icao,
            rcv_date,
            rcv_time,
            now_date,
            now_time,
            callsign,
            altitude,
            speed_str,
            "", // heading (Python uses Kalman when use_kalman_data; we don't have heading in MlatResult)
            lat,
            lon,
            vrate_str,
            squawk,
            fs,
            emerg,
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
    fn handle_result(&mut self, result: &MlatResult, _context: Option<&OutputContext>) {
        if let Some(writer) = &self.writer {
            if let Ok(mut w) = writer.lock() {
                use std::io::Write;
                let (lat, lon, alt) = crate::geodesy::ecef2llh(
                    result.position[0],
                    result.position[1],
                    result.position[2]
                );
                let speed_kts = result.ground_speed.map(|v| (v * MS_TO_KTS).round() as i32);
                let vrate_fpm = result.vertical_speed.map(|v| (v * MS_TO_FPM).round() as i32);
                // Format: timestamp,icao,lat,lon,alt_ft,speed_kts,vrate_fpm (Python: speed/vrate from Kalman)
                if let Err(e) = writeln!(
                    w,
                    "{:.6},{:06X},{:.5},{:.5},{:.0},{},{}",
                    result.timestamp,
                    result.icao,
                    lat,
                    lon,
                    alt * MTOF,
                    speed_kts.map(|v| v.to_string()).unwrap_or_default(),
                    vrate_fpm.map(|v| v.to_string()).unwrap_or_default()
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
            result.icao, lat, lon, alt * MTOF, result.timestamp
        )
    }
}

impl OutputHandler for JsonOutput {
    fn handle_result(&mut self, result: &MlatResult, _context: Option<&OutputContext>) {
        let json = Self::format_json(result);
        // Currently implies logging or future usage
        let _ = json;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
            ground_speed: None,
            vertical_speed: None,
        };
        
        let json = JsonOutput::format_json(&result);
        assert!(json.contains("\"icao\": \"4840D6\""));
    }
}
