// Shared constants for MLAT (match Python mlat/constants.py and config.py)

/// Speed of light in air (m/s). Python: constants.Cair
pub const CAIR: f64 = 299792458.0 / 1.00032;

/// Feet to metres. Python: constants.FTOM = 0.3038 (exact parity)
pub const FTOM: f64 = 0.3038;

/// Metres to feet. Python: constants.MTOF = 1.0/FTOM
pub const MTOF: f64 = 1.0 / FTOM;

/// m/s to knots. Python: constants.MS_TO_KTS
pub const MS_TO_KTS: f64 = 1.9438;

/// m/s to feet per minute. Python: constants.MS_TO_FPM = MTOF * 60
pub const MS_TO_FPM: f64 = MTOF * 60.0;

// --- From config.py (using FTOM for altitude) ---

/// Maximum altitude in metres. Python: config.MAX_ALT = 75000 * constants.FTOM
pub const MAX_ALT_M: f64 = 75000.0 * FTOM;

/// Minimum altitude in metres. Python: config.MIN_ALT = -1500 * constants.FTOM
pub const MIN_ALT_M: f64 = -1500.0 * FTOM;

/// Minimum interval between resolve attempts (s). Python: config.RESOLVE_INTERVAL
pub const RESOLVE_INTERVAL: f64 = 0.15;

/// Min time after successful position before next (s). Python: config.RESOLVE_BACKOFF
pub const RESOLVE_BACKOFF: f64 = 0.7;

/// Max message copies per group. Python: config.MAX_GROUP
pub const MAX_GROUP: usize = 15;

/// Max distance between even/odd DF17 positions (m). Python: config.MAX_INTERMESSAGE_RANGE
pub const MAX_INTERMESSAGE_RANGE: f64 = 10e3;

/// Solver max function evaluations. Python: config.SOLVER_MAXFEV
pub const SOLVER_MAXFEV: usize = 165;

/// Cohort delay before processing (s). Python: config.MLAT_DELAY
pub const MLAT_DELAY: f64 = 0.9;

/// Minimum NUCp to accept as sync message. Python: config.MIN_NUC
pub const MIN_NUC: u8 = 6;

// --- Server timing (align with Python coordinator, jsonclient, clocktrack) ---

/// Main coordinator interval (s): write_state + clear sync points. Python: coordinator.main_interval = 15.0
pub const MAIN_INTERVAL_SECS: u64 = 15;
/// Heartbeat write interval (s). Python: JsonClient.write_heartbeat_interval = 30.0
pub const HEARTBEAT_INTERVAL_SECS: u64 = 30;
/// No client messages for this long (s) → close connection. Python: read_heartbeat_interval = 150.0
pub const READ_TIMEOUT_SECS: u64 = 150;
/// Clock tracker cleanup interval (s). Python: clocktrack._cleanup reschedules with call_later(10.0, _cleanup)
pub const CLEANUP_INTERVAL_SECS: u64 = 10;
/// Handshake readline timeout (s). Python: asyncio.wait_for(self.r.readline(), timeout=15.0)
pub const HANDSHAKE_TIMEOUT_SECS: u64 = 15;
/// Status log interval is multiplied by this. Python: status_interval = status_interval * 0.95
pub const STATUS_INTERVAL_FACTOR: f64 = 0.95;
/// Zlib DoS mitigation: sleep (s) when decompressing more data in same packet. Python: await asyncio.sleep(0.1)
pub const ZLIB_DOS_SLEEP_SECS: f64 = 0.1;
