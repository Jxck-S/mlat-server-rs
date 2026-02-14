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
