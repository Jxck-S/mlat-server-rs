use clap::Parser;

/// MLAT Server Configuration
#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
pub struct Config {
    /// Listen on [host:]tcp_port[:udp_port] for connections from multilateration clients.
    #[arg(long, value_name = "ADDR")]
    pub client_listen: Vec<String>,

    /// Set the server MOTD sent to clients.
    #[arg(long, default_value = "")]
    pub motd: String,

    /// Write results in CSV format to a local file.
    #[arg(long, value_name = "FILE")]
    pub write_csv: Vec<String>,

    /// Connect to a host:port and send Basestation-format results.
    #[arg(long, value_name = "HOST:PORT")]
    pub basestation_connect: Vec<String>,

    /// Listen on a [host:]port and send Basestation-format results to clients that connect.
    #[arg(long, value_name = "ADDR")]
    pub basestation_listen: Vec<String>,

    /// Connect to a host:port and send Basestation-format results (filtered).
    #[arg(long, value_name = "HOST:PORT")]
    pub filtered_basestation_connect: Vec<String>,

    /// Listen on a [host:]port and send Basestation-format results (filtered) to clients that connect.
    #[arg(long, value_name = "ADDR")]
    pub filtered_basestation_listen: Vec<String>,

    /// Directory for debug/stats output and blacklist
    #[arg(long, value_name = "DIR")]
    pub work_dir: String,

    /// Run periodic memory leak checks (requires objgraph package in Python, mostly useful for debugging)
    #[arg(long, default_value_t = false)]
    pub check_leaks: bool,

    /// Dump pseudorange data in json format to a file
    #[arg(long, value_name = "FILE")]
    pub dump_pseudorange: Option<String>,

    /// Enable partitioning (n/count)
    #[arg(long, value_name = "N/COUNT", default_value = "1/1")]
    pub partition: String,

    /// Set process name prefix
    #[arg(long, default_value = "mlat-server")]
    pub tag: String,

    /// Status logging interval in seconds, multiple of 15, -1 to disable
    #[arg(long, default_value_t = 15)]
    pub status_interval: i32,

    /// Verbose logging (DEBUG level)
    #[arg(long, short, default_value_t = false)]
    pub verbose: bool,
}

// Helper functions for parsing specific formats if needed, or implement TryFrom/FromStr on custom types.
// For now, we'll keep them as Strings in Config and parse them in main/coordinator or helper functions.
