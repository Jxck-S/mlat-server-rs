use clap::Parser;

/// AGPL v3: URL where this server's source code is available (Python: config.AGPL_SERVER_CODE_URL).
/// Must point to the code this server is running. See COPYING and README.
pub const AGPL_SERVER_CODE_URL: &str = "https://github.com/wiedehopf/mlat-server";

/// Build MOTD string sent to clients: user motd + AGPL notice and source URL (Python: expanded_motd).
pub fn expanded_motd(motd: &str) -> String {
    format!(
        "\n\n{}\n\nThe multilateration server source code is available under\
         \nthe terms of the Affero GPL (v3 or later). You may obtain\
         \na copy of this server's source code at the following\
         \nlocation: {}",
        motd, AGPL_SERVER_CODE_URL
    )
}

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

    /// Do not advertise UDP in handshake; clients will send sync/mlat over TCP. Use when only TCP is forwarded (e.g. SSH reverse tunnel).
    #[arg(long, default_value_t = false)]
    pub no_udp: bool,

    /// Advertise UDP transport in handshake when a UDP port is bound (--client-listen host:tcp:udp). By default UDP is not advertised so sync/mlat use TCP and work behind tunnels; set this to allow clients to send over UDP.
    #[arg(long, default_value_t = false)]
    pub advertise_udp: bool,
}

// Helper functions for parsing specific formats if needed, or implement TryFrom/FromStr on custom types.
// For now, we'll keep them as Strings in Config and parse them in main/coordinator or helper functions.
