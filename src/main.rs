// MLAT Server - Main Entry Point
// Copyright (C) 2024 - Rust port of mlat-server
// Licensed under AGPL v3

use mlat_server::config::Config;
use mlat_server::coordinator::Coordinator;
use mlat_server::net::listener::TcpServer;
use mlat_server::output::SbsOutput;
use std::sync::Arc;
use tokio::signal;
use clap::Parser;
use tracing::{info, error, warn};
use tower_http::services::ServeDir;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse command-line arguments
    let config = Config::parse();
    
    // Initialize logging
    init_logging(config.verbose); 
    
    info!("Starting MLAT server");
    info!("Tag: {}", config.tag);
    
    // Create coordinator with work_dir and status interval (for status logs like Python)
    let coordinator = Arc::new(Coordinator::new_with_status(
        config.work_dir.clone(),
        config.status_interval,
    ));
    coordinator.init_work_dir().await;
    info!("Coordinator initialized");

    // Optional: HTTP server to expose work directory (JSON files: sync.json, clients.json, aircraft.json)
    if let Ok(port_str) = std::env::var("HTTP_PORT") {
        if let Ok(port) = port_str.parse::<u16>() {
            let work_dir = config.work_dir.clone();
            tokio::spawn(async move {
                let listener = match tokio::net::TcpListener::bind((std::net::Ipv4Addr::UNSPECIFIED, port)).await {
                    Ok(l) => l,
                    Err(e) => {
                        error!("HTTP server failed to bind to port {}: {}", port, e);
                        return;
                    }
                };
                info!("HTTP server on port {} serving {}", port, work_dir);
                let app = axum::Router::new().nest_service("/", ServeDir::new(work_dir));
                if let Err(e) = axum::serve(listener, app).await {
                    error!("HTTP server error: {}", e);
                }
            });
        }
    }

    // Spawn periodic tasks for coordinator
    let coordinator_clone = coordinator.clone();
    tokio::spawn(async move {
        coordinator_clone.run().await;
    });
    
    // 1. Setup Client Listeners
    let mut servers: Vec<TcpServer> = Vec::new();

    if config.client_listen.is_empty() {
        warn!("No client listeners specified! Use --client-listen [host:]port");
    }

    for listen_addr in &config.client_listen {
        // Parse [host:]tcp_port[:udp_port]
        let parts: Vec<&str> = listen_addr.split(':').collect();
        
        let (host, tcp_port, udp_port) = match parts.len() {
            1 => ("0.0.0.0", parts[0], None),
            2 => {
                // Could be host:tcp or tcp:udp
                // Check if first part is numeric
                if parts[0].chars().all(|c| c.is_numeric()) {
                    ("0.0.0.0", parts[0], Some(parts[1]))
                } else {
                    (parts[0], parts[1], None)
                }
            },
            3 => (parts[0], parts[1], Some(parts[2])),
            _ => {
                error!("Invalid listen address format: {}", listen_addr);
                continue;
            }
        };

        // Start TCP Server
        let tcp_addr_str = format!("{}:{}", host, tcp_port);
        match tcp_addr_str.parse::<std::net::SocketAddr>() {
            Ok(addr) => {
                match TcpServer::start_with_coordinator(
                    addr,
                    coordinator.clone(),
                    config.motd.clone(),
                ).await {
                    Ok(server) => {
                        eprintln!("JSON client handler listening on {} (TCP)", server.addr());
                        servers.push(server);
                    }
                    Err(e) => error!("Failed to start TCP server on {}: {}", addr, e),
                }
            },
            Err(e) => error!("Invalid TCP address '{}': {}", tcp_addr_str, e),
        }

        // Start UDP Server if specified
        if let Some(udp_p) = udp_port {
            use mlat_server::net::udp::UdpServer;
            let udp_addr_str = format!("{}:{}", host, udp_p);
            match udp_addr_str.parse::<std::net::SocketAddr>() {
                Ok(addr) => {
                     match UdpServer::start(addr, coordinator.clone()).await {
                         Ok(_) => info!("UDP server listening on {}", addr),
                         Err(e) => error!("Failed to start UDP server on {}: {}", addr, e),
                     }
                },
                Err(e) => error!("Invalid UDP address '{}': {}", udp_addr_str, e),
            }
        }
    }

    // 2. Setup Outputs
    
    // Create broadcast channel for network outputs (SBS/Beast)
    // Capacity 100 messages
    let (tx, _rx) = tokio::sync::broadcast::channel(100);
    
    // Basestation Output (TCP Connect)
    for target in &config.basestation_connect {
        info!("Basestation output to {} (Connect)", target);
        let rx = tx.subscribe();
        let target_clone = target.clone();
        tokio::spawn(async move {
            mlat_server::net::output_tcp::run_tcp_connect_output(target_clone, rx, "Basestation").await;
        });
    }
    
    // Basestation Output (TCP Listen)
    for target in &config.basestation_listen {
         info!("Basestation output on {} (Listen)", target);
         let tx_clone = tx.clone();
         let target_clone = target.clone();
         tokio::spawn(async move {
             mlat_server::net::output_tcp::run_tcp_listen_output(target_clone, tx_clone, "Basestation").await;
         });
    }
    
    // Add SBS Output Handler if any network outputs are configured
    // This feeds the broadcast channel which feeds all TCP outputs
    if !config.basestation_connect.is_empty() || !config.basestation_listen.is_empty() {
        coordinator.add_output(Box::new(SbsOutput::new(Some(tx.clone())))).await;
    }
    
    // CSV Output
    for filename in &config.write_csv {
        info!("Writing CSV results to {}", filename);
        match mlat_server::output::CsvOutput::new(filename) {
            Ok(csv_out) => {
                coordinator.add_output(Box::new(csv_out)).await;
            }
            Err(e) => {
                error!("Failed to open CSV output file {}: {}", filename, e);
            }
        }
    }

    info!("Server ready");
    
    // Wait for shutdown signal (Ctrl+C)
    match signal::ctrl_c().await {
        Ok(()) => {
            info!("Received shutdown signal (Ctrl+C)");
        }
        Err(err) => {
            error!("Unable to listen for shutdown signal: {}", err);
            return Err(err.into());
        }
    }
    
    // Graceful shutdown
    info!("Shutting down...");
    for mut server in servers {
        server.shutdown().await;
    }
    
    // Report final statistics
    let receiver_count = coordinator.receiver_count().await;
    info!("Server stopped. Final receiver count: {}", receiver_count);
    
    Ok(())
}

/// Initialize logging subsystem
fn init_logging(verbose: bool) {
    use tracing_subscriber::fmt::format::FmtSpan;
    
    let subscriber = tracing_subscriber::fmt()
        .with_target(false)
        .with_thread_ids(false)
        .with_level(true)
        .with_span_events(if verbose {
            FmtSpan::ENTER | FmtSpan::CLOSE
        } else {
            FmtSpan::NONE
        });
    
    if verbose {
        subscriber
            .with_max_level(tracing::Level::DEBUG)
            .init();
        info!("Verbose logging enabled (DEBUG level)");
    } else {
        subscriber
            .with_max_level(tracing::Level::INFO)
            .init();
    }
}
