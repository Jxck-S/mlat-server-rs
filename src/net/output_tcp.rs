use tokio::net::{TcpListener, TcpStream};
use tokio::io::AsyncWriteExt;
use tokio::sync::broadcast;
use std::net::SocketAddr;
use tracing::{info, error, warn};
use std::time::Duration;

/// Output modes
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OutputMode {
    Connect,
    Listen,
}

/// Handle output to a TCP server (Connect mode)
pub async fn run_tcp_connect_output(
    addr: String,
    mut rx: broadcast::Receiver<Vec<u8>>, // Receives raw bytes
    output_type: &str,
) {
    info!("Starting {} output client to {}", output_type, addr);

    loop {
        match TcpStream::connect(&addr).await {
            Ok(mut stream) => {
                info!("Connected to {} output at {}", output_type, addr);
                let heartbeat = Duration::from_secs(crate::constants::HEARTBEAT_INTERVAL_SECS);
                let mut interval = tokio::time::interval(heartbeat);
                interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

                loop {
                    tokio::select! {
                        result = rx.recv() => {
                            match result {
                                Ok(msg) => {
                                    if let Err(e) = stream.write_all(&msg).await {
                                        error!("Failed to write to {}: {}", addr, e);
                                        break;
                                    }
                                    if let Err(e) = stream.flush().await {
                                        error!("Failed to flush to {}: {}", addr, e);
                                        break;
                                    }
                                }
                                Err(broadcast::error::RecvError::Lagged(count)) => {
                                    warn!("Output client lagged by {} messages", count);
                                }
                                Err(broadcast::error::RecvError::Closed) => {
                                    info!("Output channel closed");
                                    return;
                                }
                            }
                        }
                        _ = interval.tick() => {
                            if let Err(e) = stream.write_all(b"\n").await {
                                error!("Failed to send heartbeat to {}: {}", addr, e);
                                break;
                            }
                            if let Err(e) = stream.flush().await {
                                error!("Failed to flush heartbeat to {}: {}", addr, e);
                                break;
                            }
                        }
                    }
                }
            }
            Err(e) => {
                warn!("Failed to connect to {} output at {}: {}", output_type, addr, e);
            }
        }
        
        // Wait before reconnecting
        tokio::time::sleep(Duration::from_secs(5)).await;
    }
}

/// Handle output from TCP clients (Listen mode)
pub async fn run_tcp_listen_output(
    addr: String,
    rx: broadcast::Sender<Vec<u8>>, // We use Sender to subscribe new clients
    output_type: &str,
) {
    // Parse address
    let socket_addr: SocketAddr = match addr.parse() {
        Ok(a) => a,
        Err(e) => {
             error!("Invalid address {}: {}", addr, e);
             return;
        }
    };

    match TcpListener::bind(socket_addr).await {
        Ok(listener) => {
            info!("Listening for {} output clients on {}", output_type, addr);
            
            loop {
                match listener.accept().await {
                    Ok((mut stream, peer_addr)) => {
                        info!("Accepted {} client connection from {}", output_type, peer_addr);
                        let mut client_rx = rx.subscribe();
                        
                        // Spawn task for this client (heartbeat every 30s like Python BasestationClient)
                        tokio::spawn(async move {
                            let heartbeat = Duration::from_secs(crate::constants::HEARTBEAT_INTERVAL_SECS);
                            let mut interval = tokio::time::interval(heartbeat);
                            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

                            loop {
                                tokio::select! {
                                    result = client_rx.recv() => {
                                        match result {
                                            Ok(msg) => {
                                                if let Err(e) = stream.write_all(&msg).await {
                                                    info!("Client {} disconnected: {}", peer_addr, e);
                                                    break;
                                                }
                                                if let Err(e) = stream.flush().await {
                                                    info!("Client {} flush error: {}", peer_addr, e);
                                                    break;
                                                }
                                            }
                                            Err(broadcast::error::RecvError::Lagged(_)) => {}
                                            Err(broadcast::error::RecvError::Closed) => break,
                                        }
                                    }
                                    _ = interval.tick() => {
                                        if let Err(e) = stream.write_all(b"\n").await {
                                            info!("Client {} heartbeat write error: {}", peer_addr, e);
                                            break;
                                        }
                                        if let Err(e) = stream.flush().await {
                                            info!("Client {} heartbeat flush error: {}", peer_addr, e);
                                            break;
                                        }
                                    }
                                }
                            }
                        });
                    }
                    Err(e) => {
                        error!("Accept failed: {}", e);
                    }
                }
            }
        }
        Err(e) => {
             error!("Failed to bind {} output listener on {}: {}", output_type, addr, e);
        }
    }
}
