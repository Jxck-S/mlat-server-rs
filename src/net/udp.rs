use tokio::net::UdpSocket;
use std::net::SocketAddr;
use std::sync::Arc;
use std::io;
use tracing::{info, debug, error};
use crate::coordinator::Coordinator;
use crate::net::messages::ClientMessage;
use tokio::sync::mpsc;

/// UDP Server for handling high-throughput client messages
pub struct UdpServer {
    socket: Arc<UdpSocket>,
    shutdown_tx: mpsc::Sender<()>,
}

impl UdpServer {
    /// Bind to address and start listening
    pub async fn start(addr: SocketAddr, coordinator: Arc<Coordinator>) -> io::Result<Self> {
        let socket = UdpSocket::bind(addr).await?;
        let socket = Arc::new(socket);
        let (shutdown_tx, mut shutdown_rx) = mpsc::channel(1);
        
        // We don't store coordinator in struct as we don't access it later
        let server = UdpServer {
            socket: socket.clone(),
            shutdown_tx,
        };

        let coord = coordinator.clone();
        tokio::spawn(async move {
            let mut buf = [0u8; 65535]; // Max UDP size
            loop {
                tokio::select! {
                    res = socket.recv_from(&mut buf) => {
                        match res {
                            Ok((size, peer)) => {
                                let data = &buf[..size];
                                Self::process_packet(data, peer, &coord).await;
                            }
                            Err(e) => {
                                error!("UDP receive error: {}", e);
                            }
                        }
                    }
                    _ = shutdown_rx.recv() => {
                        info!("UDP server shutting down");
                        break;
                    }
                }
            }
        });

        info!("UDP server listening on {}", addr);
        Ok(server)
    }

    /// Shutdown the server
    pub async fn shutdown(&self) {
        let _ = self.shutdown_tx.send(()).await;
    }
    
    /// Get bound address
    pub fn addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Process a single UDP packet
    async fn process_packet(data: &[u8], peer: SocketAddr, _coordinator: &Arc<Coordinator>) {
        // Try parsing as JSON ClientMessage
        match serde_json::from_slice::<ClientMessage>(data) {
            Ok(msg) => {
                debug!("Received UDP message from {}: {:?}", peer, msg);
                // Route to coordinator
                match msg {
                    ClientMessage::Mlat { timestamp: _, message, utc: _, .. } => {
                        // UDP packets might not have UUID, we use IP as imperfect ID or 0
                        // Since UDP is connectionless, mapping to receiver ID is tricky without handshake
                        // For now we might look up by IP if we had a map, or use a specific UDP receiver ID
                        // The Python code implies UDP clients might be "anonymous" or shared ID?
                        // Or maybe they use handshake via UDP (unlikely)?
                        // Let's assume receiver_id 0 or lookup from map if we implemented IP map.
                        // For this audit fix, we'll log:
                        debug!("UDP MLAT from {}: {}", peer, message);
                        // _coordinator.handle_mlat(0, timestamp, &message, utc.unwrap_or(0.0)).await;
                    },
                    ClientMessage::Sync { even_timestamp: _, odd_timestamp: _, even_message, odd_message, .. } => {
                        debug!("UDP Sync from {}: {}/{}", peer, even_message, odd_message);
                        // _coordinator.handle_sync(0, even_timestamp, odd_timestamp, &even_message, &odd_message).await;
                    }
                    _ => {}
                }
            }
            Err(e) => {
                debug!("Failed to parse UDP packet from {}: {}", peer, e);
            }
        }
    }
}
