// TCP listener and server
// Manages incoming client connections

use tokio::net::TcpListener;
use tokio::sync::mpsc;
use std::net::SocketAddr;
use std::io;
use std::sync::Arc;

use super::connection::Connection;

/// TCP server that accepts and manages client connections
pub struct TcpServer {
    addr: SocketAddr,
    shutdown_tx: Option<mpsc::Sender<()>>,
}

impl TcpServer {
    /// Start a new TCP server on the given address
    ///
    /// # Arguments
    /// * `addr` - Socket address to bind to
    /// * `handler` - Function to handle each new connection
    ///
    /// # Returns
    /// A TcpServer instance that can be used to manage the server
    pub async fn start<F>(addr: SocketAddr, handler: F) -> io::Result<Self>
    where
        F: Fn(Connection) + Send + Sync + 'static,
    {
        let listener = TcpListener::bind(addr).await?;
        let actual_addr = listener.local_addr()?;
        
        let (shutdown_tx, mut shutdown_rx) = mpsc::channel::<()>(1);
        let handler = Arc::new(handler);
        
        // Spawn the accept loop
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    result = listener.accept() => {
                        match result {
                            Ok((stream, peer_addr)) => {
                                let handler = Arc::clone(&handler);
                                tokio::spawn(async move {
                                    let connection = Connection::new(stream, peer_addr);
                                    handler(connection);
                                });
                            }
                            Err(e) => {
                                eprintln!("Accept error: {}", e);
                            }
                        }
                    }
                    _ = shutdown_rx.recv() => {
                        println!("TCP server shutting down");
                        break;
                    }
                }
            }
        });
        
        eprintln!("JSON client handler listening on {} (TCP)", actual_addr);
        
        Ok(TcpServer {
            addr: actual_addr,
            shutdown_tx: Some(shutdown_tx),
        })
    }
    
    /// Get the address the server is listening on
    pub fn addr(&self) -> SocketAddr {
        self.addr
    }
    
    /// Shutdown the server
    pub async fn shutdown(&mut self) {
        if let Some(tx) = self.shutdown_tx.take() {
            let _ = tx.send(()).await;
        }
    }
    
    /// Start server with coordinator integration
    pub async fn start_with_coordinator(
        addr: SocketAddr,
        coordinator: std::sync::Arc<crate::coordinator::Coordinator>,
        motd: String,
    ) -> io::Result<Self> {
        let listener = TcpListener::bind(addr).await?;
        let local_addr = listener.local_addr()?;
        let (shutdown_tx, mut shutdown_rx) = tokio::sync::mpsc::channel::<()>(1);
        
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    result = listener.accept() => {
                        match result {
                            Ok((stream, peer_addr)) => {
                                let coordinator = std::sync::Arc::clone(&coordinator);
                                let motd = motd.clone();
                                tokio::spawn(async move {
                                    let connection = super::connection::Connection::new(stream, peer_addr);
                                    let mut client = super::json_client::JsonClient::new(connection, motd, coordinator);
                                    if let Err(e) = client.run().await {
                                        eprintln!("Client error from {}: {}", peer_addr, e);
                                    }
                                });
                            }
                            Err(e) => eprintln!("Accept error: {}", e),
                        }
                    }
                    _ = shutdown_rx.recv() => break,
                }
            }
        });
        
        Ok(TcpServer {
            addr: local_addr,
            shutdown_tx: Some(shutdown_tx),
        })
    }
}

impl Drop for TcpServer {
    fn drop(&mut self) {
        // Trigger shutdown on drop
        if let Some(tx) = self.shutdown_tx.take() {
            let _ = tx.try_send(());
        }
    }
    
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    
    #[tokio::test]
    async fn test_tcp_server_start() {
        let addr: SocketAddr = "127.0.0.1:0".parse().unwrap();
        
        let server = TcpServer::start(addr, |_conn| {
            // Simple handler that does nothing
        }).await.unwrap();
        
        // Server should be listening
        assert!(server.addr().port() > 0);
    }
    
    #[tokio::test]
    async fn test_tcp_server_accept_connection() {
        let addr: SocketAddr = "127.0.0.1:0".parse().unwrap();
        
        let (tx, mut rx) = mpsc::channel(1);
        
        let server = TcpServer::start(addr, move |_conn| {
            let _ = tx.try_send(());
        }).await.unwrap();
        
        let server_addr = server.addr();
        
        // Connect a client
        let _client = TcpStream::connect(server_addr).await.unwrap();
        
        // Wait for handler to be called
        tokio::time::timeout(
            std::time::Duration::from_secs(1),
            rx.recv()
        ).await.unwrap();
    }
}
