// TCP connection handler
// Manages individual client connections

use tokio::net::TcpStream;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader, BufWriter};
use std::net::SocketAddr;
use std::io;

/// Represents a single TCP connection to a client
pub struct Connection {
    reader: BufReader<tokio::io::ReadHalf<TcpStream>>,
    writer: BufWriter<tokio::io::WriteHalf<TcpStream>>,
    peer_addr: SocketAddr,
    receiver_id: Option<usize>,
}

impl Connection {
    /// Create a new connection from a TCP stream
    pub fn new(stream: TcpStream, peer_addr: SocketAddr) -> Self {
        let (read_half, write_half) = tokio::io::split(stream);
        
        Connection {
            reader: BufReader::new(read_half),
            writer: BufWriter::new(write_half),
            peer_addr,
            receiver_id: None,
        }
    }
    
    /// Get the peer address
    pub fn peer_addr(&self) -> SocketAddr {
        self.peer_addr
    }
    
    /// Get the receiver ID (if authenticated)
    pub fn receiver_id(&self) -> Option<usize> {
        self.receiver_id
    }
    
    /// Set the receiver ID (after authentication)
    pub fn set_receiver_id(&mut self, id: usize) {
        self.receiver_id = Some(id);
    }
    
    /// Read exactly `buf.len()` bytes (for zlib packet framing)
    pub async fn read_exact(&mut self, buf: &mut [u8]) -> io::Result<()> {
        let n = self.reader.read_exact(buf).await?;
        if n != buf.len() {
            return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "read_exact: short read"));
        }
        Ok(())
    }

    /// Read a line from the connection (JSON messages are line-delimited)
    pub async fn read_line(&mut self) -> io::Result<String> {
        let mut line = String::new();
        self.reader.read_line(&mut line).await?;
        
        // Remove trailing newline
        if line.ends_with('\n') {
            line.pop();
            if line.ends_with('\r') {
                line.pop();
            }
        }
        
        Ok(line)
    }
    
    /// Write a JSON value to the connection
    pub async fn write_json(&mut self, value: &serde_json::Value) -> io::Result<()> {
        let json_str = serde_json::to_string(value)?;
        self.writer.write_all(json_str.as_bytes()).await?;
        self.writer.write_all(b"\n").await?;
        self.writer.flush().await?;
        Ok(())
    }
    
    /// Write raw bytes to the connection
    pub async fn write_bytes(&mut self, data: &[u8]) -> io::Result<()> {
        self.writer.write_all(data).await?;
        self.writer.flush().await?;
        Ok(())
    }

    /// Write a zlib2 frame: 2-byte big-endian length then payload (Python _flush_zlib format).
    /// Caller must provide the compressed payload without the 4-byte Z_SYNC_FLUSH suffix.
    pub async fn write_zlib2_frame(&mut self, payload: &[u8]) -> io::Result<()> {
        let len = payload.len();
        if len > 65535 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "zlib2 frame payload exceeds 65535 bytes",
            ));
        }
        let len_be = (len as u16).to_be_bytes();
        self.writer.write_all(&len_be).await?;
        self.writer.write_all(payload).await?;
        self.writer.flush().await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_connection_creation() {
        // Can't easily test without actual TCP connection
        // Will test in integration tests
    }
}
