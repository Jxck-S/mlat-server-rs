// UDP server for packed sync/mlat protocol (Python PackedMlatServerProtocol).
// Clients send binary packets: header (key, seq, base) then type byte + payload(s).
// We look up receiver_id by key (from handshake udp_transport) and route to coordinator.

use tokio::net::UdpSocket;
use std::net::SocketAddr;
use std::sync::Arc;
use std::io;
use tracing::{info, debug, error};
use crate::coordinator::Coordinator;
use tokio::sync::mpsc;

/// Packed protocol type bytes (Python PackedMlatServerProtocol)
const TYPE_SYNC: u8 = 1;
const TYPE_MLAT_SHORT: u8 = 2;
const TYPE_MLAT_LONG: u8 = 3;
const TYPE_SSYNC: u8 = 4;
const TYPE_REBASE: u8 = 5;
const TYPE_ABS_SYNC: u8 = 6;

const HEADER_LEN: usize = 14; // >IHQ = 4+2+8
const SYNC_LEN: usize = 36;   // >ii14s14s
const MLAT_SHORT_LEN: usize = 11; // >i7s
const MLAT_LONG_LEN: usize = 18;  // >i14s
const REBASE_LEN: usize = 8;      // >Q
const ABS_SYNC_LEN: usize = 44;   // >QQ14s14s

/// UDP Server for handling high-throughput client messages (packed binary protocol)
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

        let server = UdpServer {
            socket: socket.clone(),
            shutdown_tx,
        };

        let coord = coordinator.clone();
        tokio::spawn(async move {
            let mut buf = [0u8; 65535];
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

    pub async fn shutdown(&self) {
        let _ = self.shutdown_tx.send(()).await;
    }

    pub fn addr(&self) -> io::Result<SocketAddr> {
        self.socket.local_addr()
    }

    /// Parse packed UDP packet (Python STRUCT_HEADER + type bytes). Look up receiver by key and dispatch sync/mlat.
    async fn process_packet(data: &[u8], peer: SocketAddr, coordinator: &Arc<Coordinator>) {
        if data.len() < HEADER_LEN {
            return;
        }
        let key = u32::from_be_bytes(data[0..4].try_into().unwrap());
        let _seq = u16::from_be_bytes(data[4..6].try_into().unwrap());
        let mut base = u64::from_be_bytes(data[6..14].try_into().unwrap());

        let receiver_id = match coordinator.get_receiver_id_for_udp_key(key).await {
            Some(rid) => rid,
            None => {
                debug!("UDP packet from {}: unknown key {}", peer, key);
                return;
            }
        };

        let utc = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        let mut i = HEADER_LEN;
        while i < data.len() {
            let type_byte = data[i];
            i += 1;

            match type_byte {
                TYPE_SYNC => {
                    if i + SYNC_LEN > data.len() {
                        break;
                    }
                    let et = i32::from_be_bytes(data[i..i + 4].try_into().unwrap());
                    let ot = i32::from_be_bytes(data[i + 4..i + 8].try_into().unwrap());
                    let em = &data[i + 8..i + 22];   // 14 bytes
                    let om = &data[i + 22..i + 36];   // 14 bytes
                    i += SYNC_LEN;
                    let even_ts = (base as i64 + et as i64) as f64;
                    let odd_ts = (base as i64 + ot as i64) as f64;
                    let even_hex = hex::encode(em);
                    let odd_hex = hex::encode(om);
                    coordinator.handle_sync(receiver_id, even_ts, odd_ts, &even_hex, &odd_hex).await;
                }
                TYPE_MLAT_SHORT => {
                    if i + MLAT_SHORT_LEN > data.len() {
                        break;
                    }
                    let t = i32::from_be_bytes(data[i..i + 4].try_into().unwrap());
                    let m = &data[i + 4..i + 11]; // 7 bytes
                    i += MLAT_SHORT_LEN;
                    let timestamp = (base as i64 + t as i64) as f64;
                    let message_hex = hex::encode(m);
                    coordinator.handle_mlat(receiver_id, timestamp, &message_hex, utc).await;
                }
                TYPE_MLAT_LONG => {
                    if i + MLAT_LONG_LEN > data.len() {
                        break;
                    }
                    let t = i32::from_be_bytes(data[i..i + 4].try_into().unwrap());
                    let m = &data[i + 4..i + 18]; // 14 bytes
                    i += MLAT_LONG_LEN;
                    let timestamp = (base as i64 + t as i64) as f64;
                    let message_hex = hex::encode(m);
                    coordinator.handle_mlat(receiver_id, timestamp, &message_hex, utc).await;
                }
                TYPE_REBASE => {
                    if i + REBASE_LEN > data.len() {
                        break;
                    }
                    base = u64::from_be_bytes(data[i..i + 8].try_into().unwrap());
                    i += REBASE_LEN;
                }
                TYPE_ABS_SYNC => {
                    if i + ABS_SYNC_LEN > data.len() {
                        break;
                    }
                    let et = u64::from_be_bytes(data[i..i + 8].try_into().unwrap());
                    let ot = u64::from_be_bytes(data[i + 8..i + 16].try_into().unwrap());
                    let em = &data[i + 16..i + 30];
                    let om = &data[i + 30..i + 44];
                    i += ABS_SYNC_LEN;
                    let even_ts = et as f64;
                    let odd_ts = ot as f64;
                    let even_hex = hex::encode(em);
                    let odd_hex = hex::encode(om);
                    coordinator.handle_sync(receiver_id, even_ts, odd_ts, &even_hex, &odd_hex).await;
                }
                TYPE_SSYNC => {
                    // Skip unsupported type (same size as SYNC for skip)
                    if i + SYNC_LEN <= data.len() {
                        i += SYNC_LEN;
                    } else {
                        break;
                    }
                }
                _ => {
                    debug!("UDP bad type byte {} from {}", type_byte, peer);
                    break;
                }
            }
        }
    }
}
