# mlat-server (Rust)

Rust implementation of the **mlat-server** multilateration server. Compatible with existing mlat-client software; supports the same protocol and command-line interface as the [original Python server](https://github.com/wiedehopf/mlat-server).


## What is MLAT?

Multilateration (MLAT) determines aircraft positions from the time difference of arrival (TDOA) of Mode S messages at multiple receivers. This server works with unsynchronized receivers, uses ADS-B position reports as references, and performs multilateration for aircraft that only transmit Mode S.

## Requirements

- Rust 1.70+ (e.g. `rustup`)

## Build

```bash
cargo build --release
```

## Run

```bash
cargo run --release -- --client-listen 4100 --work-dir .work
```

Or run the built binary:

```bash
./target/release/mlat-server --client-listen 4100 --work-dir .work
```

## Command-line arguments

| Argument | Description |
|----------|-------------|
| `--client-listen ADDR` | Listen on `[host:]tcp_port[:udp_port]` for mlat clients. Can be repeated. Example: `4100` or `0.0.0.0:4100:4101`. |
| `--motd TEXT` | Message of the day sent to clients (default: empty). |
| `--write-csv FILE` | Append CSV results to a file. Can be repeated. |
| `--basestation-connect HOST:PORT` | Connect and send Basestation-format results. Can be repeated. |
| `--basestation-listen ADDR` | Listen on `host:port` and send Basestation-format to connecting clients. Can be repeated. Use a full address, e.g. `127.0.0.1:30003` (port alone is invalid). |
| `--filtered-basestation-connect HOST:PORT` | Same as above, filtered. Can be repeated. |
| `--filtered-basestation-listen ADDR` | Same as above (listen), filtered. Use `host:port`, e.g. `127.0.0.1:30003`. Can be repeated. |
| `--work-dir DIR` | Directory for debug/stats and blacklist. |
| `--check-leaks` | (Reserved; no-op in Rust.) |
| `--dump-pseudorange FILE` | Dump pseudorange data as JSON to a file. |
| `--partition N/COUNT` | Partitioning, e.g. `1/1` (default: no partitioning). |
| `--tag NAME` | Process name prefix (default: `mlat-server`). |
| `--status-interval SECS` | Status log interval in seconds (multiple of 15; use `-1` to disable). Default: `15`. |
| `--verbose`, `-v` | Enable DEBUG logging. |

### Example

```bash
./target/release/mlat-server \
  --client-listen 0.0.0.0:4100 \
  --work-dir .work \
  --basestation-listen 127.0.0.1:30003 \
  --basestation-connect 127.0.0.1:30005 \
  --write-csv results.csv \
  --verbose
```

## Architecture

### Overview

The server has two main directions of flow:

- **Input**: mlat clients (receivers) connect over TCP or UDP and send Mode S messages and sync data. The server maintains one **receiver** per client (one connection per username).
- **Output**: MLAT results (position solutions) are sent to configured outputs: Basestation-format TCP (connect or listen), CSV files, and optional work-dir JSON/HTTP.

A single **Coordinator** ties everything together: it owns receivers, the aircraft **tracker**, **clock tracker** (receiver clock sync), and **MLAT tracker** (solver). When a solution is produced, the coordinator calls each registered **output handler**.

### Inputs (clients and listeners)

| Component | Parameter | Direction | Description |
|-----------|-----------|-----------|-------------|
| **Client TCP listener** | `--client-listen ADDR` | Clients → server | Listen for mlat-client connections. Format: `[host:]tcp_port[:udp_port]` (e.g. `4100` or `0.0.0.0:4100:4101`). Each accepted connection becomes a **JSON client**; after handshake (user, lat, lon, alt) a **receiver** is created and messages (MLAT + sync) are processed by the coordinator. |
| **Client UDP** | (same `--client-listen` with optional `:udp_port`) | Clients → server | Optional UDP port for the same client protocol. |
| **MOTD** | `--motd TEXT` | Server → clients | Message of the day sent to clients on connect (AGPL notice is always appended). |

Clients send timestamped Mode S messages and sync messages. The server uses them for clock modeling and multilateration; results are then pushed to outputs.

### Outputs (results and state)

| Component | Parameter | Direction | Description |
|-----------|-----------|-----------|-------------|
| **Basestation connect** | `--basestation-connect HOST:PORT` | Server → remote | Server connects out to the given host:port and streams Basestation-format (SBS) lines for each MLAT result. |
| **Basestation listen** | `--basestation-listen ADDR` | Remote → server, then server → client | Server listens on `host:port` (e.g. `127.0.0.1:30003`); when a client connects, the server streams SBS to that client. |
| **Filtered Basestation** | `--filtered-basestation-connect`, `--filtered-basestation-listen` | Same as above | Same as above but filtered (e.g. by aircraft or region). |
| **CSV** | `--write-csv FILE` | Server → file | Append one CSV row per MLAT result to the given file(s). |
| **Work directory** | `--work-dir DIR` | Server → files | Writes `sync.json`, `clients.json`, `aircraft.json` (and handshakes.log) for debugging/stats. |
| **HTTP server** | `HTTP_PORT` (env) | Browser/tools → server | If set, serves the work directory over HTTP (read-only). No CLI flag. |

### Flow summary

```
mlat-client (TCP/UDP)  ──►  Client listeners  ──►  JsonClient  ──►  Coordinator
                                                                        │
                                    ┌───────────────────────────────────┼───────────────────────────────────┐
                                    ▼                   ▼               ▼                                   ▼
                            Receiver (per client)   Tracker      ClockTracker   MlatTracker (solver)
                                    │                   │               │               │
                                    └───────────────────┴───────────────┴───────────────┘
                                                                        │
                                                                        ▼
                                                            MLAT results (position, time)
                                                                        │
                                    ┌───────────────────────────────────┼───────────────────────────────────┐
                                    ▼                   ▼               ▼                                   ▼
                            Basestation (connect)  Basestation (listen)  CSV files   Work-dir JSON / HTTP
```

### Other parameters (behavior and tuning)

- `--work-dir` — Directory for state files and blacklist; required for writing sync/clients/aircraft JSON.
- `--partition N/COUNT` — Partitioning (e.g. `1/1` = no partition).
- `--tag` — Process name prefix in logs.
- `--status-interval SECS` — How often to log status (e.g. client count); multiple of 15, or `-1` to disable.
- `--dump-pseudorange FILE` — Dump pseudorange JSON for debugging.
- `--verbose` / `-v` — DEBUG logging.

## Docker

Build and run with Docker Compose (Rust is installed only in the build stage; the running image is minimal Debian):

```bash
docker compose build
docker compose up -d
```

- **Dockerfile**: Multi-stage build using `rust:1-bookworm` for compilation and `debian:bookworm-slim` for runtime. The binary runs as a non-root user with `WORK_DIR` at `/run/mlat-server-rs`.
- **docker-compose.yml**: Exposes client port 31090, basestation 30104, and 8080 (HTTP file server). Ulimits are set for high concurrency; override `command` or environment as needed.

### Optional HTTP server (work-dir JSON files)

If the environment variable `HTTP_PORT` is set, the server starts a simple HTTP file server on that port serving the **work directory** (e.g. `sync.json`, `clients.json`, `aircraft.json`). In Docker Compose this is enabled with `HTTP_PORT=8080`. No separate process is needed; it runs inside the same binary.

### Work directory and permissions

The server writes state files (e.g. `sync.json`, `clients.json`, `aircraft.json`) into `--work-dir`. In Docker you typically bind-mount a host directory so data persists and you can access the files.

- **Create and use a host directory**  
  Example: mount `./work` into the container’s work dir:

  ```yaml
  volumes:
    - ./work:/run/mlat-server-rs
  ```

  Ensure the directory exists (e.g. `mkdir -p work`) and that the container user can write to it (see below).

- **Running as non-root (default)**  
  The image runs as user `mlat` (non-root). The Dockerfile creates `/run/mlat-server-rs` and sets ownership to `mlat:mlat`. If you mount a host path (e.g. `./work:/run/mlat-server-rs`), that directory must be writable by the container’s user. Options:

  - **Match host UID/GID**: Create the host directory with the same UID/GID as the `mlat` user in the image (e.g. from the Dockerfile, or inspect with `docker run --rm --entrypoint id <image>`).
  - **Open permissions**: e.g. `chmod 777 work` (not recommended on shared systems).
  - **Run once as root** to create the dir and chown it to the desired UID/GID, then run as that user.

- **Running as root**  
  To run the container as root (e.g. to avoid permission issues with an existing host directory):

  ```yaml
  user: root
  ```

  Or override at run time: `docker run --user root ...`. The server will create and write to `--work-dir` as root. Only do this if you understand the security implications.

## Development tools (receiving and mapping SBS output)

For local development and testing you can pipe this server’s Basestation (SBS) output into [readsb](https://github.com/wiedehopf/readsb) and then view positions on a map with [tar1090](https://github.com/wiedehopf/tar1090).

- **readsb** receives MLAT results from this server (e.g. over SBS), merges them with live ADS-B, and can write JSON and Beast output for display.
- **tar1090** is a web UI that consumes the data readsb produces (e.g. `aircraft.json`, history) and shows aircraft on a map.

Typical flow:

1. Run mlat-server with a Basestation listen address (must be `host:port`), e.g.  
   `--basestation-listen 127.0.0.1:30003`
2. Run readsb so it connects to that SBS stream, e.g.  
   `--net-connector 127.0.0.1,30003,sbs_in` (plus your usual readsb options: `--net`, `--write-json`, etc.).
3. Install and run tar1090 pointing at the same data directory readsb uses; open the tar1090 page in a browser to see MLAT and ADS-B traffic on the map.

See the [readsb](https://github.com/wiedehopf/readsb) and [tar1090](https://github.com/wiedehopf/tar1090) repositories for build, install, and configuration details.

## Project layout

- `src/` — Rust server (modes, solver, network, coordinator).

## Testing

```bash
cargo test
```

Verification and comparison scripts live under `tools/` and `tests/` (Python); use a virtualenv if you run them.
