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
| `--basestation-listen ADDR` | Listen on `[host:]port` and send Basestation-format to connecting clients. Can be repeated. |
| `--filtered-basestation-connect HOST:PORT` | Same as above, filtered. Can be repeated. |
| `--filtered-basestation-listen ADDR` | Same as above (listen), filtered. Can be repeated. |
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
  --basestation-connect 127.0.0.1:30005 \
  --write-csv results.csv \
  --verbose
```

## Docker

Build and run with Docker Compose (Rust is installed only in the build stage; the running image is minimal Debian):

```bash
docker compose build
docker compose up -d
```

- **Dockerfile**: Multi-stage build using `rust:1-bookworm` for compilation and `debian:bookworm-slim` for runtime. The binary runs as a non-root user with `WORK_DIR` at `/run/mlat-server-rs`.
- **docker-compose.yml**: Exposes client port 31090, basestation 30104, and 8080 (reserved). Ulimits are set for high concurrency; override `command` or environment as needed.

## Project layout

- `src/` — Rust server (modes, solver, network, coordinator).

## Testing

```bash
cargo test
```

Verification and comparison scripts live under `tools/` and `tests/` (Python); use a virtualenv if you run them.
