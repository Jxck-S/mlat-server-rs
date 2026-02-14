# Multi-stage build: compile with Rust, run on minimal Debian
# Stage 1: build (Rust is pre-installed in official image)
FROM rust:1-bookworm AS builder

WORKDIR /build

# Copy manifest and source (deps cache when Cargo.toml/Cargo.lock unchanged)
COPY Cargo.toml Cargo.lock ./
COPY src ./src

RUN cargo build --release

# Stage 2: minimal runtime
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Non-root user for running the server
RUN groupadd -r mlat && useradd -r -g mlat -d /run/mlat-server-rs mlat

# Install binary from builder
COPY --from=builder /build/target/release/mlat-server /usr/local/bin/mlat-server

# Entrypoint script (forwards args to mlat-server)
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Work directory for state files (sync.json, clients.json, aircraft.json)
ENV WORK_DIR=/run/mlat-server-rs
RUN mkdir -p "$WORK_DIR" && chown -R mlat:mlat "$WORK_DIR"

# Default ports: client (Beast/Raw), basestation, HTTP (if added later)
EXPOSE 31090 30104 8080

USER mlat
WORKDIR /run/mlat-server-rs

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["--client-listen", "31090", "--filtered-basestation-listen", "30104", "--work-dir", "/run/mlat-server-rs"]
