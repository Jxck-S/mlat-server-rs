#!/bin/sh
# Pass all arguments to mlat-server (e.g. from docker-compose command:)
set -e
exec /usr/local/bin/mlat-server "$@"
