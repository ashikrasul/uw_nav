#!/bin/bash
# Restart one service: ./restart_service.sh <service>
cd "$(dirname "$0")" || exit 1

if [ -z "$1" ]; then
    echo "Usage: $0 <service_name>"
    exit 1
fi

docker compose -f docker-compose.yml restart "$1"
