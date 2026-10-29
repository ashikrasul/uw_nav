#!/bin/bash
# Open a shell in a running service: ./enter_console.sh [service]  (default: nav)
cd "$(dirname "$0")" || exit 1

SERVICE_NAME="${1:-nav}"

VALID_SERVICES=$(docker compose -f docker-compose.yml config --services)
if ! echo "$VALID_SERVICES" | grep -qx "$SERVICE_NAME"; then
    echo "Error: Invalid service name '$SERVICE_NAME'"
    echo "Usage: $0 [<service_name>]"
    echo "Available services:"
    echo "$VALID_SERVICES"
    exit 1
fi

docker compose -f docker-compose.yml exec "$SERVICE_NAME" /bin/bash
