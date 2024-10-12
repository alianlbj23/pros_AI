#!/bin/bash

# Port mapping check
PORT_MAPPING=""
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3  # Remove the first three arguments
fi

# gpu check
if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected"
    GPU_FLAGS="--runtime nvidia --gpus all"
else
    echo "No NVIDIA GPU detected or NVIDIA drivers not installed"
    GPU_FLAGS=""
fi

# env file check
if [ -f ./.env ]; then
    ENV_FILE="--env-file ./.env"
else
    echo "Warning: .env file not found"
    ENV_FILE=""
fi

# network check
if docker network inspect compose_my_bridge_network &> /dev/null; then
    NETWORK="--network compose_my_bridge_network"
else
    echo "Warning: compose_my_bridge_network not found, using default network"
    NETWORK=""
fi

# run docker container
docker run -it --rm \
    -v "$(pwd)/AI_pkg2:/workspaces/AI_pkg2" \
    -v "$(pwd)/AI_pkg:/workspaces/AI_pkg" \
    $GPU_FLAGS \
    $NETWORK \
    $ENV_FILE \
    $PORT_MAPPING \
    ghcr.io/otischung/pros_ai_image:latest \
    /bin/bash
