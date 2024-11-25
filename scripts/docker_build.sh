#!/bin/bash

# Stop docker, if running
if [ "$(docker ps -q -f name=forestscanner-ros1)" ]; then
    # Prompt the user
    read -p "The Docker container 'forestscanner-ros1' is already running. Do you want to stop it? (y/n) " answer
    case ${answer:0:1} in
        y|Y )
            # Stop the Docker container
            echo "Stopping docker..."
            docker stop forestscanner-ros1
            echo "Done."
            ;;
        * )
            # Exit the script
            echo "Exiting."
            exit
            ;;
    esac
fi

# Build
docker build \
    --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) \
    -t forestscanner-ros1:latest -f Dockerfile .