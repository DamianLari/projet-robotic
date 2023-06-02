#!/bin/bash

# Nom de l'image Docker
DOCKER_IMAGE_NAME="aruco_detection"

# Nom du conteneur Docker
DOCKER_CONTAINER_NAME="aruco_pose"

# Construire l'image Docker
docker build -t $DOCKER_IMAGE_NAME .

# Vérifier si un conteneur avec le même nom existe déjà et, si c'est le cas, le supprimer
if [ $(docker ps -a -f name=$DOCKER_CONTAINER_NAME | grep -w $DOCKER_CONTAINER_NAME | wc -l) -eq 1 ]
  then
    echo "Container $DOCKER_CONTAINER_NAME already exists, removing..."
    docker rm $DOCKER_CONTAINER_NAME
fi

# Exécuter le conteneur Docker
docker run -it --name $DOCKER_CONTAINER_NAME -e DISPLAY=host.docker.internal:0 -p 80:80 -v "C:/scopro/:/mnt/c/scopro" --network host $DOCKER_IMAGE_NAME
