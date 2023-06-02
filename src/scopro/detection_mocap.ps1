# Nom de l'image Docker
$DOCKER_IMAGE_NAME="mocap_detection"

# Nom du conteneur Docker
$DOCKER_CONTAINER_NAME="mocap_pose"

# Construire l'image Docker
docker build -t $DOCKER_IMAGE_NAME .

# Vérifier si un conteneur avec le même nom existe déjà et, si c'est le cas, le supprimer
if ((docker ps -a -f name=$DOCKER_CONTAINER_NAME | Select-String -Pattern $DOCKER_CONTAINER_NAME) -ne $null)
{
    Write-Host "Container $DOCKER_CONTAINER_NAME already exists, removing..."
    docker rm $DOCKER_CONTAINER_NAME
}

# Exécuter le conteneur Docker
docker run -it --name $DOCKER_CONTAINER_NAME -p 11311:11311 -v "C:/scopro_ws/:/mnt/c/scopro_ws" --network host $DOCKER_IMAGE_NAME
