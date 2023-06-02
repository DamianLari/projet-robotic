# Utiliser l'image de base de ROS Noetic
FROM ros:noetic

# Mise à jour des paquets et installation des dépendances
RUN apt-get update && apt-get install -y \
    python3\
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-opencv \
    ros-noetic-ros-base \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    git \
    && rm -rf /var/lib/apt/lists/*

# Installer les packages Python supplémentaires
RUN pip3 install numpy scipy paramiko

# Configuration de l'environnement ROS
ENV ROS_PACKAGE_PATH=/root/scopro_ws/src

# Création et initialisation du workspace ROS
WORKDIR /root/scopro_ws

# Copie des scripts ROS dans le workspace
COPY . /root/scopro_ws/src/scopro

# Rendre les scripts Python exécutables
RUN sudo chmod +x /root/scopro_ws/src/scopro/scripts/Calibration.py \
    && sudo chmod +x /root/scopro_ws/src/scopro/scripts/Convertion.py \
    && sudo chmod +x /root/scopro_ws/src/scopro/scripts/Correction.py \
    && sudo chmod +x /root/scopro_ws/src/scopro/scripts/aruco_pose.py \
    && sudo chmod +x /root/scopro_ws/src/scopro/scripts/letwist.py \
    && sudo chmod +x /root/scopro_ws/src/scopro/scripts/mocaruco.py \
    && sudo chmod +x /root/scopro_ws/src/scopro/scripts/mocap_client.py


# Compilation et installation des paquets ROS
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make install"

# Ajout des fichiers d'environnement ROS au .bashrc pour faciliter l'utilisation
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/scopro_ws/devel/setup.bash" >> ~/.bashrc

RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/Calibration.py
RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/Convertion.py
RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/Correction.py
RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/aruco_pose.py
RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/letwist.py
RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/mocaruco.py
RUN sed -i 's/\r$//' /root/scopro_ws/src/scopro/scripts/mocap_client.py






#ENTRYPOINT ["/bin/bash", "-c", "source /root/scopro_ws/devel/setup.bash && roslaunch scopro letwist.launch"]