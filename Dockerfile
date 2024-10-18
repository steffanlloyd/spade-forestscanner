FROM tiryoh/ros-melodic-desktop

# Add extra programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    i2c-tools \
    build-essential \
    libffi-dev \
    python3-pip \
    python3-dev \ 
    sudo \
    wget \
    cmake \ 
    ros-melodic-mavros \
    && rm -rf /var/lib/apt/lists/*

# Run geographiclib
WORKDIR /home/ros
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Install Livox SDK and Livox SDK2
RUN mkdir libraries
WORKDIR /home/ros/libraries/
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
WORKDIR /home/ros/libraries/Livox-SDK2/
RUN mkdir build
WORKDIR /home/ros/libraries/Livox-SDK2/build/
RUN cmake .. && make && make install
WORKDIR /home/ros/libraries/Livox-SDK/build/
RUN cmake .. && make && make install
WORKDIR /home/ros

# Setup user. The USER_UID needs to match the user of the host computer! That way the files won't have access issues.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \ 
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config \
    && chown -R $USER_UID:$USER_GID /home/$USERNAME

# Add sudo privelidges
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install python requirements
COPY ./docker/requirements.txt requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# Change access of the ros user. Add to i2c group
RUN usermod -aG i2c ros
RUN usermod -aG dialout ros
RUN 

# Add more extra programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Set the container's environment variables to enable rviz and others
ENV QT_X11_NO_MITSHM=1

COPY ./docker/entrypoint.sh /entrypoint.sh
COPY ./docker/bashrc.txt /home/${USERNAME}/.bashrc
RUN mkdir /home/${USERNAME}/scripts/
COPY ./docker/init_docker.sh /home/${USERNAME}/scripts/init.sh
RUN chmod +x /home/${USERNAME}/scripts/init.sh

# Update ROS dependencies
USER ros
WORKDIR /home/ros/ros1_ws
RUN rosdep update

USER root

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

CMD ["bash"]
