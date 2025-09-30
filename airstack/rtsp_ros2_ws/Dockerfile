FROM ros:humble-ros-base

# Fix expired GPG key issue by removing old keys and installing updated apt source
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list \
    && rm -f /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
    && apt-get update \
    && apt-get install -y ca-certificates curl \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    libopencv-dev \
    python3-opencv \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavdevice-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY src/ src/

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set FastDDS environment variables
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/fastdds_profile.xml
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=0

# Create FastDDS profile
RUN echo '<?xml version="1.0" encoding="UTF-8" ?>' > /ros2_ws/fastdds_profile.xml && \
    echo '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">' >> /ros2_ws/fastdds_profile.xml && \
    echo '    <transport_descriptors>' >> /ros2_ws/fastdds_profile.xml && \
    echo '        <transport_descriptor>' >> /ros2_ws/fastdds_profile.xml && \
    echo '            <transport_id>udpv4_transport</transport_id>' >> /ros2_ws/fastdds_profile.xml && \
    echo '            <type>UDPv4</type>' >> /ros2_ws/fastdds_profile.xml && \
    echo '        </transport_descriptor>' >> /ros2_ws/fastdds_profile.xml && \
    echo '    </transport_descriptors>' >> /ros2_ws/fastdds_profile.xml && \
    echo '    <participant profile_name="disable_shm_participant_profile" is_default_profile="true">' >> /ros2_ws/fastdds_profile.xml && \
    echo '        <rtps>' >> /ros2_ws/fastdds_profile.xml && \
    echo '            <userTransports>' >> /ros2_ws/fastdds_profile.xml && \
    echo '                <transport_id>udpv4_transport</transport_id>' >> /ros2_ws/fastdds_profile.xml && \
    echo '            </userTransports>' >> /ros2_ws/fastdds_profile.xml && \
    echo '            <useBuiltinTransports>false</useBuiltinTransports>' >> /ros2_ws/fastdds_profile.xml && \
    echo '        </rtps>' >> /ros2_ws/fastdds_profile.xml && \
    echo '    </participant>' >> /ros2_ws/fastdds_profile.xml && \
    echo '</profiles>' >> /ros2_ws/fastdds_profile.xml

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "rtsp_streamer", "rtsp_streamer.launch.py"]