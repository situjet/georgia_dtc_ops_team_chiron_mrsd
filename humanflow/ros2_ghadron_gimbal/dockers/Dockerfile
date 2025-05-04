# =========================================================
# Builder Stage - ARM64 交叉编译
# =========================================================
# ARG BUILDPLATFORM=linux/arm64
# FROM --platform=${BUILDPLATFORM} kamicreed/salmoncounter:latest-jetson-jetpack6 AS builder
ARG BASE_IMAGE
FROM ${BASE_IMAGE:-dustynv/l4t-pytorch:r36.2.0}
ARG TARGETARCH

# =========================================================
# 1. 设置交叉编译环境
# =========================================================
# 创建交叉编译工具链文件
WORKDIR /root/ros_ws

# Create the /ros2_ws directory before writing to it
RUN mkdir -p /ros2_ws

RUN echo 'set(CMAKE_SYSTEM_NAME Linux)' > /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_SYSTEM_PROCESSOR aarch64)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu /usr/lib/aarch64-linux-gnu /opt/ros/humble /opt/ros/humble/lib/aarch64-linux-gnu)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)' >> /ros2_ws/arm64_toolchain.cmake && \
    echo 'set(PKG_CONFIG_EXECUTABLE "/usr/bin/pkg-config")' >> /ros2_ws/arm64_toolchain.cmake

# 为交叉编译配置 PKG_CONFIG
RUN mkdir -p /usr/share/pkg-config-crossfile && \
    echo 'PKG_CONFIG_PATH="/usr/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/humble/lib/aarch64-linux-gnu/pkgconfig"' >> /etc/environment && \
    echo 'PKG_CONFIG_LIBDIR="/usr/lib/aarch64-linux-gnu/pkgconfig:/opt/ros/humble/lib/aarch64-linux-gnu/pkgconfig"' >> /etc/environment && \
    echo "export CV_BRIDGE_DIR=/opt/ros/humble/share/cv_bridge/cmake" >> /etc/environment && \
    echo "export CV_BRIDGE_LIB_DIR=/opt/ros/humble/lib" >> /etc/environment


ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
# =========================================================
# 2. 安装构建依赖项
# =========================================================
# 添加 ROS2 Humble 仓库
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add this line before your apt-get commands
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
        # 基本构建工具
        build-essential \
        cmake \
        pkg-config \
        # ROS 构建工具
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        # 交叉编译工具
        gcc-aarch64-linux-gnu \
        g++-aarch64-linux-gnu \
        qemu-user-static \
        binfmt-support && \
    # 为 ARM64 设置多架构支持
    dpkg --add-architecture arm64

# =========================================================
# 3. 安装 ARM64 依赖项
# =========================================================
# 安装基本系统库
RUN apt-get update && apt-get install -y --no-install-recommends \
        libglib2.0-dev:arm64 \
        libboost-all-dev:arm64 \
        libeigen3-dev:arm64 \
        libcurl4-openssl-dev:arm64 \
        libjsoncpp-dev:arm64 \
        pkg-config:arm64

# Remove conflicting OpenCV packages
RUN apt-get remove -y opencv-main || true
RUN apt-get remove -y 'libopencv*' || true
RUN apt-get remove -y 'opencv*' || true
RUN apt-get autoremove -y || true

RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev:arm64

# 单独安装 GStreamer 包 
RUN apt-get update && \
       apt-get install -y --no-install-recommends \
        libgstreamer1.0-dev:arm64 \
        libgstreamer-plugins-base1.0-dev:arm64 \
        gstreamer1.0-plugins-base:arm64 \
        gstreamer1.0-plugins-good:arm64 \
        gstreamer1.0-plugins-bad:arm64

# 安装 OpenCV 运行时库，避免开发包
RUN apt-get update && apt-get install -y --no-install-recommends \
        libopencv-core4.5d:arm64 \
        libopencv-imgproc4.5d:arm64 \
        libopencv-imgcodecs4.5d:arm64 \
        libopencv-videoio4.5d:arm64

# 单独安装 ROS 视觉包，避免 OpenCV 开发包依赖
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-cv-bridge:arm64 \
        ros-humble-vision-opencv:arm64 \
        python3-cv-bridge:arm64 \
        ros-humble-vision-msgs:arm64 \
        ros-humble-camera-info-manager:arm64 \
        ros-humble-image-transport:arm64


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-image-transport:arm64 \
        ros-humble-image-transport-plugins:arm64

# 初始化 rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update --rosdistro humble

# FROM public.ecr.aws/docker/library/ros:humble-ros-base

# 安装运行时依赖项
RUN apt-get update && \
    dpkg --add-architecture arm64 && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    # 视觉和 OpenCV 包 - 使用运行时库而非开发库
    ros-humble-cv-bridge:arm64 \
    ros-humble-vision-opencv:arm64 \
    libopencv-core4.5d:arm64 \
    libopencv-imgproc4.5d:arm64 \
    libopencv-imgcodecs4.5d:arm64 \
    libopencv-videoio4.5d:arm64 \
    python3-cv-bridge:arm64 \
    ros-humble-vision-msgs:arm64 \
    # MAVROS 包 - 新增
    ros-humble-mavros:arm64 \
    ros-humble-mavros-extras:arm64

# 单独安装更多依赖项
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Python 依赖项
    python3-pip \
    python3-dev \
    python3-numpy \
    # 相机和媒体包
    ros-humble-camera-info-manager:arm64 \
    ros-humble-image-transport:arm64 \
    # 其他依赖项
    libeigen3-dev:arm64

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev

RUN python3 -m pip install --upgrade pip

# Fix NumPy version compatibility issue - pin to version 1.x
RUN pip3 install "numpy<2.0.0" --force-reinstall 

# Install PyTorch, torchvision and onnxruntime from Dustin's site for Jetpack 6.1
RUN wget http://jetson.webredirect.org/jp6/cu126/+f/5cf/9ed17e35cb752/torch-2.5.0-cp310-cp310-linux_aarch64.whl && \
    wget http://jetson.webredirect.org/jp6/cu126/+f/5f9/67f920de3953f/torchvision-0.20.0-cp310-cp310-linux_aarch64.whl && \
    wget http://jetson.webredirect.org/jp6/cu126/+f/0c4/18beb3326027d/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl && \
    pip3 install torch-2.5.0-cp310-cp310-linux_aarch64.whl && \
    pip3 install torchvision-0.20.0-cp310-cp310-linux_aarch64.whl && \
    pip3 install onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl && \
    rm torch-2.5.0-cp310-cp310-linux_aarch64.whl && \
    rm torchvision-0.20.0-cp310-cp310-linux_aarch64.whl && \
    rm onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl

# Install wget for downloading packages
RUN apt-get update && apt-get install -y --no-install-recommends wget

# Add NVIDIA CUDA repository with proper keyring
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y libcusparselt0 libcusparselt-dev && \
    rm cuda-keyring_1.1-1_all.deb

# Install cuDNN 9 for Jetson
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    libcudnn9 \
    libcudnn9-dev \
    || echo "Installing cuDNN from NVIDIA repository..."

# If cuDNN is not available in the standard repos, try to get it from NVIDIA
RUN if [ ! -f /usr/lib/aarch64-linux-gnu/libcudnn.so.9 ]; then \
    apt-get update && \
    apt-get install -y --no-install-recommends libcudnn9 libcudnn9-dev; \
    fi

# Create symbolic link for libcudnn if needed
RUN if [ -f /usr/lib/aarch64-linux-gnu/libcudnn.so ] && [ ! -f /usr/lib/aarch64-linux-gnu/libcudnn.so.9 ]; then \
    ln -sf /usr/lib/aarch64-linux-gnu/libcudnn.so /usr/lib/aarch64-linux-gnu/libcudnn.so.9; \
    fi

RUN pip3 install ultralytics 
    # Install other dependencies
RUN    pip3 install matplotlib tqdm requests

# # Set CUDA environment variables for Jetson  (These are usually set by the base image, so likely not needed)
# ENV CUDA_HOME=/usr/local/cuda
# ENV PATH=$PATH:$CUDA_HOME/bin
# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64:$CUDA_HOME/extras/CUPTI/lib64


# Create symbolic links for CUDA libraries if needed
RUN ldconfig && \
    echo "Testing PyTorch CUDA availability..." && \
    python3 -c "import torch; print('CUDA available:', torch.cuda.is_available()); print('CUDA version:', torch.version.cuda if torch.cuda.is_available() else 'Not available')" || echo "PyTorch CUDA check failed - will need to debug in container"

# For JetPack 6.0/L4T R36
# 额外依赖项
RUN apt-get update && apt-get install -y \
    libgtk-3-dev:arm64 \
    libyaml-cpp-dev:arm64 \
    libprotobuf-dev:arm64 \
    protobuf-compiler:arm64 \
    libfftw3-dev:arm64 \
    libspdlog-dev:arm64 \
    libjpeg-dev:arm64 \
    libpng-dev:arm64 \
    libpng16-16:arm64 \
    libxml2-dev:arm64 \
    libssl-dev:arm64 \
    libtinyxml2-dev:arm64 \
    libzmq3-dev:arm64 \
    libglib2.0-dev \
    gstreamer1.0-tools \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libjsoncpp-dev \
    libcurl4-openssl-dev \
    ffmpeg:arm64 \
    ros-humble-rosbag2-storage-mcap

# Fix library loading issues
RUN ldconfig && \
    echo "/usr/lib/aarch64-linux-gnu" > /etc/ld.so.conf.d/aarch64-linux-gnu.conf && \
    ldconfig

# Create symbolic links for common library versions
RUN find /usr/lib/aarch64-linux-gnu -name "libpng16*.so*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libpng16-0090a4c3.so.16.44.0 || true

# Install additional FFmpeg dependencies and create necessary symlinks for OpenCV
RUN apt-get update && apt-get install -y --no-install-recommends \
    libavcodec-dev:arm64 \
    libavformat-dev:arm64 \
    libavutil-dev:arm64 \
    libswscale-dev:arm64 \
    libswresample-dev:arm64 \
    libavfilter-dev:arm64 \
    libavdevice-dev:arm64 && \
    find /usr/lib/aarch64-linux-gnu -name "libavcodec.so*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libavcodec-2e98a126.so.59.37.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libavcodec.so.59*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libavcodec-2e98a126.so.59.37.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libavformat.so*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libavformat-ef9e8359.so.59.27.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libavformat.so.59*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libavformat-ef9e8359.so.59.27.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libavutil.so*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libavutil-2dc4740f.so.57.28.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libavutil.so.57*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libavutil-2dc4740f.so.57.28.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libswscale.so*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libswscale-e52af062.so.6.7.100 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libswscale.so.6*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libswscale-e52af062.so.6.7.100 || true && \
    ldconfig

# Install Qt5 dependencies and create symbolic links for missing Qt5 libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libqt5core5a:arm64 \
    libqt5gui5:arm64 \
    libqt5widgets5:arm64 \
    libqt5network5:arm64 \
    qtbase5-dev:arm64 && \
    find /usr/lib/aarch64-linux-gnu -name "libQt5Widgets.so*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libQt5Widgets-42fd29df.so.5.15.16 || true && \
    find /usr/lib/aarch64-linux-gnu -name "libQt5Widgets.so.5*" | \
    xargs -I{} ln -sf {} /usr/lib/aarch64-linux-gnu/libQt5Widgets-42fd29df.so.5.15.16 || true && \
    ldconfig

# =========================================================
# 4. Configure working directory and volumes
# =========================================================
# Set up working directory to match the mounted path

# Set up ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create directories for volume mounting
RUN mkdir -p /root/ros_ws

# Install ROS2 CLI tools and core packages
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ros-core \
    python3-argcomplete

# Ensure ROS2 commands are in PATH
RUN echo 'export PATH=$PATH:/opt/ros/humble/bin' >> ~/.bashrc

# Install geographiclib datasets required by MAVROS
RUN apt-get update && apt-get install -y wget
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh
RUN rm install_geographiclib_datasets.sh

# Set the entrypoint script
# ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
