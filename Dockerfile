FROM ubuntu:20.04
# Install necessary packages for noetic installation
RUN apt update
RUN apt -y install sudo \ 
    gnupg2 \ 
    lsb-release \ 
    curl
    
# Get Key from ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update
# Set selections for region and city
RUN echo 'tzdata tzdata/Areas select Europe' | debconf-set-selections \
    && echo 'tzdata tzdata/Zones/Europe select Berlin' | debconf-set-selections
# Install ROS Noetic
RUN apt-get update && \
    # Start interactive installation
    DEBIAN_FRONTEND=teletype \
    apt-get install -y ros-noetic-desktop-full

# Set up ROS environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Install ROS dependencies
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# Initialize rosdep
RUN rosdep init && rosdep update

#-----------------docker-ros-frank2 is compiled until here

    # Build libfranka from source
# Delete existing libfranka packages and install necessary packages for the installation
RUN apt remove "*libfranka"
RUN apt -y install build-essential cmake git libpoco-dev libeigen3-dev iputils-ping
# Download and build libranka version 0.10.0
RUN git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0
WORKDIR /libfranka
RUN mkdir build
WORKDIR /libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
RUN cmake --build .

WORKDIR /
RUN mkdir -p /code
WORKDIR /code

# Install necessary packages

RUN apt-get update && apt-get install -y \
    wget \
    python3-dev \
    python3-pip \
    python3-setuptools \
    nano \
    # Clear cache
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy

# -------------- #
# Install Eigen3 #
# -------------- #

RUN git clone https://gitlab.com/libeigen/eigen.git \
    && cd eigen \
    && git checkout 3.4.0 \
    && mkdir build && cd build \
    && cmake .. \
    && make install

# ---------------- #
# Install PyBind11 #
# ---------------- #

RUN git clone https://github.com/pybind/pybind11.git \
    && cd pybind11 \
    && git checkout v2.10 \
    && mkdir build && cd build \
    && cmake -DPYBIND11_TEST=OFF .. \
    && make -j$(nproc) \
    && make install

# ---------------- #
# Install Catch2   #
# ---------------- #

RUN git clone https://github.com/catchorg/Catch2.git \
    && cd Catch2 \
    && git checkout v2.5.0 \
    && mkdir build && cd build \
    && cmake -DCATCH_BUILD_TESTING=OFF -DCATCH_ENABLE_WERROR=OFF -DCATCH_INSTALL_DOCS=OFF -DCATCH_INSTALL_HELPERS=OFF .. \
    && make install

# ---------------- #
# Build franky     #
# ---------------- #


RUN wget https://github.com/TimSchneider42/franky/releases/latest/download/libfranka_0-10-0_wheels.zip
RUN unzip libfranka_0-10-0_wheels.zip
RUN pip install --no-index --find-links=./dist franky-panda

#RUN git clone --recursive https://github.com/timschneider42/franky.git
#RUN mkdir -p franky/build && cd franky/build \
#    && cmake .. 
#  && make -j$(nproc) \
#  && make install \
#  && ./unit-test

#ENV PYTHONPATH=$PYTHONPATH:/code/franky/build/


# Docker has to be run with the following command/flags: 
# docker run -it --rm --network host --cap-add=sys_nice --ulimit rtprio=99 docker-ros-franka2

# example code: ./examples/communication_test 192.168.1.11


# Install libfranka and franka_ros
#RUN apt install -y ros-noetic-libfranka ros-noetic-franka-ros

# Install git
#RUN apt install git
# Install pip and other python tools (python is already installed)
#RUN apt install python3-pip \
#    python3-dev \
#    python3-setuptools

    # Setting up the real time kernel
# install necessary dependencies
#RUN apt -y install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
# Download the kernel
#RUN curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.19.tar.xz
#RUN curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.19.tar.sign
#RUN curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.19-rt10.patch.xz
#RUN curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.19-rt10.patch.sign
# Extract the files
#RUN xz -d *.xz

# Install keys to verify the files
#RUN gpg2 --locate-keys gregkh@kernel.org sebastian@breakpoint.cc bigeasy@linutronix.de 
# Verify the files
#RUN gpg2 --verify linux-*.tar.sign
#RUN gpg2 --verify patch-*.patch.sign

# compiling the kernel
#RUN tar xf linux-*.tar
#WORKDIR /linux-5.4.19/
#RUN patch -p1 < ../patch-5.4.19-rt10.patch

# Install ping
#RUN apt -y install iputils-ping
#-----------------docker-ros-frank3 is compiled until here

# Creating config file
#RUN make olddefconfig
# Set configuration options, since make menuconfig does't work
#RUN sed -i '/CONFIG_PREEMPT=/s/=.*/=y/' .config && \
#    sed -i '/CONFIG_DEBUG_PREEMPT=y/s/=y/=n/' .config && \
#    sed -i '/CONFIG_PREEMPT_VOLUNTARY=n/s/=.*/=y/' .config && \
#    sed -i '/CONFIG_PREEMPT_COUNT=y/s/=.*/=y/' .config && \
#    sed -i '/CONFIG_KEYS=/s/=.*/=y/' .config && \
#    sed -i '/CONFIG_TRUSTED_KEYS=/s/=.*/=y/' .config && \
#    sed -i '/CONFIG_SYSTEM_TRUSTED_KEYRING=/s/=.*/=y/' .config

# install packages necessary to compile the kernel
#RUN apt update && apt install rsync cpio
# Compile the kernel
#RUN make -j$(nproc) deb-pkg
# install newly created packages
#RUN dpkg -i ../linux-headers-*.deb ../linux-image-*.deb




