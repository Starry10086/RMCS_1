# ========================================
# RMCS + 机械臂仿真 融合镜像
# ROS2 Humble + Gazebo Harmonic + MoveIt 2 + RMCS
# ========================================
# 23.0 is the minimum docker engine version required to build.

# Base container, provides a runtime environment
FROM ros:humble AS rmcs-base

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Set timezone and non-interactive mode
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

# Set locale
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ========================================
# 安装 RMCS 原有工具和库 + 基础仿真依赖
# ========================================
# 第一部分：系统工具和库（使用 --no-install-recommends）
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim wget curl unzip git \
    zsh screen tmux \
    usbutils net-tools iputils-ping \
    ripgrep htop fzf \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libceres-dev \
    python3-argcomplete \
    python3-colcon-common-extensions && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# 第二部分：ROS 包（不使用 --no-install-recommends，确保完整依赖）
RUN apt-get update && apt-get install -y \
    # ROS2 可视化工具
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    # MoveIt2 运动规划完整套件
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-visual-tools \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-moveit-planners \
    ros-$ROS_DISTRO-moveit-planners-ompl \
    ros-$ROS_DISTRO-moveit-ros-planning \
    ros-$ROS_DISTRO-moveit-ros-move-group \
    ros-$ROS_DISTRO-moveit-ros-visualization \
    ros-$ROS_DISTRO-moveit-ros-perception \
    ros-$ROS_DISTRO-moveit-kinematics \
    ros-$ROS_DISTRO-moveit-simple-controller-manager \
    ros-$ROS_DISTRO-moveit-resources \
    ros-$ROS_DISTRO-moveit-resources-panda-moveit-config \
    # 机械臂支持包
    ros-$ROS_DISTRO-ur \
    ros-$ROS_DISTRO-ur-description \
    ros-$ROS_DISTRO-ur-msgs \
    # ros2_control 控制框架
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-robot-state-publisher \
    # URDF 支持包
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-urdf-tutorial \
    # 其他工具
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rviz-visual-tools \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-ruckig \
    # 显式安装底层依赖库（RViz2 和 MoveIt 需要）
    ros-$ROS_DISTRO-rviz-ogre-vendor \
    libompl-dev && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 安装 .NET SDK（如果需要）
# ========================================
RUN wget https://packages.microsoft.com/config/ubuntu/22.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb && \
    dpkg -i packages-microsoft-prod.deb && \
    rm packages-microsoft-prod.deb && \
    apt-get update && \
    apt-get install -y dotnet-sdk-8.0 && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 安装 Gazebo Harmonic
# ========================================
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    gz-harmonic \
    libgz-cmake3-dev \
    libgz-common5-dev \
    libgz-math7-dev \
    libgz-plugin2-dev \
    libgz-sim8-dev \
    libgz-transport13-dev && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 从源码编译 ros_gz 以支持 Harmonic
# ========================================
RUN mkdir -p /tmp/ros_gz_ws/src && \
    cd /tmp/ros_gz_ws/src && \
    git clone https://github.com/gazebosim/ros_gz.git -b humble && \
    cd /tmp/ros_gz_ws && \
    apt-get update && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install -r --from-paths src -i -y --rosdistro $ROS_DISTRO && \
    export GZ_VERSION=harmonic && \
    colcon build --cmake-args \
      -DBUILD_TESTING=OFF \
      -DGZ_VERSION=harmonic \
      --merge-install \
      --install-base /opt/ros/$ROS_DISTRO && \
    cd / && \
    rm -rf /tmp/ros_gz_ws && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 安装 OpenVINO runtime
# ========================================
RUN wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    apt-key add ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    rm ./GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB && \
    echo "deb https://apt.repos.intel.com/openvino ubuntu22 main" > /etc/apt/sources.list.d/intel-openvino.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends openvino-2025.2.0 && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 安装 Livox SDK
# ========================================
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    sed -i '6iset(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-c++20-compat -include cstdint")' CMakeLists.txt && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && rm -rf Livox-SDK2

# ========================================
# 初始化 rosdep
# ========================================
RUN rosdep init || true && rosdep update

# ========================================
# Mount RMCS source and install dependencies
# ========================================
RUN --mount=type=bind,target=/rmcs_ws/src,source=rmcs_ws/src,readonly \
    apt-get update && \
    rosdep install --from-paths /rmcs_ws/src --ignore-src -r -y && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 安装 unison 用于文件同步
# ========================================
RUN apt-get update && apt-get install -y unison && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# ========================================
# 设置 Gazebo 环境变量
# ========================================
ENV GZ_VERSION=harmonic
ENV GZ_SIM_RESOURCE_PATH=/usr/share/gazebo/models

# ==========================================
# Developing container, works with devcontainer
# ==========================================
FROM rmcs-base AS rmcs-develop

# Install develop tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libc6-dev gcc-12 g++-12 \
    cmake make ninja-build \
    openssh-client \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt \
    python3-argcomplete \
    python3-colcon-common-extensions && \
    wget -O ./llvm-snapshot.gpg.key https://apt.llvm.org/llvm-snapshot.gpg.key && \
    apt-key add ./llvm-snapshot.gpg.key && \
    rm ./llvm-snapshot.gpg.key && \
    echo "deb https://apt.llvm.org/jammy/ llvm-toolchain-jammy main" > /etc/apt/sources.list.d/llvm-apt.list && \
    apt-get update && \
    version=`apt-cache search clangd- | grep clangd- | awk -F' ' '{print $1}' | sort -V | tail -1 | cut -d- -f2` && \
    apt-get install -y --no-install-recommends clangd-$version && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-$version 50 && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Create ubuntu user with UID 1000
RUN useradd -m -u 1000 -s /bin/bash ubuntu && \
    mkdir -p /home/ubuntu && \
    chown -R 1000:1000 /home/ubuntu

# Copy ssh keys and setup unison
COPY --chown=1000:1000 .ssh/id_rsa /home/ubuntu/.ssh/id_rsa
COPY --chown=1000:1000 .ssh/id_rsa.pub /home/ubuntu/.ssh/id_rsa.pub
COPY --chown=1000:1000 .ssh/config /home/ubuntu/.ssh/config
RUN cd /home/ubuntu && \
    chmod 700 .ssh && \
    chmod 600 .ssh/id_rsa && \
    chmod 644 .ssh/id_rsa.pub && \
    chmod 644 .ssh/config && \
    mkdir -p .unison && \
    echo 'confirmbigdel = false' >> ".unison/default.prf" && \
    chown -R 1000:1000 .unison

# Install latest neovim
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.tar.gz && \
    rm -rf /opt/nvim && \
    tar -C /opt -xzf nvim-linux-x86_64.tar.gz && \
    rm nvim-linux-x86_64.tar.gz

# Change user
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install oh my zsh, change theme to af-magic and setup environment
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    echo 'export PATH="${PATH}:/opt/nvim-linux-x86_64/bin"' >> ~/.zshrc && \
    echo 'export PATH="${PATH}:${RMCS_PATH}/.script"' >> ~/.zshrc && \
    echo '' >> ~/.zshrc && \
    echo '# Gazebo Harmonic 环境' >> ~/.zshrc && \
    echo 'export GZ_VERSION=harmonic' >> ~/.zshrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=/usr/share/gazebo/models' >> ~/.zshrc && \
    echo '' >> ~/.zshrc && \
    echo '# 机械臂仿真别名' >> ~/.zshrc && \
    echo 'alias gz-sim="gz sim"' >> ~/.zshrc && \
    echo 'alias gz-gui="gz sim -g"' >> ~/.zshrc && \
    echo 'alias moveit-demo="ros2 launch moveit_resources_panda_moveit_config demo.launch.py"' >> ~/.zshrc && \
    echo 'alias rmcs-display="ros2 launch rmcs_description display.launch.py"' >> ~/.zshrc


# Copy environment setup scripts
COPY --chown=1000:1000 .script/template/env_setup.bash /home/ubuntu/env_setup.bash
COPY --chown=1000:1000 .script/template/env_setup.zsh /home/ubuntu/env_setup.zsh


# ==========================================
# Runtime container
# ==========================================
FROM rmcs-base AS rmcs-runtime

# Install runtime tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends tini openssh-server avahi-daemon orphan-sysvinit-scripts && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* && \
    echo 'Port 2022' >> /etc/ssh/sshd_config && \
    echo 'PermitRootLogin yes' >> /etc/ssh/sshd_config && \
    echo 'PasswordAuthentication no' >> /etc/ssh/sshd_config && \
    sed -i 's/#enable-dbus=yes/enable-dbus=no/g' /etc/avahi/avahi-daemon.conf

# Install oh my zsh and setup environment
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/plugins=(git)/plugins=()/g' ~/.zshrc && \
    sed -i "s/# zstyle ':omz:update' mode disabled/zstyle ':omz:update' mode disabled/g" ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc && \
    echo 'export PATH=${PATH}:/rmcs_install/lib/rmcs_cli' >> ~/.zshrc && \
    echo 'export GZ_VERSION=harmonic' >> ~/.zshrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=/usr/share/gazebo/models' >> ~/.zshrc && \
    chsh -s /bin/zsh root

RUN mkdir -p /rmcs_install/

COPY --chown=root:root .script/set-robot /usr/local/bin/set-robot
COPY --chown=root:root .script/template/set-hostname /usr/local/bin/set-hostname

COPY --chown=root:root .script/template/entrypoint /entrypoint
COPY --chown=root:root .script/template/rmcs-service /etc/init.d/rmcs

COPY --from=rmcs-develop --chown=root:root /home/ubuntu/.ssh/id_rsa.pub /root/.ssh/authorized_keys

WORKDIR /root/
COPY --chown=root:root .script/template/env_setup.bash env_setup.bash
COPY --chown=root:root .script/template/env_setup.zsh env_setup.zsh

ENTRYPOINT ["tini", "--"]
CMD [ "/entrypoint" ]

# ========================================
# 环境变量
# ========================================
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GZ_VERSION=harmonic

# ========================================
# 元数据标签
# ========================================
LABEL maintainer="2543022868@qq.com"
LABEL description="RMCS + 机械臂仿真融合镜像 - ROS2 Humble + Gazebo Harmonic + MoveIt 2"
LABEL version="2.0.0"
LABEL ros.distro="humble"
LABEL gazebo.version="harmonic"
LABEL project="rmcs-arm-simulation"