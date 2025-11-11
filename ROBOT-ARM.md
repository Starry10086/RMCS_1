# 在RMCS中实现硬件+Moveit协同规划

# 1.1 安装依赖
sudo apt update
sudo apt install -y build-essential cmake git libfreetype6-dev \
    libxrandr-dev libxaw7-dev freeglut3-dev libzzip-dev \
    libois-dev libboost-dev libboost-thread-dev

# 1.2 下载ogre
cd /tmp
git clone --branch v1.12.1 --depth 1 https://github.com/OGRECave/ogre.git ogre-1.12.1
cd ogre-1.12.1

# 1.3 编译并安装
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DOGRE_BUILD_COMPONENT_BITES=OFF \
    -DOGRE_BUILD_COMPONENT_JAVA=OFF \
    -DOGRE_BUILD_SAMPLES=OFF \
    -DOGRE_INSTALL_SAMPLES=OFF \
    -DOGRE_BUILD_TESTS=OFF

# 1.4 验证安装
ls -la /usr/lib/x86_64-linux-gnu/libOgreMain.so.1.12.1
echo "✓ Ogre 1.12.1 安装完成"


# 1.5 OMPL版本问题

# 1.检查当前版本，版本期望是1.6
ls -la /usr/local/lib/libompl* 2>/dev/null
dpkg -l | grep ompl

# 2.将目前使用的版本，指向目标版本（如果有的话）
sudo ln -sf /usr/lib/x86_64-linux-gnu/libompl.so.16 /usr/lib/x86_64-linux-gnu/libompl.so.18(你的)

# 3.重新编译启动

# 仿真文件配置修改位置说明
## robot_arm/config/ros2_controllers.yaml
修改在仿真中机械臂的轨迹跟踪和关节状态广播，机械臂控制器
以及各个joint的单次运行速度下限，时间还有关节限度

## robot_arm/urdf/robot_arm.urdf.xacro
用来开启仿真/硬件模式，具体在文件中有标注位置

## 关于运行
# 2.1 进入工作空间
cd /workspaces/RMCS/rmcs_ws

# 2.2 清理旧的编译文件（如果需要）
rm -rf build/ install/ log/

# 2.3 编译整个工作空间
colcon build --symlink-install

# 2.4 返回根目录
cd /workspaces/RMCS


#### 启动项目
# 1. 进入工作空间
cd /workspaces/RMCS

# 2. 清理残留进程和内存
echo "清理残留进程..."
pkill -9 -f "ros2_control_node" 2>/dev/null
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "rviz" 2>/dev/null

# 清理共享内存
ipcs -m | grep $USER | awk '{print $2}' | xargs -I {} ipcrm -m {} 2>/dev/null

# 4. USB权限    
sudo chmod 666 /dev/ttyUSB0

# 5. 启动系统
cd /workspaces/RMCS/rmcs_ws
colcon build --packages-select robot_arm
source install/setup.bash
ros2 launch robot_arm moveit.launch.py