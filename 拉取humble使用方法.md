1. 构建docker容器会报错的话可能是GPU的问题，可以尝试把/workspaces/RMCS/.devcontainer/devcontainer.json中的
		"--gpus",
		"all"
		"NVIDIA_VISIBLE_DEVICES": "all",
		"NVIDIA_DRIVER_CAPABILITIES": "all",
        这四行注释掉在构建docker容器
2. 在项目根目录（/workspaces/RMCS/）执行：
```bash
# 1. 创建 .ssh 目录
mkdir -p .ssh

# 2. 生成 SSH 密钥对（无密码）
ssh-keygen -t rsa -b 4096 -f .ssh/id_rsa -N ""

# 3. 创建空的 config 文件
touch .ssh/config

# 4. 验证文件已生成
ls -la .ssh/

执行后会看到
.ssh/
├── id_rsa          # 私钥（保密！）
├── id_rsa.pub      # 公钥（可以分享）
└── config          # SSH 配置文件
```

# 进入docker容器后
1. 先建github仓库,配置用户名和邮箱
2. 把RMCS连接到github的仓库
3. 将当前用户添加到 dialout 组
    sudo usermod -aG dialout $USER

    groups
    # 应该能看到 dialout 在列表中

4. 记得先接受github仓库的邀请（接受一次即可）

    cd /workspaces/RMCS

    # 删除可能存在的空目录
    rm -rf rmcs_ws/src/fast_tf
    rm -rf rmcs_ws/src/rmcs_core/librmcs
    rm -rf rmcs_ws/src/hikcamera
    rm -rf rmcs_ws/src/serial

    # 重新初始化
    git submodule update --init --recursive
    如果不好使的话
    # 手动克隆每个子模块
    git clone https://github.com/qzhhhi/FastTF.git rmcs_ws/src/fast_tf
    git clone https://github.com/Starry10086/test_librmcs.git rmcs_ws/src/rmcs_core/librmcs
    git clone https://github.com/Alliance-Algorithm/ros2-hikcamera.git rmcs_ws/src/hikcamera
    git clone https://github.com/Alliance-Algorithm/ros2-serial.git rmcs_ws/src/serial