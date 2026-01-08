# --- 构建阶段 ---
FROM ros:jazzy-ros-base AS builder

WORKDIR /ros_ws

# 复制服务端源码
COPY src/chat_server src/chat_server
COPY src/chat_interfaces  src/chat_interfaces

# 初始化 rosdep
RUN rosdep init || echo "rosdep already initialized" \
    && rosdep update

# 此时，接口已经作为基础层存在，无需再次复制chat_interfaces
# 直接安装依赖并编译
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select chat_interfaces chat_server

# --- 运行阶段 ---
FROM ros:jazzy-ros-core AS runtime

WORKDIR /ros_ws

# 从builder复制编译好的应用和它所依赖的接口
COPY --from=builder /ros_ws/install ./install

# 设置启动命令
CMD ["bash"]
