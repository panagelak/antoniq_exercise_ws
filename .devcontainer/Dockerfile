FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
# Must match the host's `video`/`render` GIDs (see `getent group video render`)
# so the non-root user can open /dev/dri for GPU rendering.
ARG VIDEO_GID=44
ARG RENDER_GID=110

ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic is the Gazebo release paired with ROS 2 Jazzy; ros-jazzy-ros-gz
# pulls it in automatically alongside the ROS <-> Gazebo bridge/sim packages.
#
# ros-jazzy-turtlebot3 / -simulations bring in the robot description, teleop and
# the Gazebo spawn/bringup launch files; ros-jazzy-nav2-bringup pulls in the rest
# of navigation2 plus diff_drive_controller/joint_state_broadcaster, so ros2-control
# and ros2-controllers are added explicitly to cover ros2_control usage beyond Nav2.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-turtlebot3 \
        ros-jazzy-turtlebot3-simulations \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-twist-mux \
        ros-jazzy-control-toolbox \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        mesa-utils \
        x11-apps \
        git \
        curl \
        nano \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Non-root user with the same UID/GID as the host user, plus GPU device group access.
# The base image already ships a user/group at 1000:1000 (Ubuntu's default "ubuntu"
# account), so rename it instead of creating a new one when the UID/GID is taken.
RUN if getent group ${USER_GID} > /dev/null; then \
        groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1); \
    else \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi \
    && if getent passwd ${USER_UID} > /dev/null; then \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m -s /bin/bash $(getent passwd ${USER_UID} | cut -d: -f1); \
    else \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi \
    && (getent group ${VIDEO_GID} || groupadd -g ${VIDEO_GID} video) \
    && (getent group ${RENDER_GID} || groupadd -g ${RENDER_GID} render) \
    && usermod -aG sudo,video,render ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# NVIDIA Container Toolkit reads these to decide what to inject into the container.
# "all" capabilities gives Gazebo/OGRE2 both graphics (GLX/EGL) and compute (CUDA/OpenCL).
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# PRIME render offload: lets an OpenGL/Vulkan client pick the discrete NVIDIA GPU
# even though the host's X server is driven by the laptop's Intel iGPU.
ENV __NV_PRIME_RENDER_OFFLOAD=1 \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __VK_LAYER_NV_optimus=NVIDIA_only

RUN rosdep update || true

USER ${USERNAME}
WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> ~/.bashrc \
    && echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# Bake a build of the workspace into the image so `docker build && docker run` works
# standalone on a clean machine. In the VS Code dev container this source tree is
# shadowed by the bind mount in devcontainer.json, so rerun `colcon build` after the
# container starts to pick up local edits.
COPY --chown=${USERNAME}:${USERNAME} src/ /workspace/src/

RUN /bin/bash -c ". /opt/ros/jazzy/setup.bash \
    && sudo rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install"
