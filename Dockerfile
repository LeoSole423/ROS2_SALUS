FROM ros:humble-perception

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV QT_X11_NO_MITSHM=1
ENV GZ_VERSION=fortress

ARG INSTALL_GAZEBO_SIM=auto

# Paquetes base de build + ROS + utilidades del stack
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    python3-yaml \
    python3-pytest \
    python3-serial \
    wget \
    curl \
    nano \
    vim \
    # --- NAVIGATION & LOCALIZATION ---
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-topic-tools \
    # --- ROBOT STATE & CONTROL ---
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ackermann-steering-controller \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-twist-stamper \
    # --- VISUALIZATION & TOOLS ---
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras

# Gazebo/ros_gz puede quedar fuera en ARM64 si la base tiene conflictos de
# dependencias. En la Jetson nos interesa priorizar el stack real del robot.
RUN arch="$(dpkg --print-architecture)" && \
  install_gazebo="${INSTALL_GAZEBO_SIM}" && \
  if [ "${install_gazebo}" = "auto" ]; then \
    if [ "${arch}" = "arm64" ]; then \
      install_gazebo="false"; \
    else \
      install_gazebo="true"; \
    fi; \
  fi && \
  if [ "${install_gazebo}" = "true" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-gz-ros2-control; \
  else \
    echo "Gazebo/ros_gz omitido para este build (${arch})."; \
  fi

# Mapviz: en amd64 hay binarios; en ARM64 se omite (headless).
RUN arch="$(dpkg --print-architecture)" && \
  if [ "$arch" = "amd64" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-mapviz \
      ros-${ROS_DISTRO}-mapviz-plugins \
      ros-${ROS_DISTRO}-tile-map \
      ros-${ROS_DISTRO}-multires-image; \
  else \
    echo "Mapviz omitido en ARM64 (entorno headless)."; \
  fi

# MAVROS requiere datasets de GeographicLib para GPS
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && ./install_geographiclib_datasets.sh \
  && rm install_geographiclib_datasets.sh

# Dependencias Python del proyecto
RUN python3 -m pip install --upgrade pip \
  && python3 -m pip install --no-cache-dir --force-reinstall \
    numpy==1.26.4 \
    flask==2.3.0 \
    matplotlib==3.7.0 \
    "websockets>=11.0.0" \
    onvif-zeep \
    pyserial==3.5 \
    pymavlink==2.4.43 \
    pytest>=8.0

RUN rosdep init || true \
  && rosdep update

# PAQUETES EXTRA
RUN apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-pointcloud-to-laserscan \
      ros-${ROS_DISTRO}-nav2-rviz-plugins \
      libpcap-dev \
      libyaml-cpp-dev
RUN rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
  && groupadd --gid 20 dialout || true \
  && usermod -aG dialout,tty ${USERNAME} \
  && mkdir -p /ros2_ws \
  && chown -R ${USERNAME}:${USERNAME} /ros2_ws

COPY entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

COPY .bashrc /home/ros/.bashrc
COPY mapviz_gps.mvc /home/ros/.mapviz_config
RUN chown ${USERNAME}:${USERNAME} /home/ros/.bashrc

USER ${USERNAME}
WORKDIR /ros2_ws


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
