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
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras

# La imagen base puede traer Navigation2 actualizado pero una version anterior
# de diagnostic_updater sin la biblioteca C++ que lifecycle_manager carga en
# tiempo de ejecucion. En ese estado Nav2 inicia sus procesos pero todos quedan
# "unconfigured" y no publica NavigateThroughPoses. Forzar la actualizacion
# del paquete y verificar el artefacto convierte ese fallo silencioso en un
# build reproducible.
RUN apt-get update \
  && apt-get install -y --only-upgrade ros-${ROS_DISTRO}-diagnostic-updater \
  && test -f /opt/ros/${ROS_DISTRO}/lib/libdiagnostic_updater.so

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
    "pytest>=8.0,<9"

RUN rosdep init || true \
  && rosdep update

# PAQUETES EXTRA
RUN apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-pointcloud-to-laserscan \
      ros-${ROS_DISTRO}-nav2-rviz-plugins \
      libpcap-dev \
      libyaml-cpp-dev
RUN rm -rf /var/lib/apt/lists/*

# ============================================================================
# Planificador agricola de cobertura: Fields2Cover + Coverage Server de OpenNav
# ============================================================================
# Lo usa exclusivamente el modo Campo, con coverage_planner=fields2cover. Ruta,
# patrulla y goals no lo tocan.
#
# Ninguno de los dos tiene binario para Humble, asi que se compilan aca y no a
# mano dentro de un contenedor: un contenedor nuevo tiene que arrancar con esto
# disponible.
#
# Los commits van fijados a proposito. Son las versiones verificadas contra este
# stack; dejarlos flotando en una rama haria que un cambio upstream alterara el
# comportamiento del planificador sin que nadie lo pida.
ARG FIELDS2COVER_COMMIT=8bf28f7c5b6d8b8a2688aaa8fb1ac426e5c2d942
ARG OPENNAV_COVERAGE_COMMIT=ad914562a4e3f10892c84ad105d225d1284c61d3
ENV SALUS_COVERAGE_WS=/opt/salus_coverage_ws

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
      libgeos++-dev \
      libgeos-dev \
      libgdal-dev \
      libeigen3-dev \
      libtbb-dev \
      nlohmann-json3-dev \
      libtinyxml2-dev \
  && mkdir -p ${SALUS_COVERAGE_WS}/src \
  && cd ${SALUS_COVERAGE_WS}/src \
  # Se clona y se hace checkout del commit exacto en vez de --branch: una rama
  # avanza, un commit no.
  && git clone https://github.com/Fields2Cover/Fields2Cover.git \
  && git -C Fields2Cover checkout --quiet ${FIELDS2COVER_COMMIT} \
  && git clone https://github.com/open-navigation/opennav_coverage.git \
  && git -C opennav_coverage checkout --quiet ${OPENNAV_COVERAGE_COMMIT} \
  # Solo se compilan los tres componentes verificados. El navigator y el
  # backport del bt_navigator quedan fuera a proposito: ese backport reemplaza
  # el bt_navigator de Nav2, que es de TODAS las misiones de SALUS y no solo de
  # Campo. Row coverage y las demos no se usan.
  && for ignorado in backported_bt_navigator opennav_coverage_bt \
                     opennav_coverage_demo opennav_coverage_navigator \
                     opennav_row_coverage; do \
       touch opennav_coverage/$ignorado/COLCON_IGNORE; \
     done \
  && cd ${SALUS_COVERAGE_WS} \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build \
      --packages-select fields2cover opennav_coverage_msgs opennav_coverage \
      --cmake-args -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF \
                   -DBUILD_PYTHON=OFF -DBUILD_DOC=OFF \
  # Las fuentes y los artefactos intermedios no hacen falta en tiempo de
  # ejecucion y son la mayor parte del peso.
  && rm -rf ${SALUS_COVERAGE_WS}/src ${SALUS_COVERAGE_WS}/build ${SALUS_COVERAGE_WS}/log \
  && rm -rf /var/lib/apt/lists/*

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
