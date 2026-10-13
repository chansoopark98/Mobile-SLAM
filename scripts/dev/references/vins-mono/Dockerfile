FROM ros@sha256:5dd10cb3fdff1882137b979feca65c8e54082e5c6e8c78634d50ae0dc7e64372

LABEL org.opencontainers.image.source="https://github.com/HKUST-Aerial-Robotics/VINS-Mono" \
      org.opencontainers.image.revision="90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d" \
      mobile-slam.reference="vins-mono"

ENV DEBIAN_FRONTEND=noninteractive OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential libatlas-base-dev libeigen3-dev \
      libgoogle-glog-dev libgflags-dev libsuitesparse-dev \
      ros-kinetic-cv-bridge ros-kinetic-image-transport \
      ros-kinetic-message-filters ros-kinetic-tf && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch 1.12.0 https://github.com/ceres-solver/ceres-solver.git /opt/ceres-source && \
    test "$(git -C /opt/ceres-source rev-parse HEAD)" = "029799d757b4ed2be5af64899178928f18cb6e28" && \
    mkdir /opt/ceres-build && cd /opt/ceres-build && \
    cmake /opt/ceres-source -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
      -DSUITESPARSE=OFF -DCXSPARSE=OFF -DLAPACK=OFF && \
    make -j2 && make install && ldconfig

COPY sources/VINS-Mono/ /opt/vins_ws/src/VINS-Mono/
RUN source /opt/ros/kinetic/setup.bash && cd /opt/vins_ws && \
    catkin_make -j2 -l2 -DCMAKE_BUILD_TYPE=Release \
      '-DCATKIN_WHITELIST_PACKAGES=camera_model;feature_tracker;vins_estimator' && \
    dpkg-query -W > /opt/vins_ws/package-versions.txt

WORKDIR /opt/vins_ws
