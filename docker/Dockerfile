FROM ubuntu:focal

RUN \
    export DEBIAN_FRONTEND=noninteractive && \
    export TZ=Etc/UTC && \
    apt-get update && \
    apt-get upgrade --yes && \
    apt-get install --yes \
        curl \
        htop \
        less \
        libfftw3-dev \
        libopencv-dev \
        vim-tiny

# This script installs ROS and the needed packages without
# having to escape every command in the Dockerfile.
COPY install_ros.sh .
RUN ./install_ros.sh

# Running possibly hardware accelerated programs (like rviz)
# often fails when using X11 forwarding.
ENV LIBGL_ALWAYS_SOFTWARE=1

# The entrypoint script sources either the ROS installation
# or the workspace (if it is already build).
COPY entrypoint.sh .
ENTRYPOINT ["./entrypoint.sh"]
CMD ["bash"]
