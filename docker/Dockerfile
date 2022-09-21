# The base image for this image.
ARG BASE_IMAGE

FROM $BASE_IMAGE

# The ROS version to install.
ARG ROS_VERSION

RUN \
    export DEBIAN_FRONTEND=noninteractive && \
    export TZ=Etc/UTC && \
    apt-get update && \
    apt-get upgrade --yes && \
    apt-get install --yes \
        curl \
        geeqie \
        graphicsmagick-imagemagick-compat \
        htop \
        less \
        libfftw3-dev \
        libopencv-dev \
        python3-pip \
        vim-tiny

RUN python3 -m pip install transforms3d

# This script installs ROS and the needed packages without
# having to escape every command in the Dockerfile.
COPY install_ros.sh .
RUN ./install_ros.sh "$ROS_VERSION"

# Running possibly hardware accelerated programs (like rviz)
# often fails when using X11 forwarding.
ENV LIBGL_ALWAYS_SOFTWARE=1

# The code in the bashrc file sources either the ROS installation
# or the workspace (if it is already build).
COPY bashrc /root/.bashrc
ENTRYPOINT ["bash"]
