# syntax=docker/dockerfile:1
ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}-ros-base AS upstream
# Restate for later use
ARG ROS_DISTRO
ARG REPO

# prevent interactive messages in apt install
ENV DEBIAN_FRONTEND=noninteractive
# Set bash options to deter silent failures 
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# install development tools
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
        apt-transport-https \
        apt-utils \
        ccache \
        clang \
        cmake \
        curl \
        git \
        git-lfs \
        gnupg2 \
        lld \
        llvm \
        python3-pip \
        python-is-python3 \
        ros-jazzy-rviz2 \
        software-properties-common \
        wget \
    && rm -rf /var/lib/apt/lists/*

# copy source to install repo dependencies
WORKDIR /ws
COPY . ./src
# install repo dependencies
RUN rosdep update && apt-get update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

FROM upstream AS development

ARG UID
ARG GID
ARG USER

# fail build if args are missing
# hadolint ignore=SC2028
RUN if [ -z "$UID" ]; then echo '\nERROR: UID not set. Run \n\n \texport UID=$(id -u) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$GID" ]; then echo '\nERROR: GID not set. Run \n\n \texport GID=$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
# hadolint ignore=SC2028
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# install developer tools
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,id=apt \
    apt-get update && apt-get upgrade -y \
    && apt-get install -q -y --no-install-recommends \
        clang-format \
        clang-tidy \
        clangd \
        git \
        neovim \
        openssh-client \
        socat \
        sudo \
        ssh \
        vim \
        wget \
    && rm -rf /var/lib/apt/lists/*

# install developer tools
RUN python3 -m pip install --no-cache-dir --break-system-packages \
  pre-commit==4.0.1

# install hadolint
RUN wget -q -O /bin/hadolint https://github.com/hadolint/hadolint/releases/download/v2.12.0/hadolint-Linux-x86_64 \
  && chmod +x /bin/hadolint

# Setup user home directory
RUN usermod -l $USER ubuntu \
    && groupmod -n $USER ubuntu \
    && usermod -d /home/$USER -m $USER \
    && echo "$USER ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USER \
    && chmod 0440 /etc/sudoers.d/$USER \
    && touch /home/$USER/.bashrc \
    && chown -R $UID:$GID /home/$USER

USER $USER
ENV SHELL /bin/bash
ENTRYPOINT []

RUN mkdir ~/.gazebo

# Setup mixin
WORKDIR /home/${USER}/ws
