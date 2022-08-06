# syntax=docker/dockerfile:1
FROM rust:1.62.1
ARG UIDGID
ARG USER

# fail build if args are missing
RUN if [ -z "$UIDGID" ]; then echo '\nERROR: UIDGID not set. Run \n\n \texport UIDGID=$(id -u):$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# Prevent the interactive wizards from stopping the build
ARG DEBIAN_FRONTEND=noninteractive

# Get the basics
RUN apt update -y &&       \
    apt install -y         \
        git                \
        sudo               \
        vim                \
        wget

RUN wget -O /tmp/nvim-linux64.deb https://github.com/neovim/neovim/releases/download/v0.7.0/nvim-linux64.deb && \
    apt install -y /tmp/nvim-linux64.deb

RUN git clone https://github.com/AstroNvim/AstroNvim /home/${USER}/.config/nvim

# Get rust tools
RUN rustup component add rustfmt clippy

# Get dependencies and download code
COPY . /tmp/ws
RUN cd /tmp/ws &&  \
    cargo build && \
    rm -rf /tmp/ws

# chown working directory to user
RUN mkdir -p /home/${USER}/ws && chown -R ${UIDGID} /home/${USER}
