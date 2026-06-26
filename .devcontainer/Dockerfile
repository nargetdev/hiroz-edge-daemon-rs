FROM mcr.microsoft.com/devcontainers/rust:1-bookworm

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        clang \
        cmake \
        gphoto2 \
        libclang-dev \
        libgphoto2-dev \
        libudev-dev \
        pkg-config \
        usbutils \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

USER vscode

