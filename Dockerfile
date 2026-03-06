FROM ubuntu:24.04 AS osrm-builder

RUN apt-get update && apt-get install -y \
    build-essential cmake git \
    libboost-all-dev \
    libtbb-dev \
    liblua5.4-dev \
    libluajit-5.1-dev \
    libxml2-dev \
    libzip-dev \
    libbz2-dev \
    libexpat1-dev \
    libstxxl-dev \
    libosmium2-dev \
    libprotozero-dev \
    libzmq3-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY CMakeLists.txt .
COPY osrm_wrapper.cpp .
COPY parsers.cpp .
COPY *.hpp .

RUN cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel $(nproc)
