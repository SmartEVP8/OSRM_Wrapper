FROM ubuntu:24.04 AS osrm-builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libbz2-dev \
    libxml2-dev \
    libzip-dev \
    libexpat1-dev \
    libboost-all-dev \
    liblua5.4-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY CMakeLists.txt .
COPY osrm_wrapper.cpp .
COPY parsers.cpp .
COPY *.hpp .

RUN cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel $(nproc)
