FROM ubuntu:24.04 AS osrm-build
RUN apt-get update && apt-get install -y \
    build-essential cmake pkg-config curl \
    libbz2-dev libxml2-dev libzip-dev libexpat1-dev \
    libboost-all-dev liblua5.4-dev libtbb-dev \
    && rm -rf /var/lib/apt/lists/*
RUN curl -L https://github.com/Project-OSRM/osrm-backend/archive/v26.4.1.tar.gz \
    | tar -xz \
    && cd osrm-backend-26.4.1 \
    && mkdir -p build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_FLAGS='-include unistd.h -Wno-error=unused-variable -Wno-unused-variable' \
        -DCMAKE_CXX_FLAGS='-include unistd.h -Wno-error=unused-variable -Wno-unused-variable' \
    && make -j$(nproc) \
    && make install \
    && cd ../.. \
    && rm -rf osrm-backend-26.4.1

FROM ubuntu:24.04 AS osrm-builder
RUN apt-get update && apt-get install -y \
    build-essential cmake pkg-config \
    libboost-all-dev liblua5.4-dev libtbb-dev \
    && rm -rf /var/lib/apt/lists/*
COPY --from=osrm-build /usr/local /usr/local
WORKDIR /build
COPY CMakeLists.txt .
COPY osrm_wrapper.cpp .
COPY parsers.cpp .
COPY *.hpp .
RUN cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel $(nproc)