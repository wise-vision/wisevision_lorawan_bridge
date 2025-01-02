# Prerequisites

### Install Paho MQTT Cpp library from source
```bash
sudo apt install libpaho-mqtt-dev libpaho-mqttpp-dev
```

### Setup Chirpstack
1. Clone Chirpstack repository with docker scripts.
```bash
git clone https://github.com/chirpstack/chirpstack-docker.git
```

2. Replace configuration files for Chirpstack backend and EU868 region configuration.
```bash
cp chirpstack_configuration/chirpstack.toml chirpstack-docker/configuration/chirpstack/chirpstack.toml
cp chirpstack_configuration/region_eu868.toml chirpstack-docker/configuration/chirpstack/region_eu868.toml
```

> [!NOTE]
> For more detailed information about Chirpstack check official documentation:
https://www.chirpstack.io/docs/getting-started/docker.html

### Build packer_forwarder
1. Clone repository with gateway packet forwarder.
```bash
git clone https://github.com/Lora-net/sx1302_hal.git
```

2. Build binaries.
```bash
cd sx1302_hal
make
```

<!-- TODO(styczen): Move to separate file -->
### Build gRPC with Protocol Buffers from source
1. Define gRPC installation directory and export variable.
```bash
export GRPC_INSTALL_DIR=$HOME/grpc_install_dir
mkdir $GRPC_INSTALL_DIR
```

2. Make sure that there is CMake version 3.13 or later. Try installing from package:
```bash
sudo apt install -y cmake
```
If CMake version is to low, install it from source.

3. Install other required packages:
```bash
sudo apt install -y build-essential autoconf libtool pkg-config
```

4. Clone gRPC repository.
```bash
git clone --recurse-submodules -b v1.64.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc
```

5. Build and install gRPC and Protocol Buffers.
```bash
cd grpc
mkdir -p cmake/build
pushd cmake/build
cmake -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=$GRPC_INSTALL_DIR \
    -DgRPC_BUILD_GRPC_CPP_PLUGIN=ON \
    -DgRPC_BUILD_GRPC_CSHARP_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_NODE_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_PHP_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_PYTHON_PLUGIN=OFF \
    -DgRPC_BUILD_GRPC_RUBY_PLUGIN=OFF \
    ../..
make -j 4
make install
popd
```

6. Export variables to be able to load built shared libraries and include headers. It is recommended to put those variables inside `.bashrc` file.
```bash
export GRPC_INSTALL_DIR=${HOME}/grpc_install_dir
export PATH=$PATH:${GRPC_INSTALL_DIR}/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${GRPC_INSTALL_DIR}/lib
```

# Setup
1. Download all dependencies.
```bash
vcs import < dependencies.repos
```

2. Navigate to ROS 2 workspace and build.
```bash
colcon build
```

# How to run
1. Navigate to this repository and start Chirpstack stack.
```bash
cd chirpstack-docker
docker compose up -d
```

2. Source workspace.
```bash
cd ..
source install/local_setup.bash
```

3. Generate Chirpstack API token and store it in `CHIRPSTACK_API_TOKEN` environment variable.
TODO(styczen)
3.1. Open Chirpstack dashboard panel by going to `localhost:8080` in browser.

4. Start LoRaWAN bridge (two ways to do it).

> [!NOTE]
> Read `APPLICATION_ID` from Chirpstack web UI.

> [!NOTE]
> Change `broker.host` and `broker.port` parameters if MQTT broker is deployed on different host.

- start bridge in components container
  - start component container
  ```bash 
  ros2 run rclcpp_components component_container
  ```
  - load bridge component
  ```bash
  ros2 comk 
  ```
- start bridge as standalone node
```bash
ros2 run wisevision_lorawan_bridge lorawan_bridge --ros-args --param application_id:=APPLICATION_ID
```

