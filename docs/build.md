# Build Instructions
There may be better and faster build options with using docker's buildx. Checkout this [guide](https://www.docker.com/blog/faster-multi-platform-builds-dockerfile-cross-compilation-guide/)
## Option 1 - Build in Docker Container with QEMU emulation
This method requires QEMU to emulate the arm64 architecture. Install using this command:
```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```
> [!Tip]
> Check architecture of current system using `uname -m`

1. Build ROS Humble Docker dev image:
```bash
docker build -f ./docker/host-dev/Dockerfile -t ros-humble-aarch64 .
```


2. Build:
```bash
sudo docker build -t ros-humble-aarch64 .
```

3. Mount the volume to Docker when running the image. This command starts a container, builds the program, then deletes the container.
```bash
# Must be run in root of ROS 2 workspace
docker run --rm \
    -v $PWD:/tmp/ws \
    ros-humble-aarch64 \
    bash -c "cd /tmp/ws && source /opt/ros/humble/setup.bash && colcon build [--packages-select package-name]"
```

4. After build, because volume is mounted, you can just push the install directory onto Pika Spark:
```bash
adb push install /home/pika
```
>[!Tip]
>Adding `/` after the directory will copy the stuff within that directory. No `/` is just the entire directory including `install`

> [!Note] 
> This may mess up any existing installs that use the x86 architecture. Probably a good idea to delete any existing `build` or `install` directories to avoid path issues.

5. If Pika Spark image has been reflashed, make sure it also has the same image on it:
```bash
# This saves the image as a file we can load onto Pika Spark
docker save ros-humble-aarch64 -o ros-humble-aarch64.tar  
# Load onto Pika Spark
sudo adb push ros-humble-aarch64.tar /home/pika

# Log into Pika Spark
adb shell
su  # Switch to root
# Load image
docker load -i ros-humble-aarch64.tar
# Start container for first time
docker run -it \  
	--name ros-humble-aarch64-container \  
	--net=host \  
	--privileged \  
	ros-humble-aarch64
```
6. If not first time, need to start the container: 
```bash
docker start ros-humble-aarch64
```
7. Copy the install directory into the container
```bash
docker cp install/ <container_name>:/tmp/colcon_ws/install
```
8. Attach to container and start the program
```bash
docker exec -it ros-humble-aarch64-container bash
# cd into the root of ROS 2 workspace
. /opt/ros/humble/setup.bash
. install/setup.bash
ros2 launch viper viper-quad.py
```

### Convenience Build Script
To save pasting that long command in step 3 just to build, run the [`build.sh`](../scripts/build.sh) script from ROS 2 workspace root to compile for x86 or ARM.

```bash
./build.sh [--arm] [--clean] [package_name]
```
- `--arm`: Switches the compilation pipeline from native x86_64 to target  AARCH64 emulation. This spins up the local 'ros-humble-aarch64' Docker container, binds the current working directory to internal mount paths, and triggers the colcon build isolated from host binary definitions.

- `--clean`: Forces a hard reset of compilation cache before starting. Add flag if previous build was for a different architecture so that build artifacts not mixed. Deletes the following directory nodes: 
  - `build/`
  - `install/`
  - `log/`

- `package_name`
        Specifies a singular target package. When provided, the script passes the name straight to colcon via the --packages-select flag, bypassing a full-workspace build sequence.
- `-h` | `--help`: Display help message

## Option 2: Build Directly on the Host
Use this if you just want to build on x86 and run on x86. Can also follow this to compile on Pika Spark, though it is excruciatingly slow.
```bash
mkdir -p colcon_ws/src && cd colcon_ws/src
git clone --recursive https://github.com/107-systems/viper
cd .. # Should be in viper-ws directory, i.e. ROS workspace root
source /opt/ros/humble/setup.bash
colcon build --packages-select viper
```