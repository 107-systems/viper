#!/bin/bash
HELP="NAME
     build.sh – Unified ROS 2 workspace compilation tool

SYNOPSIS
     ./build.sh [--arm] [--clean] [package_name]

DESCRIPTION
     build.sh is a wrapper script designed to streamline the compilation logic
     of a local ROS 2 workspace. It manages underlying environment sourcings,
     handles build directory purification, and provides an isolated Docker/QEMU
     cross-compilation pipeline targeting AARCH64 (ARM64) platforms directly
     from an x86_64 host architecture.

     The script must be executed from the root directory of a valid ROS 2 
     workspace containing a src/ folder. It explicitly blocks execution if a 
     package.xml file is detected in the current directory to prevent local 
     package pollution.

OPTIONS
     --arm    Switches the compilation pipeline from native x86_64 to target 
              AARCH64 emulation. This spins up the local 'ros-humble-aarch64' 
              Docker container, binds the current working directory to internal 
              mount paths, and triggers the colcon build isolated from host 
              binary definitions.

     --clean  Forces a hard reset of compilation cache before starting. If run 
              with --arm, permissions are assessed via sudo to remove Docker-
              owned root artifacts safely. Deletes the following directory nodes:
              - build/
              - install/

     package_name
              Specifies a singular target package. When provided, the script 
              passes the name straight to colcon via the --packages-select flag, 
              bypassing a full-workspace build sequence.

EXIT STATUS
     0        Successful tool execution and package compilation.
     1        Failure occurred due to environment restrictions (e.g., executing 
              outside a workspace root) or a fatal compiler fault down-stream.

EXAMPLES
     Ex 1. Standard full-workspace native host build:
           $ ./build.sh

     Ex 2. Isolate compilation to a specific driver package:
           $ ./build.sh pika_spark_bno085_driver

     Ex 3. Purge cache and compile the workspace for an embedded ARM target:
           $ ./build.sh --arm --clean

     Ex 4. Clean and compile a single package for an embedded ARM target:
           $ ./build.sh --arm --clean viper
"

if [ "$1" == '--help' ] || [ "$1" == '-h' ]; then echo "$HELP"; exit; fi

REQUIRED_IMAGE="ros-humble-aarch64"

# 1. Verify this is the root of a ROS 2 workspace
if [ ! -d "src" ]; then
    echo "Error: This script must be run from the root of a ROS 2 workspace (could not find a 'src/' directory)."
    exit 1
fi

if [ -f "package.xml" ]; then
    echo "Error: Found a 'package.xml' in the current directory."
    echo "       You are likely inside a specific ROS 2 package, not the workspace root."
    exit 1
fi

# 2. Parse arguments
BUILD_MODE="x86"
TARGET_PACKAGE=""
CLEAN_BUILD=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --arm)
            BUILD_MODE="arm"
            shift
            ;;
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        *)
            TARGET_PACKAGE=$1
            shift
            ;;
    esac
done

# 3. Handle Workspace Cleaning if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo "=== Cleaning Workspace (Removing build, install, log) ==="
    # Use sudo to clean out any old root-owned artifacts safely
    sudo rm -rf build/ install/ log/
fi

# 4. Configure colcon package filtering
PACKAGES_FLAG=""
if [ -n "$TARGET_PACKAGE" ]; then
    PACKAGES_FLAG="--packages-select $TARGET_PACKAGE"
fi

# 5. Execute based on selected architecture mode
if [ "$BUILD_MODE" = "arm" ]; then
    echo "=== Running AARCH64 Cross-Compilation Build (Docker) ==="
    
    # Check if the required Docker image exists locally
    if ! docker image inspect "$REQUIRED_IMAGE" >/dev/null 2>&1; then
        echo "Error: Need to build the '$REQUIRED_IMAGE' image first."
        exit 1
    fi

    if [ -n "$TARGET_PACKAGE" ]; then
        echo "Building only package: $TARGET_PACKAGE"
    else
        echo "Building entire workspace..."
    fi

    # Run the ARM compiler container mapped to your local host UID/GID
    docker run --rm \
        --user "$(id -u):$(id -g)" \
        -v "$PWD":/tmp/ws \
        "$REQUIRED_IMAGE" \
        bash -c "cd /tmp/ws && source /opt/ros/humble/setup.bash && colcon build $PACKAGES_FLAG"

else
    echo "=== Running Native x86_64 Build (Local Machine) ==="
    
    # Preemptively fix local permissions if an old root-run left artifacts behind
    if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
        if [ ! -w "build" ] 2>/dev/null || [ ! -w "install" ] 2>/dev/null; then
            echo "Detected root-owned artifacts. Resetting workspace permissions..."
            sudo chown -R $USER:$USER build/ install/ log/
        fi
    fi

    if [ -n "$TARGET_PACKAGE" ]; then
        echo "Building only package: $TARGET_PACKAGE"
    else
        echo "Building entire workspace..."
    fi

    # Run local build natively
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    colcon build $PACKAGES_FLAG
fi