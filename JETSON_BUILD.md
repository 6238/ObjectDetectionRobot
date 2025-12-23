# Jetson Vision Coprocessor: Architecture & Build Guide

This document explains the architecture, git structure, and Docker-based deployment strategy for the FRC 6238 Jetson vision system.

## 1. Repository Structure
The vision code is maintained as a separate repository to keep development isolated from the main robot code.

- **Git Submodule**: The `ObjectDetectionCoprocessor` directory is a git submodule.
- **Handling Submodules**: When cloning the robot code for the first time, you must run:
  ```bash
  git submodule update --init --recursive
  ```
- **Why?**: This ensures that different teams can work on vision and robot code independently while keeping a specific "known-good" version of the vision code linked to the main robot project.

## 2. Remote Build Architecture (Docker Contexts)
We use **Remote Docker Contexts** to build the image.
- **The Workflow**: You run `./gradlew deploy-jetson` on your laptop. Gradle tells your local Docker client to send the "build context" (source code and libraries) to the Jetson over SSH.
- **Why?**: 
  - **Native Performance**: Building directly on the Jetson's ARM64 processor is faster and more reliable than emulating ARM64 on an Intel/Apple Silicon Mac.
  - **Zero Setup**: Students don't need to install CUDA, TensorRT, or OpenCV on their laptops. Only Docker is required.

## 3. The "Tarball Injection" Strategy
To ensure the Docker container matches the physical Jetson hardware perfectly, we use "Tarball Injection."

- **The Problem**: NVIDIA's L4T (Linux for Tegra) drivers and certain libraries (OpenCV 4.10.0, CUDA stubs) are difficult to install via standard `apt-get` inside a container because they require hardware-level access during installation.
- **The Solution**: We "bundle" the known-good libraries from a working Jetson host into two compressed files:
  1. `opencv_custom_arm64.tar.gz`: The custom-built OpenCV 4.10.0 with CUDA support.
  2. `nvidia_runtime_libs.tar.gz`: The 1.2GB "Golden" bundle of NVIDIA drivers, TensorRT, and system math libraries (`libcblas`, `libtbb`, etc.).
- **Why?**: This guarantees that the version of OpenCV and CUDA running in the container is **bit-for-bit identical** to the version running on the host Jetson.

## 4. Building and Deploying
The main robot `build.gradle` includes a `deploy-jetson` task.
```bash
./gradlew deploy-jetson
```
This triggers a `docker build` command that uses the `ObjectDetectionCoprocessor/Dockerfile` to compile the C++ `inference_app` using Clang-15 and the injected libraries.

## 5. Runtime Configuration (The "Why" of the Run Command)
To run the container with GPU and camera access, use the following command:
```bash
docker run --rm -it \
    --runtime nvidia \
    --shm-size=1g \
    --ulimit memlock=-1 \
    --device /dev/video0:/dev/video0 \
    jetson-vision
```

### Explaining the Flags:
- `--runtime nvidia`: **CRITICAL**. Tells Docker to use the NVIDIA Container Runtime, allowing the container to access the Jetson's GPU.
- `--shm-size=1g`: Increases Shared Memory. TensorRT requires significant shared memory to load the engine files; without this, the app will crash with an "Out of Memory" error.
- `--ulimit memlock=-1`: Removes the limit on locked memory. This allows the GPU drivers to pin memory pages, which is required for high-performance DMA transfers between the CPU and GPU.
- `--device /dev/video0:/dev/video0`: Maps the physical USB/CSI camera into the container. Without this, OpenCV will report "Failed to open camera."
- `--rm -it`: Automatically removes the container after it stops and allows for interactive terminal input (so you can press ESC to quit).

## 6. Summary of Key Files
- `Dockerfile`: Defines the two-stage build (Builder stage with compilers -> Runtime stage with only the binary and libraries).
- `CMakeLists.txt`: Configured with `-rpath-link` to force the linker to find the injected libraries during compilation.
- `reefscape.engine`: The optimized TensorRT model file.

## 7. Internet & Connectivity Requirements

### The First Build (Internet Required)
The very first time you run `./gradlew deploy-jetson`, the Jetson **must** have a high-speed internet connection.
- **Base Image**: Docker will pull the `nvcr.io/nvidia/l4t-jetpack` image (approx. 3-5 GB).
- **System Packages**: `apt-get` will download around 200MB of compilers and math libraries.
- **Gradle & Dependencies**: The build will download the Gradle distribution and any external C++ libraries (like Google Test and WPILib headers) defined in `CMakeLists.txt`.

### Subsequent Builds (Offline Capable)
Once the first build is successful, Docker "caches" every step. 
- **No Internet Needed**: As long as you don't change the `Dockerfile` or the `FetchContent` sections of `CMakeLists.txt`, you can rebuild and redeploy your code at a competition without an internet connection.
- **Radio Connectivity**: Your laptop only needs to be on the same local network (Robot Radio or Shop Wi-Fi) as the Jetson to send the code updates.

### Summary Table
| Phase | Internet Needed? | Reason |
|-------|------------------|--------|
| **Initial Setup** | **YES** | Pulling base image & apt packages |
| **Code Changes** | **NO** | Re-using cached layers & local source |
| **Adding new `apt` packages** | **YES** | Downloading new software |
| **Competition Deployment** | **NO** | Local SSH transfer to Jetson |

