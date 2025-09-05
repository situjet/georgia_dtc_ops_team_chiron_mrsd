# Unified Docker README

## Overview
This README provides clear instructions for modifying and making changes to the repository when using the new unified Docker setup. It is intended for developers and contributors working with the `airstack/docker/Dockerfile.unified` and related files.

---

## Table of Contents
1. [Repository Structure](#repository-structure)
2. [Unified Docker Overview](#unified-docker-overview)
3. [Modifying the Dockerfile](#modifying-the-dockerfile)
4. [Building the Docker Image](#building-the-docker-image)
5. [Running the Docker Container](#running-the-docker-container)
6. [Making Code Changes](#making-code-changes)
7. [Testing Your Changes](#testing-your-changes)
8. [Best Practices](#best-practices)
9. [Troubleshooting](#troubleshooting)

---

## Repository Structure
- `airstack/docker/Dockerfile.unified`: Main Dockerfile for the unified environment.
- `airstack/ros_ws/`: ROS workspace containing source code and build scripts.
- `airstack/ros_ws/build.sh`: Script to build the ROS workspace.
- `airstack/ros_ws/entrypoint.sh`: Entrypoint script for the Docker container.
- `airstack/ros_ws/src/`: Source code for ROS packages and other modules.

---

## Unified Docker Overview
The unified Docker setup provides a consistent environment for development, testing, and deployment. All dependencies and build steps are encapsulated in the Dockerfile, ensuring reproducibility across different systems.

---

## Modifying the Dockerfile
1. **Location:** Edit `airstack/docker/Dockerfile.unified` to change the base image, install new packages, or modify environment variables.
2. **Adding Dependencies:**
   - For system packages, add `RUN apt-get install ...` lines.
   - For Python packages, use `RUN pip install ...`.
   - For ROS packages, use `RUN rosdep install ...` as needed.
3. **Custom Scripts:**
   - Place custom scripts in `airstack/ros_ws/` or a subdirectory, and copy them in the Dockerfile using `COPY`.

**Example:**
```dockerfile
# Install a new system package
RUN apt-get update && apt-get install -y <package-name>
```

---

## Building the Docker Image
1. Open a terminal in the root of the repository.
2. Navigate to the Docker directory:
   ```bash
   cd airstack/docker
   ```
3. Build the Docker image:
   ```bash
   docker build -f Dockerfile.unified -t <image-name>:<tag> .
   ```
   Replace `<image-name>` and `<tag>` as desired (e.g., `chiron:latest`).

---

## Running the Docker Container
1. Jetson Orin NX (recommended): use the working AirStack L4T image and run:
   ```bash
   ./run_unified.sh
   ```
   This script pulls the ghcr.io/strapsai/airstack L4T image, mounts `../ros_ws`, builds if needed, and launches `robot_bringup`.
2. For custom local runs, you can still `docker run` manually as needed.

---

## Making Code Changes
- **Source Code:**
  - Edit files in `airstack/ros_ws/src/` as needed.
  - If using a mounted volume, changes on your host are reflected in the container.
- **Build Scripts:**
  - Modify `build.sh` or `entrypoint.sh` for custom build or startup logic.
- **ROS Packages:**
  - Add or update packages in `airstack/ros_ws/src/`.
  - Update `CMakeLists.txt` and `package.xml` as required.

---

## Testing Your Changes
1. **Build the Workspace:**
   Inside the container, run:
   ```bash
   cd /ros_ws
   ./build.sh
   ```
2. **Run Nodes or Launch Files:**
   Use ROS commands as needed, e.g.:
   ```bash
   roslaunch <package> <file.launch>
   ```
3. **Debugging:**
   - Use `echo`, `print`, or ROS logging for debugging.
   - Check logs in `/ros_ws/log` or as output in the terminal.

---

## Best Practices
- **Rebuild the Docker image** after changing the Dockerfile or adding system-level dependencies.
- **Use volumes** to avoid rebuilding the image for every code change.
- **Document changes** in code and update this README as needed.
- **Test in a clean container** to ensure reproducibility.

---

## Troubleshooting
- **Build Failures:**
  - Check the Docker build output for errors.
  - Ensure all dependencies are correctly specified.
- **Permission Issues:**
  - Use `chmod` to set correct permissions on scripts.
- **ROS Issues:**
  - Source the ROS setup script: `source /opt/ros/<distro>/setup.bash`.
  - Check environment variables and paths.

---

## Additional Resources
- [Docker Documentation](https://docs.docker.com/)
- [ROS Documentation](https://wiki.ros.org/)
- [Project README](../../README.md)

---

For further questions, contact the repository maintainers or open an issue.
