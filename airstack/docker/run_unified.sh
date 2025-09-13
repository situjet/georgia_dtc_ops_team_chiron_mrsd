#!/usr/bin/env bash
set -euo pipefail

# Launch a unified container on NVIDIA Orin NX using the known-good AirStack L4T image,
# mounting this repo's ROS workspace and using host networking and NVIDIA runtime.

REPO_ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
ROS_WS_HOST="${REPO_ROOT_DIR}/ros_ws"

# Resolve local image to use (no pulling). Prefer the running 'airstack-robot_l4t-1' container's image, else any local *_robot-l4t image
PREFERRED_CONTAINER="airstack-robot_l4t-1"
if docker ps -a --format '{{.Names}}' | grep -q "^${PREFERRED_CONTAINER}$"; then
	IMAGE=$(docker inspect -f '{{.Config.Image}}' ${PREFERRED_CONTAINER} 2>/dev/null || true)
fi
if [ -z "${IMAGE:-}" ]; then
	# fallback: find any local image tagged *_robot-l4t
	IMAGE=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep -E '_robot-l4t$' | head -n1 || true)
fi
if [ -z "${IMAGE:-}" ]; then
	# final fallback: use env vars if the image exists locally
	IMAGE_REGISTRY="${PROJECT_DOCKER_REGISTRY:-airlab-storage.andrew.cmu.edu:5001/shared}"
	IMAGE_NAME="${PROJECT_NAME:-airstack}"
	IMAGE_TAG="v${PROJECT_VERSION:-1.0.3}_robot-l4t"
	CANDIDATE_IMAGE="${IMAGE_REGISTRY}/${IMAGE_NAME}:${IMAGE_TAG}"
	if docker image inspect "${CANDIDATE_IMAGE}" >/dev/null 2>&1; then
		IMAGE="${CANDIDATE_IMAGE}"
	fi
fi
if [ -z "${IMAGE:-}" ]; then
	echo "ERROR: No local L4T AirStack image found. Please ensure an '*_robot-l4t' image exists locally." >&2
	exit 1
fi

CONTAINER_NAME="airstack-unified"

echo "Using image: ${IMAGE}"

# Ensure image exists locally
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
	echo "ERROR: Image ${IMAGE} not found locally (and pulling is disabled)." >&2
	exit 1
fi

# Ensure ROS workspace exists
if [ ! -d "${ROS_WS_HOST}" ]; then
	echo "ROS workspace not found at ${ROS_WS_HOST}" >&2
	exit 1
fi

# Allow X11 if available
XSOCK="/tmp/.X11-unix"
XAUTH_OPTION=()
if [ -d "$XSOCK" ]; then
	XAUTH_OPTION=( -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=${DISPLAY:-:0} -e QT_X11_NO_MITSHM=1 )
fi

# Stop existing container if running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
	echo "Stopping existing ${CONTAINER_NAME}..."
	docker stop "${CONTAINER_NAME}" >/dev/null
fi

# Remove existing container if exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
	echo "Removing old ${CONTAINER_NAME}..."
	docker rm "${CONTAINER_NAME}" >/dev/null
fi

# Sanitize robot name for ROS namespace (allow only [A-Za-z0-9_])
HOST_ROBOT_NAME=${ROBOT_NAME:-robot_1}
ROBOT_NAME_SANITIZED=$(echo "$HOST_ROBOT_NAME" | tr -c '[:alnum:]_' '_')

# Run container
echo "Starting ${CONTAINER_NAME}..."
docker run -d \
	--entrypoint "" \
	--name ${CONTAINER_NAME} \
	--runtime nvidia \
	--network host \
	--privileged \
	-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-70} \
		-e ROBOT_NAME=${ROBOT_NAME_SANITIZED} \
		-e ROBOT_NAMESPACE=${ROBOT_NAME_SANITIZED} \
	"${XAUTH_OPTION[@]}" \
	-v "${ROS_WS_HOST}":/root/ros_ws:rw \
	-v /var/run/docker.sock:/var/run/docker.sock \
	${IMAGE} \
	bash --noprofile --norc -lc "set -e; \
		service ssh restart; \
		source /opt/ros/humble/setup.bash; \
		if [ ! -f /root/ros_ws/install/share/robot_bringup/package.xml ]; then \
			echo 'Rebuilding workspace (robot_bringup not installed or merge-install missing)...'; \
			cd /root/ros_ws && rm -rf build install log && colcon build --symlink-install --merge-install; \
		fi; \
		unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH; \
		source /root/ros_ws/install/setup.bash; \
		env | egrep 'AMENT_PREFIX_PATH|COLCON_PREFIX_PATH|CMAKE_PREFIX_PATH' || true; \
		ros2 pkg list | grep robot_bringup; \
		ros2 launch robot_bringup robot.launch.xml"

echo "Container ${CONTAINER_NAME} started. Attaching logs..."
docker logs -f ${CONTAINER_NAME}

