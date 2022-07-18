#!/usr/bin/env bash

TAG="andrejorsula/ign_moveit2_examples"

if [ "${#}" -gt "0" ]; then
    if [[ $(docker images --format "{{.Tag}}" "${TAG}") =~ (^|[[:space:]])${1}($|[[:space:]]) || $(wget -q https://registry.hub.docker.com/v2/repositories/${TAG}/tags -O - | grep -Poe '(?<=(\"name\":\")).*?(?=\")') =~ (^|[[:space:]])${1}($|[[:space:]]) ]]; then
        # Use the first argument as a tag is such tag exists either locally or on the remote registry
        TAG="${TAG}:${1}"
        CMD=${*:2}
    else
        CMD=${*:1}
    fi
fi

## GUI
# To enable GUI, make sure processes in the container can connect to the x server
XAUTH=/tmp/.docker.xauth
if [ ! -f ${XAUTH} ]; then
    touch ${XAUTH}
    chmod a+r ${XAUTH}

    XAUTH_LIST=$(xauth nlist "${DISPLAY}")
    if [ -n "${XAUTH_LIST}" ]; then
        # shellcheck disable=SC2001
        XAUTH_LIST=$(sed -e 's/^..../ffff/' <<<"${XAUTH_LIST}")
        echo "${XAUTH_LIST}" | xauth -f ${XAUTH} nmerge -
    fi
fi
# GUI-enabling volumes
GUI_VOLUMES=(
    "${XAUTH}:${XAUTH}"
    "/tmp/.X11-unix:/tmp/.X11-unix"
    "/dev/input:/dev/input"
)
# GUI-enabling environment variables
GUI_ENVS=(
    XAUTHORITY="${XAUTH}"
    QT_X11_NO_MITSHM=1
    DISPLAY="${DISPLAY}"
)

## Additional volumes
CUSTOM_VOLUMES=()
# Synchronize timezone with host
CUSTOM_VOLUMES+=("/etc/localtime:/etc/localtime:ro")

## Additional environment variables
CUSTOM_ENVS=()
# Synchronize ROS_DOMAIN_ID with host
if [ -n "${ROS_DOMAIN_ID}" ]; then
    CUSTOM_ENVS+=("ROS_DOMAIN_ID=${ROS_DOMAIN_ID}")
fi
# Synchronize IGN_PARTITION with host
if [ -n "${IGN_PARTITION}" ]; then
    CUSTOM_ENVS+=("IGN_PARTITION=${IGN_PARTITION}")
fi
# Synchronize RMW configuration with host
if [ -n "${RMW_IMPLEMENTATION}" ]; then
    CUSTOM_ENVS+=("RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}")
fi
if [ -n "${CYCLONEDDS_URI}" ]; then
    CUSTOM_ENVS+=("CYCLONEDDS_URI=${CYCLONEDDS_URI}")
    CUSTOM_VOLUMES+=("${CYCLONEDDS_URI//file:\/\//}:${CYCLONEDDS_URI//file:\/\//}:ro")
fi
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
    CUSTOM_ENVS+=("FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}")
    CUSTOM_VOLUMES+=("${FASTRTPS_DEFAULT_PROFILES_FILE}:${FASTRTPS_DEFAULT_PROFILES_FILE}:ro")
fi

DOCKER_RUN_CMD=(
    docker run
    --interactive
    --tty
    --rm
    --network host
    --ipc host
    --privileged
    --security-opt "seccomp=unconfined"
    "${GUI_VOLUMES[@]/#/"--volume "}"
    "${GUI_ENVS[@]/#/"--env "}"
    "${CUSTOM_VOLUMES[@]/#/"--volume "}"
    "${CUSTOM_ENVS[@]/#/"--env "}"
    "${TAG}"
    "${CMD}"
)

echo -e "\033[1;30m${DOCKER_RUN_CMD[*]}\033[0m" | xargs

# shellcheck disable=SC2048
exec ${DOCKER_RUN_CMD[*]}
