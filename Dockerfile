ARG BASE_IMAGE="ubuntu"
ARG TAG="22.04"
FROM ${BASE_IMAGE}:${TAG}

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ardupilot
ARG USER_UID=1000
ARG USER_GID=1000

# Optional ArduPilot install-prereqs switches
ARG SKIP_AP_EXT_ENV=0
ARG SKIP_AP_GRAPHIC_ENV=1
ARG SKIP_AP_COV_ENV=1
ARG SKIP_AP_GIT_CHECK=1
ARG DO_AP_STM_ENV=1

# Pick ArduPilot ref (branch/tag). For stability you can pin, e.g. "Copter-4.5.7"
ARG ARDUPILOT_REF="master"

WORKDIR /ardupilot

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release sudo tzdata git default-jre bash-completion ca-certificates curl \
    python3 python3-pip \
 && rm -rf /var/lib/apt/lists/*

# Create user
RUN groupadd ${USER_NAME} --gid ${USER_GID} \
 && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash \
 && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME} \
 && chmod 0440 /etc/sudoers.d/${USER_NAME}

# Clone ArduPilot WITH submodules (modules/waf is required)
RUN git clone --recurse-submodules --depth 1 --branch ${ARDUPILOT_REF} \
      https://github.com/ArduPilot/ardupilot.git /ardupilot

# Fix ownership
RUN chown -R ${USER_NAME}:${USER_NAME} /ardupilot

USER ${USER_NAME}

# Avoid "dubious ownership" for git operations during build
RUN git config --global --add safe.directory /ardupilot

# Run prereqs
RUN chmod +x /ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh

RUN SKIP_AP_EXT_ENV=$SKIP_AP_EXT_ENV \
    SKIP_AP_GRAPHIC_ENV=$SKIP_AP_GRAPHIC_ENV \
    SKIP_AP_COV_ENV=$SKIP_AP_COV_ENV \
    SKIP_AP_GIT_CHECK=$SKIP_AP_GIT_CHECK \
    DO_AP_STM_ENV=$DO_AP_STM_ENV \
    AP_DOCKER_BUILD=1 \
    USER=${USER_NAME} \
    bash /ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

# Ensure ~/.local/bin is in PATH for --user installs
RUN echo 'if [ -d "$HOME/.local/bin" ] ; then PATH="$HOME/.local/bin:$PATH"; fi' >> /home/${USER_NAME}/.ardupilot_env

# Build Copter SITL
RUN ./waf configure --board=sitl && ./waf copter

# Runtime env
ENV BUILDLOGS=/tmp/buildlogs
ENV CCACHE_MAXSIZE=1G

# Create entrypoint (no heredoc; works reliably in Docker RUN)
USER root
RUN printf '%s\n' \
  '#!/bin/bash' \
  'set -e' \
  '[ -f "/home/ardupilot/.ardupilot_env" ] && source "/home/ardupilot/.ardupilot_env"' \
  'exec "$@"' \
  > /ardupilot_entrypoint.sh \
 && chmod +x /ardupilot_entrypoint.sh

USER ${USER_NAME}

ENTRYPOINT ["/ardupilot_entrypoint.sh"]
CMD ["bash"]
