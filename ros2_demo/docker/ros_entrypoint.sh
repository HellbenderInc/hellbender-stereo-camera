#!/bin/bash
set -e

# Install baked-in defaults that are not already present at the destination.
# Bind-mounted paths (e.g. /etc/hb-camera-config from the host) are handled
# by walking into them: missing entries are populated, existing ones are
# preserved so host-side edits survive container restarts.
install_defaults() {
    local src_root="$1"
    local dest_root="$2"
    [ -d "$src_root" ] || return 0
    mkdir -p "$dest_root"
    while IFS= read -r src; do
        local rel="${src#$src_root/}"
        local dest="$dest_root/$rel"
        if [ -d "$src" ]; then
            mkdir -p "$dest"
        elif [ ! -e "$dest" ]; then
            cp -a "$src" "$dest"
            echo "installed default: $dest"
        fi
    done < <(find "$src_root" -mindepth 1)
}

install_defaults /opt/stereo_depth_node/configs /etc
install_defaults /opt/stereo_depth_node/networks /networks

source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
