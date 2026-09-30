#!/usr/bin/env bash
# Set up the Raspberry Pi 4 for Stage 1 of docs/poc-plan.md: ROS 2 Jazzy
# (ros-base), LDRobot's LD14P lidar driver, Cartographer, the recorder,
# foxglove_bridge, and this repository's workspace.
#
# WRITTEN, NOT YET RUN ON A PI. The lidar driver's build was checked against
# Jazzy on an x86 machine only. See docs/getting-started.md, "Set up the Pi".
#
# Usage, as your normal user (it asks for sudo itself):
#   robot/scripts/setup-pi.sh [ROS_DOMAIN_ID]      default 42, allowed 1-101
# Safe to run again: every step checks what is already done.
#
# The Pi only listens. Nothing installed here talks to the car's motor
# controller or radio receiver, and the workspace's only keyboard driving
# program (teleop_twist_keyboard) is skipped on purpose, along with the
# simulator and the screen tools (Gazebo, RViz), which a Pi with no screen
# cannot use. Nav2 is still installed, because the workspace lists it, but
# nothing in Stage 1 starts it.
set -euo pipefail

DOMAIN_ID=${1:-42}
REPO=$(cd "$(dirname "$0")/../.." && pwd)
WS=$REPO/robot/ros2_ws
LIDAR_WS=$HOME/ldlidar_ws
LIDAR_SRC=$LIDAR_WS/src/ldlidar_ros2
# LDRobot's official driver, pinned to its latest commit (2023-03-12) so a
# re-run builds the same thing. Its sdk/ folder is a git submodule.
LIDAR_URL=https://github.com/ldrobotSensorTeam/ldlidar_ros2.git
LIDAR_COMMIT=0f6101c93db0b4448cbe7a654826adcdee50622f
# ament_python and ament_pytest are skipped because rosdep has no entry for
# either: every package here lists both, which is why the usual command
# carries -r. Skipping just those keys lets any other failure stop this
# script instead of scrolling past.
# slam_toolbox is skipped because the car maps with Cartographer, and its
# package would pull RViz and Qt onto the Pi anyway.
SKIP_KEYS="ament_python ament_pytest rviz2 ros_gz_sim ros_gz_bridge joint_state_publisher_gui teleop_twist_keyboard slam_toolbox"

say() { printf '\n== %s\n' "$*"; }
fail() { printf '\nSTOPPED: %s\n' "$*" >&2; exit 1; }
# Never stop to ask a question, and wait for the first-boot automatic updates
# to let go of apt instead of failing.
apt_get() {
  sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a \
    apt-get -o DPkg::Lock::Timeout=600 "$@"
}

say "Is this the right computer?"
[[ $EUID -ne 0 ]] || fail "run this as your normal user, without sudo. It asks for sudo itself."
if ! [[ $DOMAIN_ID =~ ^[0-9]+$ ]] || ((10#$DOMAIN_ID < 1 || 10#$DOMAIN_ID > 101)); then
  fail "ROS_DOMAIN_ID must be a whole number from 1 to 101, not '$DOMAIN_ID'."
fi
DOMAIN_ID=$((10#$DOMAIN_ID))
# shellcheck disable=SC1091
. /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 24.04 ]] || fail "needs Ubuntu 24.04, found $PRETTY_NAME."
arch=$(dpkg --print-architecture)
[[ $arch == arm64 ]] || fail "needs the 64-bit (arm64) Ubuntu image, found $arch."
echo "OK: $PRETTY_NAME, $arch."

say "Memory (free -h)"
free -h
BUILD_ARGS=(--symlink-install)
mem_kb=$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)
if ((mem_kb < 2 * 1024 * 1024)); then
  # The 1 GB and 2 GB Pi 4 both land here: a "2 GB" Pi reports about 1.8.
  echo "Under 2 GB: adding 2 GB of swap and building one package at a time."
  if [[ ! -f /swapfile ]]; then
    sudo fallocate -l 2G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
  fi
  swapon --show=NAME --noheadings | grep -qx /swapfile || sudo swapon /swapfile
  grep -q '^/swapfile ' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
  BUILD_ARGS+=(--executor sequential --parallel-workers 1)
else
  echo "OK: 2 GB or more."
fi

say "ROS 2 apt repository (the official ros2-apt-source package)"
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
  echo "OK: already set up."
else
  apt_get update
  apt_get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  v=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    grep -F '"tag_name"' | awk -F'"' '{print $4}')
  [[ -n $v ]] || fail "could not ask GitHub for the ROS apt source version. Is the Wi-Fi up?"
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$v/ros2-apt-source_$v.${UBUNTU_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

say "Installing ROS 2 Jazzy ros-base, Cartographer, foxglove_bridge"
# ros-base includes rosbag2, the recorder. libraspberrypi-bin gives vcgencmd,
# iw reads the Wi-Fi band, and avahi-daemon lets the Mac find the Pi by name
# (<hostname>.local).
apt_get update
apt_get upgrade -y
apt_get install -y git ros-dev-tools ros-jazzy-ros-base \
  ros-jazzy-cartographer-ros ros-jazzy-foxglove-bridge \
  libraspberrypi-bin iw avahi-daemon

say "rosdep"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update

say "Permission to read the lidar's USB adapter (dialout group)"
if id -nG "$USER" | grep -qw dialout; then
  echo "OK: $USER is in dialout."
else
  sudo usermod -aG dialout "$USER"
  echo "Added $USER to dialout. It takes effect after you log out and back in."
fi

say "LDRobot lidar driver (ldlidar_ros2) into $LIDAR_WS"
[[ -d $LIDAR_SRC/.git ]] || git clone "$LIDAR_URL" "$LIDAR_SRC"
git -C "$LIDAR_SRC" checkout -q "$LIDAR_COMMIT"
git -C "$LIDAR_SRC" submodule update --init --recursive

# ROS's setup scripts read unset variables, so relax -u while sourcing them.
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
set -u

say "Installing what the two workspaces need (rosdep)"
rosdep install -y --rosdistro jazzy --ignore-src \
  --from-paths "$WS/src" "$LIDAR_WS/src" --skip-keys "$SKIP_KEYS"

say "Building the lidar driver"
# One SDK file (log_module.cpp) uses pthread without including it, which
# Ubuntu 24.04's compiler refuses. Including it from the command line fixes
# that without changing LDRobot's code.
(cd "$LIDAR_WS" && colcon build "${BUILD_ARGS[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_CXX_FLAGS=-include pthread.h")
set +u
# shellcheck disable=SC1091
. "$LIDAR_WS/install/setup.bash"
set -u

say "Building this repository's workspace"
(cd "$WS" && colcon build "${BUILD_ARGS[@]}")

say "Settings for every new terminal (~/.bashrc)"
touch ~/.bashrc
sed -i '/^# >>> setup-pi.sh >>>$/,/^# <<< setup-pi.sh <<<$/d' ~/.bashrc
cat >>~/.bashrc <<EOF
# >>> setup-pi.sh >>>
# Written by robot/scripts/setup-pi.sh. Running it again replaces this block.
source /opt/ros/jazzy/setup.bash
source $LIDAR_WS/install/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN_ID
# <<< setup-pi.sh <<<
EOF
echo "OK: ROS_DOMAIN_ID=$DOMAIN_ID."

say "Checks"
if command -v iw >/dev/null && freq=$(iw dev wlan0 link 2>/dev/null | awk '/freq:/ {print int($2)}') && [[ -n $freq ]]; then
  if ((freq >= 5000)); then echo "Wi-Fi: OK, 5 GHz ($freq MHz)."; else echo "Wi-Fi: PROBLEM, 2.4 GHz ($freq MHz), the band the car's remote uses."; fi
else
  echo "Wi-Fi: could not read the band; skipped."
fi

if compgen -G '/dev/ttyUSB*' >/dev/null; then
  echo "Lidar: OK, USB adapter seen at $(echo /dev/ttyUSB*). The LD14P launch expects /dev/ttyUSB0."
else
  echo "Lidar: no USB adapter seen. Plug it in and run: ls /dev/ttyUSB*"
fi

if command -v vcgencmd >/dev/null; then
  t=$(sudo vcgencmd get_throttled | cut -d= -f2)
  echo "Power: get_throttled=$t"
  if ((t == 0)); then
    echo "Power: OK, no low voltage, slowdown or overheating since the Pi started."
  else
    if ((t & 0x1)); then echo "Power: PROBLEM NOW, the voltage is too low. The power bank is not keeping up."; fi
    if ((t & 0x10000)); then echo "Power: PROBLEM, the voltage has dropped too low since the Pi started."; fi
    if ((t & 0x60006)); then echo "Power: the Pi has slowed itself down since it started."; fi
    if ((t & 0x80008)); then echo "Power: the Pi has reached its temperature limit since it started."; fi
  fi
else
  echo "Power: vcgencmd is not available; skipped."
fi

say "Done"
echo "Log out and back in (or run: sudo reboot), so the new settings apply."
echo "Foxglove on the Mac connects to: ws://$(hostname).local:8765"
echo "If that name does not work, use this address instead: ws://$(hostname -I | awk '{print $1}'):8765"
