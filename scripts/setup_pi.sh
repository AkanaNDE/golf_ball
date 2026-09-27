#!/usr/bin/env bash
# One-time setup of the golf ball robot on a Raspberry Pi 5 running Ubuntu 24.04 (arm64).
#   git clone https://github.com/AkanaNDE/golf_ball.git ~/golf_ws
#   bash ~/golf_ws/scripts/setup_pi.sh
# Installs ROS 2 Jazzy, builds the micro-ROS agent in ~/uros_ws and this workspace.
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UROS_DIR="$HOME/uros_ws"

. /etc/os-release
if [ "$VERSION_ID" != "24.04" ]; then
  echo "This script expects Ubuntu 24.04 (found $PRETTY_NAME). ROS 2 Jazzy binaries need 24.04." >&2
  exit 1
fi

echo "=== 1/5 ROS 2 Jazzy apt repository ==="
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  sudo apt update
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
fi

echo "=== 2/5 ROS 2 packages ==="
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-jazzy-joy ros-dev-tools python3-rosdep git
source /opt/ros/jazzy/setup.bash
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update

echo "=== 3/5 micro-ROS agent ($UROS_DIR) ==="
if [ ! -f "$UROS_DIR/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" ]; then
  mkdir -p "$UROS_DIR/src"
  cd "$UROS_DIR"
  [ -d src/micro_ros_setup ] || git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
  rosdep install --from-paths src --ignore-src -y
  colcon build
  source install/local_setup.bash
  ros2 run micro_ros_setup create_agent_ws.sh
  ros2 run micro_ros_setup build_agent.sh
fi
source "$UROS_DIR/install/local_setup.bash"

echo "=== 4/5 golf_ws ($WS_DIR) ==="
cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -y --skip-keys micro_ros_agent
colcon build --symlink-install

echo "=== 5/5 serial permission and ~/.bashrc ==="
sudo usermod -aG dialout "$USER"
for line in "source /opt/ros/jazzy/setup.bash" \
            "source $UROS_DIR/install/local_setup.bash" \
            "source $WS_DIR/install/setup.bash"; do
  grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
done

echo
echo "Done. Log out and back in (for the serial port group), then run:"
echo "  ros2 launch golf_bot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0"
