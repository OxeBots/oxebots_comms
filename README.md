# Oxebots Comms

This package contains the communication modules used by the **Oxebots** team for the **SSL RoboCup league**. It includes nodes that bridge **Protocol Buffers** messages from league modules to our ROS2 infrastructure, facilitating seamless communication between different components of our robotic system.

## Table of Contents

- [Oxebots Comms](#oxebots-comms)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Reporting Issues](#reporting-issues)
  - [License](#license)

## Features

- Bridges Protocol Buffers messages (version 2) to ROS2 topics.
- Compatible with ROS2 Humble on Ubuntu 22.04.

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS.
- **ROS2 Distribution**: Humble Hawksbill.

## Installation

To install the package, clone the repository into your colcon workspace and build it:

```bash
# Source your ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to your colcon workspace
cd ${YOUR_COLCON_WORKSPACE}/src

# Clone the Oxebots Comms repository
git clone git@github.com:OxeBots/oxebots_comms.git

# Navigate back to the workspace root
cd ..

# Install the dependencies
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro=$ROS_DISTRO

# Build the workspace
colcon build --packages-select oxebots_comms

# Source the workspace
source install/setup.bash
```

*Note:* This package is part of the Oxebots software stack and depends on other packages from the team. You can find our complete software stack at [OxeBots/software_ws](https://github.com/OxeBots/software_ws).

## Usage

After building the package, you can run the communication nodes using:

```bash
ros2 run oxebots_comms comms_node
```

## Reporting Issues

If you encounter any issues or have suggestions for improvements, please open an issue on the [GitHub repository](https://github.com/OxeBots/oxebots_comms/issues).

## License

This project is licensed under the **GPL-3.0 license** - see the [LICENSE](LICENSE) file for details.
