# TF Tree Terminal

A simple ROS 2 Python tool that prints the current TF tree to your terminal using a recursive "folder-like" structure. This is useful for quickly verifying frame hierarchies without needing a GUI (Rviz) or generating PDF files.

## Features
- **Terminal Based**: No GUI required.
- **Tree Visualization**: Uses Unicode characters (`├──`, `└──`, `│`) to show parent-child relationships clearly.
- **Auto-Exit**: Collects data for 2 seconds and then exits automatically.
- **Robust**: Handles both dictionary and list formats returned by the TF buffer across different ROS 2 versions.

## Installation

1. Navigate to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
    ```

2. Clone or move this package into the `src` folder.

3. Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select tf_tree_terminal
    source install/setup.bash
    ```

## Usage

Simply run the executable:

    ros2 run tf_tree_terminal show

### Saving to a file
To export the tree to a text file for documentation:

    ros2 run tf_tree_terminal show ~/path/to/save/tree.txt

### Example Output

```text
--- Terminal TF Tree ---
map
└── odom
    └── base_footprint
        └── base_link
            ├── camera_link
            ├── chassis_link
            │   ├── front_axle
            │   └── rear_axle
            └── velodyne_link
------------------------

```

## How it works

The node initializes a `tf2_ros` Buffer and Listener. It waits for 2 seconds to allow the buffer to fill with broadcasted transforms, parses the frame data into a recursive tree structure, and prints it using a depth-first search (DFS) algorithm to ensure correct indentation.

