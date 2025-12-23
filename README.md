# TF Tree Terminal
```text
+------------------------------------------+
|  ░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀           |
|  ░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀           |
|  ░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀           |
+------------------------------------------+
```

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

Simply run the command :

    tf-tree

or the executable:

    ros2 run tf_tree_terminal show

### Example Output

```text
--- ROS 2 TF Tree Snapshot ---
map
└── odom
    └── base_footprint
        └── base_link
            ├── camera_link
            ├── chassis_link
            │   ├── front_axle
            │   └── rear_axle
            └── velodyne_link
------------------------------
Generated on: 2025-12-23 13:10:59

```

### Saving to a file
To export the tree to a text file for documentation:

    tf-tree my-robot.txt

or

    ros2 run tf_tree_terminal show ~/path/to/save/tree.txt

## How it works

The node initializes a `tf2_ros` Buffer and Listener. It waits for 2 seconds to allow the buffer to fill with broadcasted transforms, parses the frame data into a recursive tree structure, and prints it using a depth-first search (DFS) algorithm to ensure correct indentation.

