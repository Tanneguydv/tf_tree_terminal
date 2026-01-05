# TF-Tree CLI Debugger

```text
░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
      TF-TREE CLI DEBUGGER
```

A lightweight ROS 2 utility for auditing, visualizing, and validating your Coordinate Transform (TF) tree. This tool tries to go beyond simple visualization by providing deep-source auditing and standard compliance checking (REP 105/199).

## 🚀 Key Features

* **Deep Source Auditing**: Tracks exactly which nodes are publishing to `/tf` and `/joint_states` for every link.
* **Live vs. Static Analysis**: Automatically distinguishes between truly `STATIC` latched transforms and `LIVE` high-frequency streams.
* **Diagnostic Profiles**: Built-in validation for specific robot types:
* **Mobile**: Validates the `map` → `odom` → `base_link` chain.
* **Arm**: Validates articulated chains from `base_link` to `tool0`.


* **Precision Latency**: Reports the "age" of transforms in milliseconds to detect jitter or stale data.
* **Disjointed Tree Alerts**: Visually flags if your tree is broken into multiple disconnected components.

---

## 🛠 Usage

Once built and sourced, you can use the `tf-tree` command:

### Command Flags

| Flag | Short | Description |
| --- | --- | --- |
| `--profile` | `-p` | Diagnostic mode: `mobile`, `arm`, or `auto` (default). |
| `--save` | `-s` | Export the analysis to a file (e.g., `-s robot_report.txt`). |
| `--alive` | `-a` | Keep node alive; refreshes terminal every 5 seconds. |
| `--light` | `-l` | Get a light TF-Tree as output |
| `--help` | `-h` | Show help message and exit. |

### Examples

**Standard One-Shot Audit:**

```bash
tf-tree
```

**Save a mobile robot diagnostic to a file:**

```bash
tf-tree -p mobile -s my_robot.txt
```

**Live monitoring during a mission:**

```bash
tf-tree --alive
```

#### Example outputs

running ```tf-tree -p mobile```

```bash

░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
TF-TREE CLI DEBUGGER

[INFO] [1767483009.498731809] [tf_tree_cli_helper]: Mode: REP 105 (Mobile)
[INFO] [1767483009.499220178] [tf_tree_cli_helper]: Single-shot mode: Buffering (3s)...

--- TF SNAPSHOT: 2026-01-03 23:30:12 ---
⚯ Link: base_link [STATIC]
├── ⚯ Link: base_footprint [STATIC] [STATIC]
│   ⚙  Joint: to_base_footprint [TF: robot_state_publisher | JointState: joint_state_publisher]
├── ⚯ Link: camera_link [STATIC] [STATIC]
│   ⚙  Joint: to_camera_link [TF: robot_state_publisher | JointState: joint_state_publisher]
│   └── ⚯ Link: camera_link_optical [STATIC] [STATIC]
│       ⚙  Joint: to_camera_link_optical [TF: robot_state_publisher | JointState: joint_state_publisher]
├── ⚯ Link: left_wheel [12.3 Hz] [LIVE: 81.5ms]
│   ⚙  Joint: to_left_wheel [TF: robot_state_publisher | JointState: joint_state_publisher]
└── ⚯ Link: right_wheel [12.3 Hz] [LIVE: 81.6ms]
    ⚙  Joint: to_right_wheel [TF: robot_state_publisher | JointState: joint_state_publisher]
    └── ⚯ Link: test_orphan [STATIC] [STATIC]
        ⚙  Joint: to_test_orphan [TF: robot_state_publisher | JointState: joint_state_publisher]
✔ /joint_states topic

--- COMPLIANCE DIAGNOSTIC ---
✖ Recommendation: map
✖ Recommendation: odom
✔ Recommendation: base_link
```
and light mode, running ```tf-tree -l```

```bash
--- TF SNAPSHOT: 2026-01-03 23:38:17 ---
base_link
├── base_footprint
├── camera_link
│   └── camera_link_optical
├── left_wheel
└── right_wheel
    └── test_orphan
```

or broken, running ```tf-tree -l``` :

```bash
--- TF SNAPSHOT: 2026-01-03 23:41:04 ---
⚠ ALERT: 2 DISJOINTED TREES!
└─ Roots: right_wheel, base_link

right_wheel
└── test_orphan
base_link
├── base_footprint
└── camera_link
    └── camera_link_optical
```

---

## 📊 Understanding the Output

* **`⚯ Link`**: The TF frame name with its frequency (Hz).
* **`[LIVE: 15ms]`**: Data is current.
* **`[STALE: 2.1s]`**: The broadcaster has likely stopped or crashed.
* **`⚙ Joint`**: Lists the publisher nodes for both the transform and the joint state.
* **`✔ Recommendation`**: Compliance status based on the selected profile.

---

## 🔧 Installation

1. **Clone to your workspace:**
```bash
cd ~/ros2_ws/src
git clone https://github.com/Tanneguydv/tf_tree_terminal.git
```


2. **Build:**
```bash
cd ~/ros2_ws
colcon build --packages-select tf_tree_terminal
source install/setup.bash
```



---

## 💡 How it works

The debugger utilizes a `MultiThreadedExecutor` to simultaneously listen to the TF buffer while calculating statistics. It performs a **Depth-First Search (DFS)** to reconstruct the tree from the YAML data provided by the ROS 2 buffer, enriching each node with metadata gathered from the graph introspection API.
