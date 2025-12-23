import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import yaml
import sys
import os
from datetime import datetime

ASCII = r"""
░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
"""

class TFTreeTerminal(Node):
    def __init__(self):
        super().__init__('tf_tree_terminal')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        print(ASCII)
        self.get_logger().info("Buffering TF data for 2 seconds...")
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # Check if a filename was provided as an argument
        self.output_file = sys.argv[1] if len(sys.argv) > 1 else None

    def timer_callback(self):
        self.display_tree()
        raise SystemExit

    def display_tree(self):
        yaml_data = self.tf_buffer.all_frames_as_yaml()
        
        if not yaml_data or yaml_data in ["{}", "[]"]:
            print("\nNo TF tree detected.")
            return

        data = yaml.safe_load(yaml_data)
        tree = {}
        child_frames = set()

        items = data.items() if isinstance(data, dict) else []
        if isinstance(data, list):
            for entry in data:
                items.extend(entry.items())

        for frame_id, details in items:
            parent = details.get('parent')
            if parent:
                if parent not in tree: tree[parent] = []
                tree[parent].append(frame_id)
                child_frames.add(frame_id)

        roots = [f for f in tree.keys() if f not in child_frames]

        # Prepare output buffer
        output_lines = ["--- ROS 2 TF Tree Snapshot ---"]
        for root in roots:
            self._build_tree_string(root, tree, output_lines)
        output_lines.append("------------------------------")

        # Join lines into one string
        final_output = "\n".join(output_lines)

        # Print to terminal
        print(final_output)

        # Save to file if requested
        if self.output_file:
            try:
                # Generate timestamp
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                with open(self.output_file, 'w') as f:
                    f.write(final_output)
                    f.write(f"\nGenerated on: {now}\n") # Add the date and time here
                
                print(f"\n[INFO] Tree saved to: {os.path.abspath(self.output_file)}")
            except Exception as e:
                print(f"\n[ERROR] Could not save to file: {e}")

    def _build_tree_string(self, frame, tree, lines, indent="", is_last=True, is_root=True):
        marker = "" if is_root else ("└── " if is_last else "├── ")
        lines.append(f"{indent}{marker}{frame}")

        new_indent = indent + ("" if is_root else ("    " if is_last else "│   "))
        
        if frame in tree:
            children = sorted(tree[frame])
            for i, child in enumerate(children):
                self._build_tree_string(child, tree, lines, new_indent, i == len(children)-1, False)

def main():
    rclpy.init()
    node = TFTreeTerminal()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()