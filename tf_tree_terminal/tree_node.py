import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.executors import MultiThreadedExecutor
import yaml
import argparse
import os
import io
from datetime import datetime

ASCII = r"""
░▀█▀░█▀▀░░░░░▀█▀░█▀▄░█▀▀░█▀▀
░░█░░█▀▀░▄▄▄░░█░░█▀▄░█▀▀░█▀▀
░░▀░░▀░░░░░░░░▀░░▀░▀░▀▀▀░▀▀▀
TF-TREE CLI DEBUGGER
"""


class TFTreeCLI(Node):
    def __init__(self, profile, output_file, keep_alive, light):
        super().__init__('tf_tree_cli_helper')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.profile = profile
        self.output_file = output_file
        self.keep_alive = keep_alive
        self.light = light

        self.profiles_config = {
            'mobile': {'req': ['map', 'odom', 'base_link'], 'desc': 'REP 105 (Mobile)'},
            'arm': {'req': ['base_link', 'tool0'], 'desc': 'REP 199 (Articulated)'},
            'auto': {'req': ['base_link'], 'desc': 'Auto-Detection'}
        }

        print(ASCII)
        self.get_logger().info(f"Mode: {self.profiles_config[profile]['desc']}")

        if self.keep_alive:
            self.get_logger().info("Alive mode: Updating every 5s. Ctrl+C to stop.")
            self.timer = self.create_timer(5.0, self.timer_callback)
        else:
            self.get_logger().info("Single-shot mode: Buffering (3s)...")
            self.timer = self.create_timer(3.0, self.timer_callback)

    def get_publisher_name(self):
        try:
            publishers = self.get_publishers_info_by_topic('/tf')
            static_pubs = self.get_publishers_info_by_topic('/tf_static')
            all_pubs = publishers + static_pubs
            return ", ".join(set([p.node_name for p in all_pubs])) if all_pubs else "Unknown"
        except:
            return "N/A"

    def timer_callback(self):
        # Prepare output
        output_buffer = io.StringIO()
        self.perform_analysis(output_buffer)
        final_text = output_buffer.getvalue()

        # Print and Save
        print(final_text)
        if self.output_file:
            self.save_to_file(final_text)

        # Shutdown logic for MultiThreadedExecutor
        if not self.keep_alive:
            self.get_logger().info("Analysis complete. Shutting down...")
            self.timer.cancel()
            rclpy.shutdown()

    def perform_analysis(self, buf):
        yaml_raw = self.tf_buffer.all_frames_as_yaml()
        if not yaml_raw or yaml_raw in ["{}", "[]"]:
            buf.write("\n❌ No TF tree detected.\n")
            return

        data = yaml.safe_load(yaml_raw)
        tree, child_to_parent, all_frames = {}, {}, set()

        for frame_id, details in data.items():
            all_frames.add(frame_id)
            parent = details.get('parent')
            if parent:
                all_frames.add(parent)
                child_to_parent[frame_id] = parent
                tree.setdefault(parent, []).append(frame_id)

        roots = [f for f in all_frames if f not in child_to_parent]

        buf.write(f"\n--- TF SNAPSHOT: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")
        if len(roots) > 1:
            buf.write(f"🔥 ALERT: {len(roots)} DISJOINTED TREES!\n")
            buf.write(f"└─ Roots: {', '.join(roots)}\n\n")

        for root in roots:
            self._print_recursive(root, tree, data, "", buf)

        if not self.light:
            self._print_diagnostic(all_frames, buf)

    def _print_recursive(self, frame, tree, raw_data, indent, buf, is_last=True, is_root=True):
        stats = raw_data.get(frame, {})
        freq = stats.get('frequency', 0.0)
        age_msg = ""
        actual_hz = freq
        parent = next((p for p, children in tree.items() if frame in children), None)

        if parent:
            try:
                now = self.get_clock().now()
                t = self.tf_buffer.lookup_transform(parent, frame, rclpy.time.Time())
                stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec

                if stamp_ns == 0:
                    age_msg = " [STATIC]"
                    actual_hz = 0.0
                else:
                    age_ms = (now.nanoseconds - stamp_ns) / 1e6
                    if age_ms < 500:
                        age_msg = f" [LIVE: {age_ms:.1f}ms]"
                        if freq == 0.0 and age_ms > 0:
                            actual_hz = 1000.0 / age_ms
                    else:
                        age_msg = f" [STALE: {age_ms/1000:.1f}s]"
            except:
                age_msg = " [NO DATA]"

        freq_label = f"{actual_hz:.1f} Hz" if actual_hz > 0 else "STATIC"
        marker = "" if is_root else ("└── " if is_last else "├── ")
        if self.light:
            buf.write(f"{indent}{marker}{frame}\n")
        else:
            buf.write(f"{indent}{marker}🔗 Link: {frame} [{freq_label}]{age_msg}\n")

        if not is_root and not self.light:
            tf_pub = self.get_publisher_name()
            js_pubs = self.get_joint_state_publishers()
            js_src = ", ".join(js_pubs) if js_pubs else "❌ none"
            sub_indent = indent + ("    " if is_last or is_root else "│   ")
            buf.write(
                f"{sub_indent}⚙️  Joint: to_{frame} "
                f"[TF: {tf_pub} | JointState: {js_src}]\n"
            )

        if frame in tree:
            for i, child in enumerate(sorted(tree[frame])):
                self._print_recursive(child, tree, raw_data,
                                     indent + ("" if is_root else ("    " if is_last else "│   ")),
                                     buf, i == len(tree[frame])-1, False)

    def _print_diagnostic(self, all_frames, buf):
        has_js = bool(self.get_publishers_info_by_topic('/joint_states'))
        status = "✅" if has_js else "❌"
        buf.write(f"{status} /joint_states topic\n")

        buf.write("\n--- COMPLIANCE DIAGNOSTIC ---\n")
        for f in self.profiles_config[self.profile]['req']:
            status = "✅" if f in all_frames else "❌"
            buf.write(f"{status} Recommendation: {f}\n")

    def get_joint_state_publishers(self):
        try:
            infos = self.get_publishers_info_by_topic('/joint_states')
            return set(i.node_name for i in infos)
        except:
            return set()

    def save_to_file(self, content):
        try:
            with open(self.output_file, 'w') as f:
                f.write(content)
                f.write(f"\nGenerated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            print(f"[INFO] Saved to: {os.path.abspath(self.output_file)}")
        except Exception as e:
            print(f"[ERROR] Save failed: {e}")


def main():
    parser = argparse.ArgumentParser(description="ROS 2 TF Tree CLI Helper")
    parser.add_argument('-p', '--profile', choices=['mobile', 'arm', 'auto'], default='auto')
    parser.add_argument('-s', '--save', type=str, metavar='FILE')
    parser.add_argument('-a', '--alive', action='store_true')
    parser.add_argument('-l', '--light', action='store_true',
                    help='Light mode: display only the TF tree structure')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = TFTreeCLI(args.profile, args.save, args.alive, args.light)

    # We use MultiThreadedExecutor to keep the TF buffer updating
    # while the main loop waits for the timer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (SystemExit, KeyboardInterrupt):
        # Catching SystemExit allows a clean exit from timer_callback
        pass
    finally:
        if args.alive:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
