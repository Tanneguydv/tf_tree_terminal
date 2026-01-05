import pytest
import rclpy
import os
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from io import StringIO

from tf_tree_terminal.tree_node import TFTreeCLI

@pytest.fixture(scope="session")
def ros_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()

def test_tf_tree_generation(ros_init):
    """Test if the node correctly parses a simulated hierarchy."""
    node = TFTreeCLI('auto', None, False, True, False)
    br = TransformBroadcaster(node)
    
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'base_link'
    t.transform.rotation.w = 1.0
    
    # Broadcast and spin to fill buffer
    for _ in range(5):
        t.header.stamp = node.get_clock().now().to_msg()
        br.sendTransform(t)
        rclpy.spin_once(node, timeout_sec=0.1)

    buf = StringIO()
    node.perform_analysis(buf)
    output = buf.getvalue()

    assert "map" in output
    assert "base_link" in output
    node.destroy_node()

def test_save_to_file_functionality(ros_init, tmp_path):
    """Test if the node correctly exports analysis to a file."""
    # tmp_path is a built-in pytest fixture for unique temp directories
    test_file = tmp_path / "tf_report.txt"
    
    # Initialize node with a save path
    node = TFTreeCLI('auto', str(test_file), False, True, False)
    
    test_content = "TEST_TF_DATA_123"
    node.save_to_file(test_content)
    
    # Check if file exists
    assert test_file.exists()
    
    # Check content (stripping ANSI occurs in your save_to_file method)
    with open(test_file, 'r') as f:
        saved_text = f.read()
        assert "TEST_TF_DATA_123" in saved_text
        assert "Generated on:" in saved_text

    node.destroy_node()

def test_compliance_logic(ros_init):
    """Test if 'mobile' profile flags missing frames."""
    # Set light=False (4th arg) so the diagnostic section is printed
    node = TFTreeCLI('mobile', None, False, False, False) 
    br = TransformBroadcaster(node)
    
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'base_link'
    t.transform.rotation.w = 1.0
    
    # Broadcast and spin
    for _ in range(5):
        t.header.stamp = node.get_clock().now().to_msg()
        br.sendTransform(t)
        rclpy.spin_once(node, timeout_sec=0.1)

    buf = StringIO()
    node.perform_analysis(buf)
    output = buf.getvalue()

    # Diagnostic section should now be present in the buffer
    assert "--- COMPLIANCE DIAGNOSTIC ---" in output
    assert "✖ Recommendation: odom" in output
    node.destroy_node()