#!/usr/bin/env python3

import unittest
import json
import tempfile
import rostest
from geometry_msgs.msg import PoseStamped
from unittest.mock import patch, MagicMock
import os

from jimmy_tools_pkg.scripts.goto_objects import GotoObject


class TestGotoObjects(unittest.TestCase):
    def setUp(self):
        # Create a temporary JSON file for testing
        self.temp_file = tempfile.NamedTemporaryFile(mode='w+', delete=False)
        self.test_data = {
            "chair": [1.0, 2.0, 3.0],
            "table": [[1.5, 2.5, 3.5], [4.0, 5.0, 6.0]]
        }
        json.dump(self.test_data, self.temp_file)
        self.temp_file.close()

        # Mock ROS publisher
        self.mock_pub = MagicMock()

    def tearDown(self):
        os.unlink(self.temp_file.name)

    def test_load_from_file_success(self):
        """Test loading objects from a valid JSON file."""
        goto = GotoObject()
        goto.filename = self.temp_file.name
        objects = goto.load_from_file()
        self.assertEqual(objects, self.test_data)

    def test_load_from_file_not_found(self):
        """Test behavior when JSON file does not exist."""
        goto = GotoObject()
        goto.filename = "nonexistent_file.json"
        objects = goto.load_from_file()
        self.assertEqual(objects, {})

    @patch('builtins.input', return_value='chair')
    def test_ask_for_object_valid_input(self, mock_input):
        """Test selecting a valid object."""
        goto = GotoObject()
        goto.filename = self.temp_file.name
        goto.objects = goto.load_from_file()
        goto.goal_pub = self.mock_pub

        goto.ask_for_object(None)  # Simulate timer callback
        self.mock_pub.publish.assert_called_once()

    @patch('builtins.input', return_value='nonexistent_object')
    def test_ask_for_object_invalid_input(self, mock_input):
        """Test selecting an object that doesn't exist."""
        goto = GotoObject()
        goto.filename = self.temp_file.name
        goto.objects = goto.load_from_file()
        goto.goal_pub = self.mock_pub

        goto.ask_for_object(None)
        self.mock_pub.publish.assert_not_called()

    def test_send_goal_single_position(self):
        """Test sending a goal with a single position."""
        goto = GotoObject()
        goto.goal_pub = self.mock_pub

        test_position = [1.0, 2.0, 3.0]
        goto.send_goal(test_position)

        # Check if the published goal has correct coordinates
        published_goal = self.mock_pub.publish.call_args[0][0]
        self.assertIsInstance(published_goal, PoseStamped)
        self.assertEqual(published_goal.pose.position.x, 3.0)  # z → x
        self.assertEqual(published_goal.pose.position.y, -1.0)  # x → y (inverted)
        self.assertEqual(published_goal.pose.position.z, 0.0)

    def test_send_goal_multiple_positions(self):
        """Test sending a goal when multiple positions exist (uses first position)."""
        goto = GotoObject()
        goto.goal_pub = self.mock_pub

        test_positions = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        goto.send_goal(test_positions)

        published_goal = self.mock_pub.publish.call_args[0][0]
        self.assertEqual(published_goal.pose.position.x, 3.0)
        self.assertEqual(published_goal.pose.position.y, -1.0)

    @patch('rospy.loginfo')
    def test_ask_for_object_empty_list(self, mock_loginfo):
        """Test behavior when no objects are available."""
        goto = GotoObject()
        goto.objects = {}
        goto.ask_for_object(None)
        mock_loginfo.assert_called_with("No objects available to navigate to.")


if __name__ == '__main__':
    rostest.rosrun('jimmy_tools_pkg', 'test_goto_objects', TestGotoObjects)
