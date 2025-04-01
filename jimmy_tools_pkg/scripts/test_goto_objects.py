#!/usr/bin/env python3

import unittest
import rospy
import rostest

from goto_objects import GotoObject


class TestGotoObject(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_goto_object", anonymous=True)  # ✅ Initialize once for testing
        self.node = GotoObject()

    def test_objects_loading(self):
        """Check that objects are loaded correctly"""
        self.assertIsInstance(self.node.objects, dict)


if __name__ == "__main__":
    rostest.rosrun("jimmy_tools_pkg", "test_goto_objects", TestGotoObject)
