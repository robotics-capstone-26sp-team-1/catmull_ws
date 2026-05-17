from __future__ import annotations

from typing import TYPE_CHECKING
from geometry_msgs.msg import Twist

if TYPE_CHECKING:
    from assistfour.main import Main


class NavigationManager:
    def __init__(self, node: Main):
        self.node = node

    def search_for_marker(self, name: str):
        # Stow for safety.
        self.node.stow_the_robot()

        # Switch to nav mode.
        self.node.switch_to_navigation_mode()

        # Begin spinning.
        command = Twist()
        command.angular.z = 0.2

        self.node.vel_publisher.publish(command)
