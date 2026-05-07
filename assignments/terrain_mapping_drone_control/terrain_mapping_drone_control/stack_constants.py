"""ROS 2 / uXRCE settings shared by assignment 3 launch files (keep in sync with MISSION_AND_SLAM.md)."""

# PX4 uXRCE + Micro XRCE Agent publish DDS on domain 0 by default.
# Keep ROS 2 on the same domain or /fmu/out/* will have zero publishers.
#
# Avoid conflicts with a snap daemon agent by using a different UDP port (8889),
# and stop the snap service if needed (`sudo snap stop micro-xrce-dds-agent`).
ROS_DOMAIN_ID = '0'
# UDP port for Micro XRCE DDS Agent (snap defaults to 8888; PX4 must use a different port here).
UXRCE_UDP_PORT = '8889'
