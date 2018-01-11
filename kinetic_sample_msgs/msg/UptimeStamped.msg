# This message provides informaiton on a systems current uptime

# The system startup time can be found with:
#   start = header.stamp - uptime

# Variables:
#   - header: standard message header
#   - uptime: total time the system has been online for

std_msgs/Header header
std_msgs/Duration uptime
