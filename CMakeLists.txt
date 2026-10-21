cmake_minimum_required(VERSION 3.0.2)
project(px4_agent)
find_package(catkin REQUIRED)
catkin_package()

# Runtime scripts use the project layout (web assets, VIO workspace and logs).
# A source-space absolute launch works without rebuilding this package.
