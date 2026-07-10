cmake_minimum_required(VERSION 3.8)
project(ares_erc_bringup)

find_package(ament_cmake REQUIRED)

# Bring-up package: installs ERC launch/config only (no compiled targets).
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
