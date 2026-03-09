cmake_minimum_required(VERSION 3.8)
project(wl100_bringup)

find_package(ament_cmake REQUIRED)

# 安装 launch 文件、配置文件、脚本
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# TODO: 当 scripts/ 下有实际脚本时启用
# install(DIRECTORY scripts/
#   DESTINATION lib/${PROJECT_NAME}
#   USE_SOURCE_PERMISSIONS
# )

ament_package()
