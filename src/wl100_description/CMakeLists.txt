cmake_minimum_required(VERSION 3.8)
project(wl100_description)

find_package(ament_cmake REQUIRED)

# 安装 URDF 模型文件
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
