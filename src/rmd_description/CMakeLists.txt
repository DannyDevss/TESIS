cmake_minimum_required(VERSION 3.8)
project(rmd_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
