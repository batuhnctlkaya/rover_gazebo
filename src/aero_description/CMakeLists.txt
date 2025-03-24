cmake_minimum_required(VERSION 3.8)
project(aero_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch meshes config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()