cmake_minimum_required(VERSION 3.8) # version
project(ptu_driver) # define nombre del proyecto

# advertencias de compilacion
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Incluir headers del paquete
include_directories(include)

# Crear el ejecutable
add_executable(ptu_node_driver
  src/ptu_node_driver.cpp
)

# Agregar las dependencias del ejecutable
ament_target_dependencies(ptu_node_driver
  rclcpp
  std_msgs
)

# Instalar el ejecutable para poder usar `ros2 run`
install(TARGETS
  ptu_node_driver
  DESTINATION lib/${PROJECT_NAME}
)

# bloque de testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
