cmake_minimum_required(VERSION 3.8)
project(rust_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  rust_mpc/load_raceline.py
  rust_mpc/mpc_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/launch_mpc.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
