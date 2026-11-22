cmake_minimum_required(VERSION 3.10)
project(vacuum_perception)

# A Python package built through CMake rather than setup.py, like the other vacuum packages.
# Python because the finder is a PyTorch model run on request, not on every camera frame.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/object_finder DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 120)
endif()

ament_package()
