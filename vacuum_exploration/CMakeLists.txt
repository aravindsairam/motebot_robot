cmake_minimum_required(VERSION 3.10)
project(vacuum_exploration)

# A Python package built through CMake rather than setup.py, like vacuum_coverage: colcon's
# symlink install of setup.py packages breaks with the pip-installed setuptools on this PC.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/exploration_node scripts/explore DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 120)
endif()

ament_package()
