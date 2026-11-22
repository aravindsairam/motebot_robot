cmake_minimum_required(VERSION 3.10)
project(vacuum_mode_manager)

# A Python package built through CMake rather than setup.py: colcon's symlink install of
# setup.py packages breaks with newer pip-installed setuptools ("option --editable not recognized").
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/mode_manager scripts/mode scripts/save_map scripts/zones scripts/settings_node scripts/settings
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test)
endif()

ament_package()
