cmake_minimum_required(VERSION 3.8)
project(curtmini_piper_gz_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Readiness has an 80-second deadline; allow extra time for teardown.
  add_launch_test(test/test_simulation_startup.py TIMEOUT 110)
endif()

ament_package()
