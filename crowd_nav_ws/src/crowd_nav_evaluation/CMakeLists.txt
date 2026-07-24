cmake_minimum_required(VERSION 3.16)
project(crowd_nav_evaluation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scenarios.py
  scripts/episode_monitor.py
  scripts/run_episode.py
  scripts/run_matrix.py
  scripts/make_plots.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
