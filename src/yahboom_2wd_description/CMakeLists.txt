cmake_minimum_required(VERSION 3.8)
project(yahboom_2wd_description)

find_package(ament_cmake REQUIRED)

foreach(dir urdf rviz)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()
