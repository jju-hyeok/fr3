cmake_minimum_required(VERSION 3.22)
project(my_robot)

find_package(ament_cmake REQUIRED)

# launch 디렉토리 있으면 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# config 디렉토리 있을 때만 설치 (없어도 에러 안 나게)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# MoveIt Setup Assistant용 숨김 파일은 지금 없으니 설치 안 함
# install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()

