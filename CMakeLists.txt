cmake_minimum_required(VERSION 3.14)

project(bitbot)

set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE ON)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)

include(bitbot.cmake)

BitbotAddDependency(
  NAME           bitbot_cifx
  GIT_REPO       https://github.com/limymy/bitbot_kernel.git
  GIT_TAG        main
  FILE_NAME      "bitbot_cifx-main.zip"
  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
  USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
  USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_executable(main_test main.cpp user_func.cpp)
target_link_libraries(main_test
  BitbotCifx
)
target_compile_options(main_test PUBLIC)
target_compile_features(main_test PUBLIC cxx_std_20)
