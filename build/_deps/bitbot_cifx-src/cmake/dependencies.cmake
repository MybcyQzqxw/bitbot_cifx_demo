include(${CMAKE_CURRENT_LIST_DIR}/bitbot.cmake)

if(NOT DEFINED BITBOT_KERNEL_INCLUDED OR NOT ${BITBOT_KERNEL_INCLUDED})
  BitbotAddDependency(
    NAME           bitbot_kernel
    GIT_REPO       git@github.com:LiMYmy/bitbot_kernel.git
    GIT_TAG        main
    FILE_NAME      "bitbot_kernel-main.zip"
    FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
    USE_PROXY      ${BITBOT_DEPENDENCY_USE_PROXY}
    USE_LOCAL_FILE ${BITBOT_DEPENDENCY_USE_LOCAL_FILE}
  )
endif()