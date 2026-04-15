cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(bitbot_cifx)

include(cmake/dependencies.cmake)

if(CIFX_HEADER OR CIFX_INC_LIB)
  set(LIBCIFX_INCLUDE_DIRS  ${CIFX_HEADER})
  set(LIBCIFX_LIBRARIES "-pthread -lrt -lcifx -L${CIFX_INC_LIB}")
else(CIFX_HEADER OR CIFX_INC_LIB)
  include(FindPkgConfig)
  pkg_check_modules(LIBCIFX REQUIRED cifx)
endif(CIFX_HEADER OR CIFX_INC_LIB)

aux_source_directory(src/bus SRC_BUS)
aux_source_directory(src/bus/cifx SRC_BUS)
aux_source_directory(src/device SRC_DEVICE)
aux_source_directory(src/kernel SRC_KERNEL)

add_library(BitbotCifx STATIC
            ${SRC_BUS}
            ${SRC_KERNEL}
            ${SRC_DEVICE}
)
target_include_directories(BitbotCifx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${LIBCIFX_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>)
target_compile_features(BitbotCifx PUBLIC cxx_std_20)
target_link_libraries(BitbotCifx
                      m 
                      ${LIBCIFX_LIBRARIES}
                      BitbotKernel
)
