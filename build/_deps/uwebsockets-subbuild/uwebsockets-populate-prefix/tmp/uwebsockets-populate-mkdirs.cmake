# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-src")
  file(MAKE_DIRECTORY "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-src")
endif()
file(MAKE_DIRECTORY
  "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-build"
  "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix"
  "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix/tmp"
  "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix/src/uwebsockets-populate-stamp"
  "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix/src"
  "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix/src/uwebsockets-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix/src/uwebsockets-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mxqwthl/bitbot_cifx_demo/build/_deps/uwebsockets-subbuild/uwebsockets-populate-prefix/src/uwebsockets-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
