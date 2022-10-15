SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR RISCV)

if(MINGW OR CYGWIN OR WIN32)
    set(WHERE_CMD where)
    set(TOOLCHAIN_SUFFIX ".exe")
elseif(UNIX OR APPLE)
    set(WHERE_CMD which)
    set(TOOLCHAIN_SUFFIX "")
endif()

# message(STATUS "TOOLCHAIN_PREFIX:${CONFIG_TOOLCHAIN}")
# set(TOOLCHAIN_PREFIX riscv64-unknown-elf-)
set(TOOLCHAIN_PREFIX ${CONFIG_TOOLCHAIN})

execute_process(
  COMMAND ${WHERE_CMD} ${TOOLCHAIN_PREFIX}gcc
  # COMMAND ${WHERE_CMD} ${CONFIG_TOOLCHAIN}gcc
  OUTPUT_VARIABLE TOOLCHAIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "TOOLCHAIN_PATH:${TOOLCHAIN_PATH}")
# specify cross compilers and tools
SET(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}ld${TOOLCHAIN_SUFFIX} CACHE INTERNAL "")
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy CACHE INTERNAL "")
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump CACHE INTERNAL "")
set(SIZE ${TOOLCHAIN_PREFIX}size CACHE INTERNAL "")

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})
# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)