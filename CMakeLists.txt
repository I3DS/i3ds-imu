cmake_minimum_required(VERSION 2.8)

################################################################################
# Set some project wide defintions
################################################################################

project (i3ds-imu)

set (PROJECT_AUTHOR "SINTEF Digital")
set (PROJECT_URL "http://i3ds-h2020.eu/")
set (PROJECT_LICENSE "MPLv2")
set (PROJECT_VERSION "1.0")

set (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/scripts)

option(BUILD_EMBEDDED "Build for embedded systems" OFF)
option(GENERATE_ASN "Generate new source files from ASN.1 definitions" ON)
option(I3DS_ALT_PATH  "Alternative location for I3DS libraries (framework, asn1)" OFF)

if (BUILD_EMBEDDED)
  if(NOT DEFINED CMAKE_FIND_ROOT_PATH)
    MESSAGE(FATAL_ERROR "CMAKE_FIND_ROOT_PATH path must be set when compiling for embedded")
  endif(NOT DEFINED CMAKE_FIND_ROOT_PATH)
  if (DEFINED BUILD_SYSROOT)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${BUILD_SYSROOT}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${BUILD_SYSROOT}")
  else ()
    SET(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
    SET(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
  endif ()

endif (BUILD_EMBEDDED)


find_package (Boost COMPONENTS program_options log REQUIRED)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17 -Werror -Wfatal-errors -pedantic -O3")
endif()

include_directories (include)

################################################################################
# ASN.1 generation
################################################################################
if (GENERATE_ASN)
  find_package (ASN1CC REQUIRED)

  if ("${ASN1CC_EXECUTABLE}" STREQUAL "" OR "${ASN1CC_EXECUTABLE}" STREQUAL "ASN1CC_EXECUTABLE-NOTFOUND")
    MESSAGE(FATAL_ERROR "ASN.1 compiler not found.\n"
      "Please specify the ASN1CC_SRC_ROOT_FOLDER or set GENERATE_ASN=OFF")
  endif()
                 
  set (DMU30_DIR ${PROJECT_SOURCE_DIR}/asn/i3ds-dmu30-asn)
  set (ASN1_FILES
    ${DMU30_DIR}/DMU30.asn
    )

  set (ASN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/i3ds")
  asn1cc_generate_c (ASN1_SRCS ASN1_HDRS "${ASN_OUTPUT_DIR}" ${ASN1_FILES})
  include_directories (BEFORE "${CMAKE_CURRENT_BINARY_DIR}")

else (GENERATE_ASN)
  include_directories (BEFORE "generated")

  file(GLOB ASN1_SRCS generated/i3ds/*.c)
  file(GLOB ASN1_HDRS generated/i3ds/*.h)

endif (GENERATE_ASN)

if (I3DS_ALT_PATH)
  add_library(i3ds         SHARED IMPORTED)
  add_library(i3ds_clients SHARED IMPORTED)
  add_library(i3ds_asn1    SHARED IMPORTED)

  set_target_properties( i3ds_asn1    PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${I3DS_ALT_PATH}/lib/libi3ds_asn1.so)
  set_target_properties( i3ds_clients PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${I3DS_ALT_PATH}/lib/libi3ds_clients.so)
  set_target_properties( i3ds         PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${I3DS_ALT_PATH}/lib/libi3ds.so)

  include_directories(BEFORE "${I3DS_ALT_PATH}/include")
endif (I3DS_ALT_PATH)

# For the generated ASN.1 defs for the IMU itself
include_directories (BEFORE "generated/")
file(GLOB ASN1_SRCS generated/i3ds/*.c)
#file(GLOB ASN1_HDRS generated/i3ds/*.h)

################################################################################
# Add sources and targets
################################################################################
set (TARGET i3ds_imu)

set(SRCS
  ${ASN1_SRCS}
  src/imu_dmu30_debug.cpp
  src/imu_dmu30.cpp)

set (LIBS
  zmq
  pthread
  i3ds
  i3ds_clients
  i3ds_asn1
  ${Boost_LIBRARIES}
  )

add_executable (${TARGET} src/i3ds_imu.cpp ${SRCS})

target_link_libraries (${TARGET} ${LIBS})
target_compile_features(${TARGET} PRIVATE cxx_range_for)
target_compile_features(${TARGET} PRIVATE cxx_constexpr)

install(TARGETS ${TARGET} DESTINATION bin)
