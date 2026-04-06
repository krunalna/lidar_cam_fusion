cmake_minimum_required(VERSION 3.22)
project(perception_pipeline_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# Conda prefix so find_package can locate OpenCV, ONNX Runtime, PCL, etc.
list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
set(CONDA_LIB_DIR "$ENV{CONDA_PREFIX}/lib")

# ── numpy _core/include workaround ────────────────────────────────────────────
# rclcpp's cmake config lists numpy's _core/include in INTERFACE_INCLUDE_DIRECTORIES.
# After a fresh pixi install (or env wipe) that directory is sometimes missing.
# Create it so cmake doesn't error on a non-existent path.
set(_numpy_core_inc
  "$ENV{CONDA_PREFIX}/lib/python3.12/site-packages/numpy/_core/include")
if(NOT EXISTS "${_numpy_core_inc}")
  file(MAKE_DIRECTORY "${_numpy_core_inc}")
  message(STATUS "Created missing numpy _core/include stub: ${_numpy_core_inc}")
endif()

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

# ── PCL ───────────────────────────────────────────────────────────────────────
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

# ── Eigen3 ────────────────────────────────────────────────────────────────────
find_package(Eigen3 REQUIRED)

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ── ONNX Runtime ─────────────────────────────────────────────────────────────
# The conda-forge 'onnxruntime' package ships the runtime library through the
# Python package, but not the C++ headers we need for compilation. We therefore:
#   1. Locate the runtime shared library and version via the Python package.
#   2. Download the matching official prebuilt archive for headers using
#      FetchContent (one-time, cached in the build dir).

find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Step 1 — version
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
    "import onnxruntime; print(onnxruntime.__version__)"
  OUTPUT_VARIABLE ORT_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _ort_ver_rc
)

# Step 2 — shared library path
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
    "import onnxruntime, os, glob; \
     capi = os.path.join(os.path.dirname(onnxruntime.__file__), 'capi'); \
     patterns = ['libonnxruntime*.dylib', 'libonnxruntime.so*']; \
     libs = []; \
     [libs.extend(glob.glob(os.path.join(capi, pattern))) for pattern in patterns]; \
     libs = [p for p in sorted(libs) if 'providers' not in os.path.basename(p)]; \
     print(libs[0] if libs else '')"
  OUTPUT_VARIABLE ORT_SHARED_LIB
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(APPLE)
  set(ORT_PREBUILT_ARCHIVE "onnxruntime-osx-arm64-${ORT_VERSION}.tgz")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  set(ORT_PREBUILT_ARCHIVE "onnxruntime-linux-x64-${ORT_VERSION}.tgz")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(ORT_PREBUILT_ARCHIVE "onnxruntime-linux-aarch64-${ORT_VERSION}.tgz")
else()
  message(FATAL_ERROR
    "Unsupported platform for automatic ONNX Runtime header download: "
    "${CMAKE_SYSTEM_NAME} / ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(_ort_ver_rc OR NOT ORT_VERSION OR NOT EXISTS "${ORT_SHARED_LIB}")
  message(FATAL_ERROR
    "ONNX Runtime Python package not found in the pixi environment.\n"
    "Run: pixi install")
endif()
get_filename_component(ORT_SHARED_LIB_DIR "${ORT_SHARED_LIB}" DIRECTORY)
get_filename_component(ORT_SHARED_LIB_NAME "${ORT_SHARED_LIB}" NAME)
message(STATUS "ONNX Runtime: version=${ORT_VERSION}  shared_lib=${ORT_SHARED_LIB}")

# Step 3 — download matching headers via FetchContent (one-time, CMake-cached)
include(FetchContent)
# CMP0169: allow calling FetchContent_Populate directly for non-cmake archives
cmake_policy(SET CMP0169 OLD)
FetchContent_Declare(ort_prebuilt
  URL "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/${ORT_PREBUILT_ARCHIVE}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(ort_prebuilt)
if(NOT ort_prebuilt_POPULATED)
  message(STATUS "Downloading ONNX Runtime ${ORT_VERSION} headers (one-time) ...")
  FetchContent_Populate(ort_prebuilt)
endif()

# Step 4 — create import target
add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION             "${ORT_SHARED_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${ort_prebuilt_SOURCE_DIR}/include")
message(STATUS "ONNX Runtime headers: ${ort_prebuilt_SOURCE_DIR}/include")

# ── macOS: flat-namespace Python preload + libatomic stub ─────────────────────
# robostack's rosidl_generator_py libs use flat-namespace lookup for Python
# symbols. On macOS the binary must explicitly link libpython so it is loaded
# before those dylibs, making _PyExc_* symbols available in the flat namespace.
# robostack's rcutils also embeds -latomic which doesn't exist on macOS/clang;
# we satisfy the linker with an empty stub.
if(APPLE)
  set(_stub_dir "${CMAKE_BINARY_DIR}/macOS_stubs")
  file(MAKE_DIRECTORY "${_stub_dir}")
  file(WRITE "${_stub_dir}/atomic_stub.c"
    "// macOS stub: clang has atomic operations built in\n"
    "void __macOS_atomic_stub__(void) {}\n")
  add_library(_atomic_stub STATIC "${_stub_dir}/atomic_stub.c")
  set_target_properties(_atomic_stub PROPERTIES
    OUTPUT_NAME atomic
    ARCHIVE_OUTPUT_DIRECTORY "${_stub_dir}")
  link_directories("${_stub_dir}")

  # Find and record the Python library path for flat-namespace preloading
  find_library(_python_lib python3.12 HINTS "$ENV{CONDA_PREFIX}/lib")
endif()

# ── calib_utils (Phase 5 — static library, no ROS dependency) ────────────────
add_library(calib_utils STATIC
  src/calibration.cpp
  src/projector.cpp
  src/fusion_engine.cpp
)
target_include_directories(calib_utils
  PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(calib_utils
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# ── lidar_processor_cpp ───────────────────────────────────────────────────────
add_executable(lidar_processor_cpp
  src/lidar_preprocessor.cpp
  src/lidar_processor_node.cpp
)
target_include_directories(lidar_processor_cpp
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)
target_link_directories(lidar_processor_cpp PRIVATE "${CONDA_LIB_DIR}")
target_compile_definitions(lidar_processor_cpp PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(lidar_processor_cpp
  ${PCL_LIBRARIES}
  $<$<PLATFORM_ID:Darwin>:${_python_lib}>
)
ament_target_dependencies(lidar_processor_cpp
  rclcpp
  sensor_msgs
  std_msgs
)
if(APPLE)
  add_dependencies(lidar_processor_cpp _atomic_stub)
endif()

# ── camera_detector_cpp ───────────────────────────────────────────────────────
add_executable(camera_detector_cpp
  src/camera_detector.cpp
  src/camera_detector_node.cpp
)
target_include_directories(camera_detector_cpp
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(camera_detector_cpp PRIVATE "${CONDA_LIB_DIR}")
target_link_libraries(camera_detector_cpp
  onnxruntime::onnxruntime
  ${OpenCV_LIBS}
  $<$<PLATFORM_ID:Darwin>:${_python_lib}>
)
set_target_properties(camera_detector_cpp PROPERTIES
  BUILD_RPATH "$ORIGIN;${ORT_SHARED_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN;${ORT_SHARED_LIB_DIR}"
)
if(UNIX AND NOT APPLE)
  add_custom_command(TARGET camera_detector_cpp POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E create_symlink
            "${ORT_SHARED_LIB}"
            "$<TARGET_FILE_DIR:camera_detector_cpp>/${ORT_SHARED_LIB_NAME}"
    COMMAND "${CMAKE_COMMAND}" -E create_symlink
            "${ORT_SHARED_LIB_NAME}"
            "$<TARGET_FILE_DIR:camera_detector_cpp>/libonnxruntime.so.1"
    COMMAND "${CMAKE_COMMAND}" -E create_symlink
            "${ORT_SHARED_LIB_DIR}/libonnxruntime_providers_shared.so"
            "$<TARGET_FILE_DIR:camera_detector_cpp>/libonnxruntime_providers_shared.so"
    VERBATIM
  )
endif()
ament_target_dependencies(camera_detector_cpp
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
)
if(APPLE)
  add_dependencies(camera_detector_cpp _atomic_stub)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
# ── fusion_node_cpp ───────────────────────────────────────────────────────────
add_executable(fusion_node_cpp
  src/fusion_node.cpp
)
target_include_directories(fusion_node_cpp
  PRIVATE include ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(fusion_node_cpp
  calib_utils
  ${OpenCV_LIBS}
  $<$<PLATFORM_ID:Darwin>:${_python_lib}>
)
ament_target_dependencies(fusion_node_cpp
  rclcpp
  sensor_msgs
  vision_msgs
  geometry_msgs
  message_filters
  visualization_msgs
)
if(APPLE)
  add_dependencies(fusion_node_cpp _atomic_stub)
endif()

install(TARGETS calib_utils lidar_processor_cpp camera_detector_cpp fusion_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)
if(UNIX AND NOT APPLE)
  install(FILES
    "${ORT_SHARED_LIB}"
    "${ORT_SHARED_LIB_DIR}/libonnxruntime_providers_shared.so"
    DESTINATION lib/${PROJECT_NAME}
  )
  install(CODE
    "execute_process(
      COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
              \"${ORT_SHARED_LIB_NAME}\"
              \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/libonnxruntime.so.1\")"
  )
endif()
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Resolve path to calibration.yaml (installed alongside perception_pipeline)
  set(_calib_yaml
    "${CMAKE_CURRENT_SOURCE_DIR}/../perception_pipeline/config/calibration.yaml")
  if(NOT EXISTS "${_calib_yaml}")
    # Fallback: installed share directory
    set(_calib_yaml
      "${CMAKE_INSTALL_PREFIX}/../perception_pipeline/share/perception_pipeline/config/calibration.yaml")
  endif()

  ament_add_gtest(test_projector test/test_projector.cpp)
  target_link_libraries(test_projector calib_utils)
  target_compile_definitions(test_projector PRIVATE
    CALIBRATION_YAML_PATH="${_calib_yaml}")

  ament_add_gtest(test_fusion_engine test/test_fusion_engine.cpp)
  target_link_libraries(test_fusion_engine calib_utils)
endif()

ament_package()
