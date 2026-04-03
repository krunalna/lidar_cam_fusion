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

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── PCL ───────────────────────────────────────────────────────────────────────
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

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

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ── lidar_processor_cpp ──────────────────────────────────────────────────────
add_executable(lidar_processor_cpp
  src/lidar_preprocessor.cpp
  src/lidar_processor_node.cpp
)
target_include_directories(lidar_processor_cpp
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
)
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

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS lidar_processor_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
