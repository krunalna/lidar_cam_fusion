cmake_minimum_required(VERSION 3.22)
project(perception_pipeline_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

# Conda prefix on PATH so find_package can locate OpenCV, etc.
list(APPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")

# ── ROS 2 ─────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── OpenCV ────────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# ── macOS: stub libatomic ─────────────────────────────────────────────────────
# robostack's rcutils embeds -latomic in its link flags. On macOS/clang, atomic
# ops are built into the compiler and no standalone libatomic exists. Create an
# empty stub library so the linker is satisfied.
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
endif()

# ── kitti_publisher_cpp ───────────────────────────────────────────────────────
add_executable(kitti_publisher_cpp
  src/kitti_reader.cpp
  src/kitti_publisher_node.cpp
)
target_include_directories(kitti_publisher_cpp
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(kitti_publisher_cpp
  ${OpenCV_LIBS}
)
ament_target_dependencies(kitti_publisher_cpp
  rclcpp
  sensor_msgs
  std_msgs
)
if(APPLE)
  add_dependencies(kitti_publisher_cpp _atomic_stub)
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS kitti_publisher_cpp
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
  ament_add_gtest(test_kitti_reader
    test/test_kitti_reader.cpp
    src/kitti_reader.cpp
  )
  target_include_directories(test_kitti_reader PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_kitti_reader ${OpenCV_LIBS})
  if(APPLE)
    add_dependencies(test_kitti_reader _atomic_stub)
  endif()
endif()

ament_package()
