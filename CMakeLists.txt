cmake_minimum_required(VERSION 3.8)
project(astra_pro_ros2)

# Configurações do compilador (opcional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== ASTRA SDK CONFIG =====
# Define manualmente os caminhos (substitua se necessário)
set(ASTRA_SDK_INCLUDE "/root/studies_ws/src/AstraPro/include")
set(ASTRA_SDK_LIB "/root/studies_ws/src/AstraPro/lib")
set(SAMPLE_DIR_FOLDER "/root/studies_ws/src/AstraPro/samples")
set(SAMPLE_COMMON_DIR "/root/studies_ws/src/AstraPro/samples/common")
set (_projname "astra_pro_ros2")

# Verifica se os diretórios existem
if(NOT EXISTS ${ASTRA_SDK_INCLUDE})
  message(FATAL_ERROR "ASTRA SDK include dir not found: ${ASTRA_SDK_INCLUDE}")
endif()

if(NOT EXISTS ${ASTRA_SDK_LIB})
  message(FATAL_ERROR "ASTRA SDK lib dir not found: ${ASTRA_SDK_LIB}")
endif()

# ===== DEPENDÊNCIAS ROS 2 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Configura pkg-config para encontrar SFML (se a Astra SDK precisar)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SFML REQUIRED sfml-all)  # Ou sfml-graphics, sfml-window, etc.

# ===== INCLUDE DIRS =====
include_directories(
  ${ASTRA_SDK_INCLUDE}       # Headers da Astra
  ${OpenCV_INCLUDE_DIRS}     # Headers do OpenCV
  ${SFML_INCLUDE_DIRS}
  ${SAMPLE_COMMON_DIR}
)

# ===== EXECUTÁVEIS =====

set("${PROJECT_NAME}_SOURCES"
  color
  depth
  rgbd 
  camera_info_calibration
)

# ===============================================================
# ==================== ASTRA COLOR PUBLISHER ====================
# ===============================================================
add_executable(color
  src/color.cpp  
)

# ===== LINKAGEM =====
target_include_directories(color PUBLIC
  ${cv_bridge_INCLUDE_DIRS}
)

set_target_properties(color PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}/cpp-api")

ament_target_dependencies(color
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  std_msgs
)

target_link_libraries(color
  # OpenCv libs
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}

  # Visualization
  ${SFML_LIBRARIES}
  
  # Astra libs
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"  # Main
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so"          
)


# ===============================================================
# ==================== ASTRA DEPTH PUBLISHER ====================
# ===============================================================
add_executable(depth 
  src/depth.cpp 
)

# ===== LINKAGEM =====
target_include_directories(depth PUBLIC
  ${cv_bridge_INCLUDE_DIRS}
)

set_target_properties(depth PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}/cpp-api")

ament_target_dependencies(depth
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  std_msgs
)

target_link_libraries(depth
  # OpenCv libs
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}

  # Visualization
  ${SFML_LIBRARIES}
  
  # Astra libs
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"  # Main
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so" 
)


# ===============================================================
# ==================== ASTRA RGBD-D PUBLISHER ===================
# ===============================================================
add_executable(rgbd 
  src/rgbd.cpp 
)

# ===== LINKAGEM =====
target_include_directories(rgbd PUBLIC
  ${cv_bridge_INCLUDE_DIRS}
)

set_target_properties(rgbd PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}/cpp-api")

ament_target_dependencies(rgbd
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  std_msgs
)

target_link_libraries(rgbd
  # OpenCv libs
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}

  # Visualization
  ${SFML_LIBRARIES}
  
  # Astra libs
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"  # Main
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so"      
)

# ===============================================================
# ========================= CAMERA INFO =========================
# ===============================================================

add_executable(camera_info_calibration
  src/camera_info_calibration.cpp 
)

ament_target_dependencies(camera_info_calibration
  rclcpp
  sensor_msgs
)

# ===== EXECUTABLES INSTALLATION =====
install(TARGETS ${${PROJECT_NAME}_SOURCES}
  DESTINATION
  lib/${PROJECT_NAME}
)

# ===== LIBS INSTALLATION =====
install(DIRECTORY ${ASTRA_SDK_LIB}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

# ===== LAUNCH INSTALLATION =====
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()