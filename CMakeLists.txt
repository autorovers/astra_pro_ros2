cmake_minimum_required(VERSION 3.8)
project(astra_pro_ros2)

# Configurações do compilador (opcional)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic -Wno-ignored-qualifiers -Wno-unused-parameter -Wno-unused-variable)
  set(CMAKE_CXX_STANDARD 20)
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

# ===============================================================
# ==================== ASTRA COLOR PUBLISHER ====================
# ===============================================================
add_executable(color
  src/color.cpp  
)

ament_target_dependencies(color
  rclcpp
  sensor_msgs
  cv_bridge
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
  "${ASTRA_SDK_LIB}/libastra_core_api.so" 
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so"          
)


# ===============================================================
# ==================== ASTRA DEPTH PUBLISHER ====================
# ===============================================================
add_executable(depth 
  src/depth.cpp 
)

ament_target_dependencies(depth
  rclcpp
  sensor_msgs
  cv_bridge
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
  "${ASTRA_SDK_LIB}/libastra_core_api.so" 
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so" 
)


# ===============================================================
# ==================== ASTRA MULTI PUBLISHER ====================
# ===============================================================
add_executable(multi 
  src/multi.cpp 
)

ament_target_dependencies(multi
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

target_link_libraries(multi
  # OpenCv libs
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}

  # Visualization
  ${SFML_LIBRARIES}
  
  # Astra libs
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so"      
)

# # ===============================================================
# # ================ ASTRA multi-D ALIGNED PUBLISHER ===============
# # ===============================================================
# add_executable(multi_aligned 
#   src/multi_aligned.cpp 
# )

# ament_target_dependencies(multi_aligned
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   image_transport
#   std_msgs
# )

# target_link_libraries(multi_aligned
#   # OpenCv libs
#   ${OpenCV_LIBS}
#   ${cv_bridge_LIBRARIES}

#   # Visualization
#   ${SFML_LIBRARIES}
  
#   # Astra libs
#   ${ASTRA_LIBRARIES}
#   "${ASTRA_SDK_LIB}/libastra_core_api.so"
#   "${ASTRA_SDK_LIB}/libastra.so"           
#   "${ASTRA_SDK_LIB}/libastra_core.so"      
# )


# ===============================================================
# ================== ASTRA INFRARED PUBLISHER ===================
# ===============================================================
add_executable(infrared 
  src/infrared.cpp 
)

ament_target_dependencies(infrared
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

target_link_libraries(infrared
  # OpenCv libs
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}

  # Visualization
  ${SFML_LIBRARIES}
  
  # Astra libs
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"
  "${ASTRA_SDK_LIB}/libastra.so"           
  "${ASTRA_SDK_LIB}/libastra_core.so"      
)

# ===============================================================
# ========================= CAMERA INFO =========================
# ===============================================================

add_executable(camera_info_calibration
  src/calibration/camera_info_calibration.cpp 
)

ament_target_dependencies(camera_info_calibration
  rclcpp
  sensor_msgs
)


# ===== EXECUTABLES INSTALLATION =====
set("${PROJECT_NAME}_SOURCES"
  color
  depth
  infrared
  multi 
  camera_info_calibration
)

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