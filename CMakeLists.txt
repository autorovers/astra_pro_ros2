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

set (${_projname}_SOURCES
  src/color.cpp
)

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

# # ===== INCLUDE DIRS =====
include_directories(
  ${ASTRA_SDK_INCLUDE}       # Headers da Astra
  ${OpenCV_INCLUDE_DIRS}     # Headers do OpenCV
  ${SFML_INCLUDE_DIRS}
  ${SAMPLE_COMMON_DIR}
)

# ===== EXECUTÁVEIS =====

# ===============================================================
# ==================== ASTRA COLOR PUBLISHER ====================
# ===============================================================
add_executable(color
  ${${_projname}_SOURCES}  
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
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${SFML_LIBRARIES}
  
  # Bibliotecas da Astra (verifique os nomes no diretório /lib/)
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"  # Principal
  "${ASTRA_SDK_LIB}/libastra.so"           # Se existir
  "${ASTRA_SDK_LIB}/libastra_core.so"      # Se existir
  # Outras dependências do sistema (se necessário)
  -ludev                                   # Para acesso USB (comum em drivers de câmera)
)

install(TARGETS color DESTINATION lib/${PROJECT_NAME})


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
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${SFML_LIBRARIES}
  
  # Bibliotecas da Astra (verifique os nomes no diretório /lib/)
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"  # Principal
  "${ASTRA_SDK_LIB}/libastra.so"           # Se existir
  "${ASTRA_SDK_LIB}/libastra_core.so"      # Se existir
  # Outras dependências do sistema (se necessário)
  -ludev                                   # Para acesso USB (comum em drivers de câmera)
)

install(TARGETS depth DESTINATION lib/${PROJECT_NAME})


# ===============================================================
# ==================== ASTRA MULTI PUBLISHER ====================
# ===============================================================
add_executable(multi 
  src/multi.cpp 
)

# ===== LINKAGEM =====
target_include_directories(multi PUBLIC
  ${cv_bridge_INCLUDE_DIRS}
)

set_target_properties(multi PROPERTIES FOLDER "${SAMPLE_DIR_FOLDER}/cpp-api")

ament_target_dependencies(multi
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  std_msgs
)

target_link_libraries(multi
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${SFML_LIBRARIES}
  
  # Bibliotecas da Astra (verifique os nomes no diretório /lib/)
  ${ASTRA_LIBRARIES}
  "${ASTRA_SDK_LIB}/libastra_core_api.so"  # Principal
  "${ASTRA_SDK_LIB}/libastra.so"           # Se existir
  "${ASTRA_SDK_LIB}/libastra_core.so"      # Se existir
  # Outras dependências do sistema (se necessário)
  -ludev                                   # Para acesso USB (comum em drivers de câmera)
)

install(TARGETS multi DESTINATION lib/${PROJECT_NAME})

# ===== INSTALAÇÃO =====

# Instala as bibliotecas da Astra junto com seu pacote
install(DIRECTORY ${ASTRA_SDK_LIB}/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

ament_package()