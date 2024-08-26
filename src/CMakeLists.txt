file(GLOB foxglove_files
  foxglove_server.cpp 
  proto_utility.cpp 
  utility.cpp
  foxglove_utils.cpp
  visualizer.cpp
)

# visualization protos
set(PROTO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto/foxglove/proto)
file(GLOB FG_VIZ_PROTO_FILES ${PROTO_DIR}/*.proto)
FOREACH(proto ${FG_VIZ_PROTO_FILES})
  FILE(TO_NATIVE_PATH ${proto} proto_native)
  EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
   --proto_path=${PROTO_ROOT_DIR} --cpp_out=${PROTO_ROOT_DIR} ${proto_native}
  RESULT_VARIABLE rv)
  IF(${rv})
      MESSAGE("Generation of data model returned ${rv} for proto ${proto_native}")
  ENDIF()
ENDFOREACH(proto)

file(GLOB FG_VIZ_PB_SRC ${PROTO_DIR}/*.cc)
file(GLOB FG_VIZ_PB_HEADER ${PROTO_DIR}/*.h)
file(GLOB FG_HEADER ${PROJECT_SOURCE_DIR}/include/foxglove/*.h)

add_library(${PROJECT_NAME}_lib SHARED ${foxglove_files} ${FG_VIZ_PB_SRC})
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC USE_BOOST_ASIO)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${PROTOBUF_LIBRARIES} ${common_thirdparty_libs})
target_include_directories(${PROJECT_NAME}_lib PUBLIC 
  $<BUILD_INTERFACE:${PROTO_ROOT_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

install(FILES
  ${FG_VIZ_PB_HEADER}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/foxglove/proto
)

install(FILES
  ${FG_HEADER}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/foxglove/
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT foxglove-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(EXPORT foxglove-targets
  FILE ${PROJECT_NAME}_lib-config.cmake
  DESTINATION share/cmake/${PROJECT_NAME}_lib/
)