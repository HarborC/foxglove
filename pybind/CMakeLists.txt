find_package(pybind11 REQUIRED)
set(PYBIND11_LTO_CXX_FLAGS "")

pybind11_add_module(pyfoxglove foxglove_binds.cpp)
target_link_libraries(pyfoxglove PUBLIC ${PYTHON_LIBRARIES} ${PROJECT_NAME}_lib pthread)
install(TARGETS pyfoxglove
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)