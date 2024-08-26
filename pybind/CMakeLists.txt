find_package(pybind11 REQUIRED)
set(PYBIND11_LTO_CXX_FLAGS "")

message(STATUS "Python_EXECUTABLE: ${PYTHON_EXECUTABLE}")
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Numpy include dir: ${NUMPY_INCLUDE_DIR}")

include_directories("${NUMPY_INCLUDE_DIR}")

pybind11_add_module(pyfoxglove foxglove_binds.cpp ndarray_converter.cpp)
target_link_libraries(pyfoxglove PUBLIC ${PYTHON_LIBRARIES} ${PROJECT_NAME}_lib pthread)
install(TARGETS pyfoxglove
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)