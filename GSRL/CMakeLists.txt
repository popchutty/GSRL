cmake_minimum_required(VERSION 3.22)

project(gsrl LANGUAGES C CXX)

add_library(${PROJECT_NAME} OBJECT)

if(NOT DEFINED BSP_ROOT)
    set(BSP_ROOT "CubeMX_BSP") # Default: F4 BSP folder
endif()

set(CMSISCORE "${CMAKE_SOURCE_DIR}/${BSP_ROOT}/Drivers/CMSIS/Core")
add_subdirectory(Dependence/CMSIS-DSP)
add_subdirectory(Dependence/eigen)

file(GLOB_RECURSE GSRL_ALGORITHM_SOURCES
    "Algorithm/src/*.c"
    "Algorithm/src/*.cpp"
)

file(GLOB_RECURSE GSRL_DEVICE_SOURCES
    "Device/src/*.c"
    "Device/src/*.cpp"
)

file(GLOB_RECURSE GSRL_DRIVER_SOURCES
    "Driver/src/*.c"
    "Driver/src/*.cpp"
)

target_sources(${PROJECT_NAME} PRIVATE
    ${GSRL_ALGORITHM_SOURCES}
    ${GSRL_DEVICE_SOURCES}
    ${GSRL_DRIVER_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    Include
    Algorithm/inc
    Device/inc
    Driver/inc
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    EIGEN_NO_MALLOC
)

target_compile_features(${PROJECT_NAME} PUBLIC
    c_std_11
    cxx_std_17
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    stm32cubemx
    CMSISDSP
    eigen
)
