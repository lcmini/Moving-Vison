add_library(AMOV_Gimbal ${LIB_FLAG})

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -fPIC")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fPIC")

add_definitions(
    -DMAX_QUEUE_SIZE=100
)

add_subdirectory(FIFO)
########## add types of gimbal ##############
add_subdirectory(G1)
add_subdirectory(G2)
add_subdirectory(Q10f)
add_subdirectory(AT10)

file(GLOB LIB_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

target_sources(AMOV_Gimbal
    PRIVATE
    ${LIB_FILES}
)

target_link_libraries(AMOV_Gimbal
    PRIVATE
    Gimabl_G1
    Gimabl_G2
    Gimabl_Q10f
    Gimabl_AT10
)

target_include_directories(AMOV_Gimbal
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

