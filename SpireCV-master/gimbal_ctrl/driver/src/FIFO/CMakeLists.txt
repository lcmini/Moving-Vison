add_library(FIFO)

file(GLOB LIB_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

target_sources(FIFO
    PRIVATE
    ${LIB_FILES}
)

target_include_directories(FIFO
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)