add_library(serial-interface SerialInterface.cpp)

add_executable(jeep main.cpp)
target_include_directories(jeep PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(jeep serial-interface)
