cmake_minimum_required(VERSION 3.10)
project(quadsquad2026)

file(GLOB LIB_SOURCES "src/QCLib/*.cpp")

add_library(quadcopter SHARED
    ${LIB_SOURCES}
)

set_target_properties(quadcopter PROPERTIES PUBLIC_HEADER include/Quadcopter.h)


target_include_directories(quadcopter PRIVATE include)
target_include_directories(quadcopter PRIVATE src)

include(GNUInstallDirs)

install(TARGETS quadcopter
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


################ TEST EXECUTABLE ################ (must have the exact same configuration as simulate)

# file(GLOB TEST_SOURCES "src/QCLib/*.cpp" "src/test.cpp")
# add_executable(test ${TEST_SOURCES})

# target_include_directories(test PRIVATE
#     ${foxglove_SOURCE_DIR}/include
#     ${foxglove_SOURCE_DIR}/include/foxglove
#     ${PROJECT_SOURCE_DIR}/include
# )

# target_sources(test PRIVATE ${FOXGLOVE_SOURCES})

# target_link_libraries(test PRIVATE
#     ${foxglove_SOURCE_DIR}/lib/libfoxglove.a
#     pthread
# )