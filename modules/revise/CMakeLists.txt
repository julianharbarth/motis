cmake_minimum_required(VERSION 3.10)
project(motis)

include_directories(include)

file(GLOB_RECURSE motis-revise-files src/*.cc)
add_library(motis-revise STATIC ${motis-revise-files})
target_include_directories(motis-revise PUBLIC include)
target_compile_features(motis-revise PUBLIC cxx_std_17)
target_link_libraries(motis-revise ${Boost_PROGRAM_OPTIONS_LIBRARY} ${Boost_SYSTEM_LIBRARY} motis-core motis-module)
target_compile_options(motis-revise PRIVATE ${MOTIS_CXX_FLAGS})
