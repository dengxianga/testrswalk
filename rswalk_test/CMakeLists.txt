include_directories(/home/nao/testunsw/testunsw/rswalk2014)
link_directories(/home/nao/testunsw/testunsw/rswalk2014)

add_executable(test_unsw test_unsw.cpp)
target_link_libraries(test_unsw rswalk2014) # librswalk2014.so is found in ${CMAKE_SOURCE_DIR}/lib
