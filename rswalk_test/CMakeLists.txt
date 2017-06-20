#include_directories(/home/nao/testrswalk/rswalk2014)
#link_directories(/home/nao/testrswalk/rswalk2014 /home/nao/local/lib)
include_directories(/home/dx/Desktop/testrswalk/rswalk2014)
link_directories(/home/dx/Desktop/testrswalk/rswalk2014 /usr/local/lib)

add_executable(test_unsw test_RSWalkModule2014.cpp rswalkhack.cpp)
target_link_libraries(test_unsw rswalk2014 zmq) # librswalk2014.so is found in ${CMAKE_SOURCE_DIR}/lib

MESSAGE("CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})
