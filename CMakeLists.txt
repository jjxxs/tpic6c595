cmake_minimum_required(VERSION 3.8)
project(tpic6c595 C)

set(CMAKE_C_STANDARD 99)

include_directories(inap375t PUBLIC /home/apixdev/VolkeOS_4.14.134-rt63_includes)

add_executable(tpic6c595 tpic6c595_test.c)
target_link_libraries(tpic6c595 PRIVATE /usr/local/lib/libvolke.a)