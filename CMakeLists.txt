cmake_minimum_required(VERSION 3.8)
project(tpic6c595_test C)

set(CMAKE_C_STANDARD 99)

include_directories(tpic6c595_test PUBLIC /home/apixdev/VolkeOS_4.14.134-rt63_includes)

add_executable(tpic6c595_test tpic6c595.h tpic6c595_test.c)
target_link_libraries(tpic6c595_test PRIVATE /usr/local/lib/libvolke.a)