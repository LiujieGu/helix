add_executable(test_basic test_basic.cpp)

add_executable(test_lp test_lp.cpp)
target_link_libraries(test_lp PRIVATE helix)

add_executable(test_qp test_qp.cpp)
target_link_libraries(test_qp PRIVATE helix)

add_test(NAME test_basic COMMAND test_basic)
add_test(NAME test_lp    COMMAND test_lp)
add_test(NAME test_qp    COMMAND test_qp)
