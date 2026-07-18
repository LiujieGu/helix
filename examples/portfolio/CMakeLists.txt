add_executable(helix_portfolio_lp Portfolio_LP.cpp)
target_link_libraries(helix_portfolio_lp PRIVATE helix::helix)
target_compile_options(helix_portfolio_lp PRIVATE -Wall -Wextra -Wpedantic)

add_executable(helix_portfolio_qp Portfolio_QP.cpp)
target_link_libraries(helix_portfolio_qp PRIVATE helix::helix)
target_compile_options(helix_portfolio_qp PRIVATE -Wall -Wextra -Wpedantic)
