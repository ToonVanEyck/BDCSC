add_executable(${PROJECT_NAME}_test test.c)

add_library(unity STATIC /opt/unity/src/unity.c)
target_include_directories(unity PUBLIC /opt/unity/src)

target_link_libraries(${PROJECT_NAME}_test
  rbuff
  unity
)

add_test(${PROJECT_NAME}_test ${PROJECT_NAME}_test)
