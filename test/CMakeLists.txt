enable_testing(true)

add_custom_target(tests "${CMAKE_CTEST_COMMAND}" "-V" VERBATIM)

foreach(_flag ${FLAGS})
  set(_tgt test_mean_and_covariance_${_flag})
  add_executable(${_tgt} test_mean_and_covariance.cpp)
  target_include_directories(${_tgt} PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(${_tgt} mean_and_covariance ${GTEST_LIBRARIES})
  add_flag(${_tgt} ${_flag})
  add_dependencies(${_tgt} gtest)
  add_dependencies(tests ${_tgt})
  add_test(NAME ${_flag} COMMAND ${_tgt})
endforeach()
