add_custom_target(benchmarks "bash" "${CMAKE_CURRENT_LIST_DIR}/benchmark.sh" "bench_mean_and_covariance" "${COMPILER}" "${FLAGS}" VERBATIM)

foreach(_precision float double)
  foreach(_flag ${FLAGS})
    set(_tgt bench_mean_and_covariance_${_precision}_${_flag})
    add_executable(${_tgt} bench_mean_and_covariance.cpp)
    target_link_libraries(${_tgt} mean_and_covariance ${CELERO_LIBRARIES} ${PCL_LIBRARIES})
    target_include_directories(${_tgt} PRIVATE SYSTEM ${CELERO_INCLUDE_DIRS})
    target_compile_options(${_tgt} PRIVATE "-save-temps")
    target_compile_definitions(${_tgt} PRIVATE "-DPRECISION=${_precision}")
    set_target_properties(${_tgt} PROPERTIES INSTALL_RPATH ${CELERO_RPATH} BUILD_WITH_INSTALL_RPATH TRUE)
    add_flag(${_tgt} ${_flag})
    add_dependencies(${_tgt} celero)
    add_dependencies(benchmarks ${_tgt})
    add_custom_command(
      TARGET ${_tgt}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/bench_mean_and_covariance.s" "${CMAKE_CURRENT_BINARY_DIR}/${_tgt}.s"
      COMMENT "Saving disassembly for ${_tgt}..."
    )
  endforeach()
endforeach()
