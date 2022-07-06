# The set of built-in models should be very small. Only the most essential
# models needed to make Gazebo run without an internet connection should be
# included here.
set (dirs
  ground_plane
  sun
)

foreach(dir ${dirs})
  install (DIRECTORY ${dir} DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/choi_stella_cartographer-${GAZEBO_VERSION}/models)
endforeach()