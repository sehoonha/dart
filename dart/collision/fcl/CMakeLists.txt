# Search all header and source files
file(GLOB srcs "*.cpp")
file(GLOB hdrs "*.h")

set(dart_collision_hdrs ${dart_collision_hdrs} ${hdrs} PARENT_SCOPE)
set(dart_collision_srcs ${dart_collision_srcs} ${srcs} PARENT_SCOPE)

# Library
#dart_add_library(dart_collision_fcl ${srcs} ${hdrs})
#target_link_libraries(dart_collision_fcl ${DART_CORE_DEPENDENCIES})

# Generate header for this namespace
dart_get_filename_components(header_names "collision_fcl headers" ${hdrs})
dart_generate_include_header_list(
  collision_fcl_headers
  "dart/collision/fcl/"
  "collision_fcl headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/fcl.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/fcl.h
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/fcl.h
  DESTINATION include/dart/collision/fcl
  COMPONENT headers
)
#install(TARGETS dart_collision_fcl EXPORT DARTCoreTargets DESTINATION lib)
#install(TARGETS dart_collision_fcl EXPORT DARTTargets DESTINATION lib)

