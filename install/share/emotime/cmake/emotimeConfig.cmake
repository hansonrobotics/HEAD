# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(emotime_CONFIG_INCLUDED)
  return()
endif()
set(emotime_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(emotime_SOURCE_PREFIX /home/icog-labs/hansonrobotics/HEAD/src/vision/emotime_ros/emotime)
  set(emotime_DEVEL_PREFIX /home/icog-labs/hansonrobotics/HEAD/devel/.private/emotime)
  set(emotime_INSTALL_PREFIX "")
  set(emotime_PREFIX ${emotime_DEVEL_PREFIX})
else()
  set(emotime_SOURCE_PREFIX "")
  set(emotime_DEVEL_PREFIX "")
  set(emotime_INSTALL_PREFIX /home/icog-labs/hansonrobotics/HEAD/install)
  set(emotime_PREFIX ${emotime_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'emotime' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(emotime_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(emotime_INCLUDE_DIRS "")
  set(_include_dirs "")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${emotime_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'emotime' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'lina <lina@todo.todo>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'emotime' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/icog-labs/hansonrobotics/HEAD/install/${idir}'.  Ask the maintainer 'lina <lina@todo.todo>' to fix it.")
    endif()
    _list_append_unique(emotime_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND emotime_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND emotime_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND emotime_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/icog-labs/hansonrobotics/HEAD/install/lib;/home/icog-labs/hansonrobotics/HEAD/devel/lib;/opt/ros/indigo/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(emotime_LIBRARY_DIRS ${lib_path})
      list(APPEND emotime_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'emotime'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND emotime_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(emotime_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${emotime_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 emotime_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${emotime_dep}_FOUND)
      find_package(${emotime_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${emotime_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(emotime_INCLUDE_DIRS ${${emotime_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(emotime_LIBRARIES ${emotime_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${emotime_dep}_LIBRARIES})
  _list_append_deduplicate(emotime_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(emotime_LIBRARIES ${emotime_LIBRARIES})

  _list_append_unique(emotime_LIBRARY_DIRS ${${emotime_dep}_LIBRARY_DIRS})
  list(APPEND emotime_EXPORTED_TARGETS ${${emotime_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${emotime_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
