#cmake version
cmake_minimum_required(VERSION 3.5...3.15)

if(NOT TARGET ${RamRodArduinoSerial_LIBRARIES})
  project(RamRodArduinoSerial
    VERSION 1.0.0
    DESCRIPTION "Ram-Rod's arduino serial communication"
    LANGUAGES CXX C
  )

  # Boost library
  find_package(Boost REQUIRED COMPONENTS system chrono filesystem thread)

  #header files
  set(HPP_FILES
    include/ramrod/arduino/constants.h
    include/ramrod/arduino/serial.h
  )

  #source files
  set(CPP_FILES
    src/ramrod/arduino/serial.cpp
  )

  # // ------------------------------------------------------------------------------------ //
  # // ------------------------------------ START LIBS ------------------------------------ //
  # // ------------------------------------------------------------------------------------ //


  # // ------------------------------------------------------------------------------------ //
  # // ------------------------------------ END LIBS -------------------------------------- //
  # // ------------------------------------------------------------------------------------ //

  set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL ""
  )

  set(${PROJECT_NAME}_LIBRARIES
    ${PROJECT_NAME}
    CACHE INTERNAL ""
  )

  #add every file to the executable
  add_library(${PROJECT_NAME})

  target_sources(${PROJECT_NAME}
    PRIVATE
      ${CPP_FILES}
      ${HPP_FILES}
  )

  target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

  set_target_properties(${PROJECT_NAME}
    PROPERTIES
      CXX_STANDARD 17
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
  )

  target_include_directories(${PROJECT_NAME}
    PRIVATE
      include
      ${Boost_INCLUDE_DIRS}
  )

  #indicates which libraries to use in the executable
  target_link_libraries(${PROJECT_NAME}
    # Boost library linkage
    ${Boost_LIBRARIES}
  )

  # TODO: delete this after finished
  if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_options(${PROJECT_NAME}
      PRIVATE
      -Wall
      -Wextra
      -Wundef
      -pedantic
      -Wconversion
      -Wcast-align
      -Wfloat-equal
      -Wpointer-arith
      -Wsign-conversion
      -Wunreachable-code
      -Wunused-function
      -Wignored-qualifiers
    )

    ##########################
    ## include-what-you-use ##
    ##########################

    find_program(iwyu_path NAMES include-what-you-use iwyu)
    if(NOT iwyu_path)
      message(FATAL_ERROR "Could not find the program include-what-you-use")
    else()
      message(STATUS "include-what-you-use found: ${iwyu_path}")
      get_filename_component(PARENT_DIR ${iwyu_path} DIRECTORY)
      get_filename_component(G_PARENT_DIR ${PARENT_DIR} DIRECTORY)
      set(iwyu_share_path ${G_PARENT_DIR}/share/include-what-you-use)
      message("iwyu share path: ${iwyu_share_path}")
      set(iwyu_path_and_options
        ${iwyu_path}
        -Xiwyu
        --mapping_file=${iwyu_share_path}/boost-all.imp
        -Xiwyu
        --mapping_file=${iwyu_share_path}/boost-all-private.imp
      )
      set_target_properties(
        ${PROJECT_NAME}
        PROPERTIES
        CXX_INCLUDE_WHAT_YOU_USE
        "${iwyu_path_and_options}"
      )
    endif()
  endif()
endif(NOT TARGET ${RamRodArduinoSerial_LIBRARIES})


