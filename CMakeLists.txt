# Copyright © 2023 CCP ehf.

cmake_minimum_required(VERSION 3.16)
project(destiny VERSION 1.0.0)

find_package(carbon-blue CONFIG REQUIRED)
find_package(carbon-core CONFIG REQUIRED)
find_package(carbon-math CONFIG REQUIRED)

include(cmake/CcpTargetConfigurations.cmake)
include(cmake/CcpDocsGenerator.cmake)
include(cmake/CcpPackageConfigHelpers.cmake)
include(cmake/CcpBuildConfigurations.cmake)
include(cmake/CcpMonolithVendorConfig.cmake)

ccp_add_library(destiny MODULE)
target_link_libraries(destiny PUBLIC Blue CcpCore PRIVATE CcpMath)

set(PUBLIC_HEADER_FILES_INCLUDE
    include/IEveReferencePoint.h
    include/IEveBallpark.h
    include/IEveObstructionQuery.h
)

set(_SOURCES
    src/AABB.cpp
    src/AABB.h
    src/autoversion.h
    src/Ball.cpp
    src/Ball.h
    src/Ball_Blue.cpp
    src/Ballpark.cpp
    src/Ballpark.h
    src/Ballpark_Blue.cpp
    src/Box.cpp
    src/Box.h
    src/BoxShape.cpp
    src/BoxShape.h
    src/Capsule.cpp
    src/Capsule.h
    src/Capsule_Blue.cpp
    src/Collision.cpp
    src/Collision.h
    src/CollisionStructures.cpp
    src/CollisionStructures.h
    src/CollisionBallProperties.cpp
    src/CollisionBallProperties.h
    src/destiny.cpp
    src/DstConstants.h
    src/MapOfBalls.h
    src/MapOfLongLongs.h
    src/MiniBall.cpp
    src/MiniBall.h
    src/Miniball_Blue.cpp
    src/MiniBox.cpp
    src/MiniBox.h
    src/MiniBox_Blue.cpp
    src/MiniCapsule.cpp
    src/MiniCapsule.h
    src/MiniCapsule_Blue.cpp
    src/OrientedBox.cpp
    src/OrientedBox.h
    src/OrientedBox_Blue.cpp
    src/Partition.cpp
    src/Partition.h
    src/Partition_Blue.cpp
    src/Partitionable.cpp
    src/Partitionable.h
    src/Plane.cpp
    src/Plane.h
    src/Random.cpp
    src/Random.h
    src/resource.h
    src/Settings.h
    src/Settings.cpp
    src/Settings_Blue.cpp
    src/SortedSets.h
    src/StaticCollidable.cpp
    src/StaticCollidable.h
    src/StdAfx.cpp
    src/Thunkers.cpp
    src/Triangle.cpp
    src/Triangle.h
    src/Vector3d.h
    src/version.h
    ${PUBLIC_HEADER_FILES_INCLUDE}
)
target_sources(destiny PRIVATE ${_SOURCES})

target_precompile_headers(destiny PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/StdAfx.h)
target_include_directories(destiny
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(APPLE)
    # this is temporary so that we don't drown in warnings
    set_property(TARGET destiny APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-nonportable-include-path -Wno-inconsistent-missing-override")
endif()

set_target_properties(destiny PROPERTIES OUTPUT_NAME "_destiny")

ccp_add_library(destinyStatic STATIC)
target_link_libraries(destinyStatic PUBLIC Blue CcpCore CcpMath)
target_sources(destinyStatic PRIVATE ${_SOURCES})
target_precompile_headers(destinyStatic PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/StdAfx.h)
target_include_directories(destinyStatic
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(APPLE)
    # this is temporary so that we don't drown in warnings
    set_property(TARGET destinyStatic APPEND_STRING PROPERTY COMPILE_FLAGS " -Wno-nonportable-include-path -Wno-inconsistent-missing-override")
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    option(BUILD_TESTING "Build and run tests. Enabled by default." ON)
    include(CTest)
    if(BUILD_TESTING)
        find_package(carbon-exefile-interpreter REQUIRED CONFIG)
        # Interpreter must be created in this file, since it creates a POST_BUILD target on destiny.
        ccp_prepare_interpreter(
            TARGET destiny
            BUILD_TYPE ${CMAKE_BUILD_TYPE}
            COMMAND_OUT EXEFILE_INTERPRETER
            PYTHONPATH_OUT PYTHONPATH
            RUN_DIR $<TARGET_FILE_DIR:destiny>
            BLUE_DIR_OUT BLUE_LOCATION_DIR
        )
	    add_subdirectory(tests)
    endif()
endif()

# Provide an install target iff this is the top level project only
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    option(INSTALL_TO_MONOLITH "Install this package to a perforce branch vendor folder - provided for backwards compatability" OFF)
    if(INSTALL_TO_MONOLITH)
        # Install rule to ensure that our runtime and linker files are in the expected, platform-specific folders
        install(
            TARGETS destiny
            CONFIGURATIONS ${CMAKE_CONFIGURATION_TYPES}
            ARCHIVE DESTINATION lib/${CCP_PLATFORM}/${CCP_ARCHITECTURE}/${CCP_TOOLSET}/
            LIBRARY DESTINATION lib/${CCP_PLATFORM}/${CCP_ARCHITECTURE}/${CCP_TOOLSET}/
            RUNTIME DESTINATION bin/${CCP_PLATFORM}/${CCP_ARCHITECTURE}/${CCP_TOOLSET}/
        )
        # Install rule for available public headers
        install(FILES ${PUBLIC_HEADER_FILES_INCLUDE} DESTINATION include)

        # Install a custom DestinyConfig.cmake file, since as a dependency we are
        # basically a header-only library, not something that gets linked against.
        install(FILES ${PROJECT_SOURCE_DIR}/cmake/DestinyConfig.cmake DESTINATION .)

        # Install python package
        install(DIRECTORY python/ DESTINATION python
                FILES_MATCHING
                PATTERN "*.py"
                PATTERN "package.yaml"
        )
    else()
        # Install rule to ensure that our runtime and linker files are in the expected, platform-specific folders
        install(
                TARGETS destiny
                EXPORT DestinyTargets
                CONFIGURATIONS ${CMAKE_CONFIGURATION_TYPES}
                ARCHIVE DESTINATION lib/
                LIBRARY DESTINATION lib/
                RUNTIME DESTINATION bin/
        )

        install(EXPORT DestinyTargets
                DESTINATION share/carbon-destiny
                FILE carbon-destiny.cmake
        )

        install(FILES carbon-destinyConfig.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/share/carbon-destiny)

        # Install rule for available public headers
        install(FILES ${PUBLIC_HEADER_FILES_INCLUDE} DESTINATION include)

        # Install python package
        install(DIRECTORY python/destiny DESTINATION bin/python
                FILES_MATCHING
                PATTERN "*.py"
                PATTERN "package.yaml"
        )
    endif()
endif()
