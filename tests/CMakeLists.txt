# Copyright © 2023 CCP ehf.

ccp_add_executable(DestinyTest)
find_package(GTest REQUIRED)
find_package(Tracy REQUIRED NO_CMAKE_PATH)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
target_link_libraries(DestinyTest PRIVATE destinyStatic GTest::GTest GTest::Main)

target_sources(DestinyTest PRIVATE
    destinyTest.cpp
    StdAfx.cpp
    TestAABB.cpp
    TestBoxShape.cpp
    TestCollision.cpp
    TestPartition.cpp
    TestPlane.cpp
    TestTriangle.cpp
    TestVector3d.cpp
)

target_precompile_headers(DestinyTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/StdAfx.h)
target_include_directories(DestinyTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(DestinyTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)

if(APPLE)
    target_link_libraries(DestinyTest PRIVATE "-framework CoreFoundation")
endif()

if (WIN32)
    # Copy shared library dependencies to folder containing test executable.
    add_custom_command(TARGET DestinyTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:Blue>
        $<TARGET_FILE_DIR:DestinyTest>
    )

    add_custom_command(TARGET DestinyTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:DestinyTest>
            $<TARGET_FILE_DIR:DestinyTest>
            COMMAND_EXPAND_LISTS
    )

    add_custom_command(TARGET DestinyTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:Tracy::TracyClient>
            $<TARGET_FILE_DIR:DestinyTest>
    )

    add_custom_command(TARGET DestinyTest POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:destiny>
            $<TARGET_FILE_DIR:DestinyTest>
    )
endif ()

set_target_properties(DestinyTest PROPERTIES FOLDER "Tests")
gtest_discover_tests(DestinyTest myListOfTests)

add_test(NAME PythonTests
        COMMAND ${EXEFILE_INTERPRETER} /inherit /buildflavor=$<LOWER_CASE:$<CONFIG>> /py -m unittest discover -v ${CMAKE_CURRENT_SOURCE_DIR}/../python/destiny/test
        COMMAND_EXPAND_LISTS
)
if (WIN32)
    set_tests_properties(PythonTests PROPERTIES ENVIRONMENT "PYTHONPATH=${PYTHONPATH}\;${CMAKE_CURRENT_SOURCE_DIR}/../python;BUILDFLAVOR=$<LOWER_CASE:$<CONFIG>>")
else ()
    set_tests_properties(PythonTests PROPERTIES ENVIRONMENT "PYTHONPATH=${PYTHONPATH}:${CMAKE_CURRENT_SOURCE_DIR}/../python;BUILDFLAVOR=$<LOWER_CASE:$<CONFIG>>")
endif ()
