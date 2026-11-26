cmake_minimum_required(VERSION 3.16)
project(local_orthogonal_router LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LOCALROUTE_BUILD_TESTS "Build unit tests and benchmarks" ON)
option(LOCALROUTE_BUILD_QT_DEMO "Build Qt interactive demo" ON)

find_package(Threads REQUIRED)

add_library(local_router STATIC
    src/lazy_hanan_a_star_solver.cc
    src/local_orthogonal_router.cc
    src/channel_nudger.cc
    src/async_local_router.cc
)
target_include_directories(local_router PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_router PUBLIC Threads::Threads)

if(LOCALROUTE_BUILD_TESTS)
    enable_testing()
    add_executable(test_unit tests/local_router_test.cc)
    target_link_libraries(test_unit PRIVATE local_router)
    foreach(t
        SpatialGridBasic
        DirtyOnlyNoRerouteOnOverlap
        LocalObstacleAvoidance
        AdaptiveWindowExpansion
        OverlappingBlocksFallback
        ParallelColdStartDeterminism
        FastReuseAndThirdPartyObstruction
        AsyncDiscardsStaleResults
        LazyHananAStarBoundaryCases
        ChannelNudgingParallelBundle
        ChannelNudgingCascadeAndNoHook
        NarrowGapObstacleAvoidance
        SolderJointDetection
        ManualSegmentDragAndPreserve
        AutoRouteSelectedLines
        WireCongestionSoftPenalty
    )
        add_test(NAME local.${t} COMMAND test_unit ${t})
    endforeach()

    add_executable(bench_large_scale tests/bench_large_scale.cc)
    target_link_libraries(bench_large_scale PRIVATE local_router)
endif()

if(LOCALROUTE_BUILD_QT_DEMO AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/qt_demo/CMakeLists.txt")
    add_subdirectory(qt_demo)
endif()
