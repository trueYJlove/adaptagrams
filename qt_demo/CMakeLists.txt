set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(qt_demo
    main.cc
    main_window.h
    main_window.cc
    canvas_widget.h
    canvas_widget.cc
)

target_include_directories(qt_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/tests
)

target_link_libraries(qt_demo PRIVATE
    local_router
    Qt5::Widgets
)
