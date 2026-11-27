set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(QT_DEMO_SOURCES
    main.cc
    main_window.h
    main_window.cc
    canvas_widget.h
    canvas_widget.cc
    library_widget.h
    library_widget.cc
)

if(WIN32)
    add_executable(qt_demo WIN32 ${QT_DEMO_SOURCES})
    target_link_libraries(qt_demo PRIVATE local_router Qt5::Widgets Qt5::WinMain)
else()
    add_executable(qt_demo ${QT_DEMO_SOURCES})
    target_link_libraries(qt_demo PRIVATE local_router Qt5::Widgets)
endif()

target_include_directories(qt_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/tests
)
