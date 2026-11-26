set(CMAKE_AUTOMOC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(qt_demo
    main.cpp
    MainWindow.h
    MainWindow.cpp
    CanvasWidget.h
    CanvasWidget.cpp
)

target_include_directories(qt_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/tests
)

target_link_libraries(qt_demo PRIVATE
    local_router
    Qt5::Widgets
)
