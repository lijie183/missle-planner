# 声明要求的最低CMake版本
cmake_minimum_required(VERSION 3.16)

# 定义项目名称、版本和语言
project(MissilePlanner VERSION 1.0 LANGUAGES CXX)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 让CMake自动处理Qt的各种宏
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# =================你的 Qt 路径=================
set(CMAKE_PREFIX_PATH "C:/Qt/6.11.0/msvc2022_64" ${CMAKE_PREFIX_PATH})
# ===============================================

# 寻找我们在VCPKG中已经安装好的依赖包
find_package(Qt6 COMPONENTS Widgets REQUIRED)

# =================【关键修改点�?================
# 寻找OSG引擎合集（这样它会自动抓取所有绝对路径生成一个变量）
find_package(OpenSceneGraph REQUIRED osgDB osgViewer osgGA)
find_package(osgEarth REQUIRED)
# ===============================================

# 收集项目中所有的源码文件
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "main.cpp"
    "ui/*.cpp" "ui/*.h" "ui/*.ui"
    "render/*.cpp" "render/*.h"
    "core/*.cpp" "core/*.h"
    "sim/*.cpp" "sim/*.h"
)

# 创建可执行程序文�?
add_executable(${PROJECT_NAME} ${SOURCES})

if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# 让工程内�?"core/..."�?render/..." 形式的头文件可被所有源文件解析�?
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${osgEarth_INCLUDE_DIR}
)

# =================【关键修改点�?================
# 把各种库用变量名称链接到我们的主程序上，这样传过去的就是绝对路径了！
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Widgets                   # Qt界面�?
    ${OPENSCENEGRAPH_LIBRARIES}    # OSG所有的核心库（包含绝对路径�?
    osgEarth::osgEarth             # OSGEarth主库（包含导入目标属性）
)
# ===============================================

# 运行时同步 data 目录，避免输出目录使用陈旧 earth 配置
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_CURRENT_SOURCE_DIR}/data"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>/data"
)

# 运行时部署OSG插件，确保osgEarth地形引擎与数据驱动可被找�?
if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
    file(GLOB OSG_PLUGIN_DIRS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/plugins/osgPlugins-*")
    foreach(OSG_PLUGIN_DIR IN LISTS OSG_PLUGIN_DIRS)
        if(IS_DIRECTORY "${OSG_PLUGIN_DIR}")
            get_filename_component(OSG_PLUGIN_DIR_NAME "${OSG_PLUGIN_DIR}" NAME)
            add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                        "${OSG_PLUGIN_DIR}"
                        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/${OSG_PLUGIN_DIR_NAME}"
            )
        endif()
    endforeach()
endif()