cmake_minimum_required(VERSION 3.12)

file(GLOB_RECURSE SRC_FILES
	CONFIGURE_DEPENDS
	Src/*.cpp
	Src/*.c
	Src/*.hpp
	Src/*.h
)

add_library(DebugRendering SHARED
	${SRC_FILES}
)

target_include_directories(DebugRendering PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/Src
)

target_link_libraries(DebugRendering PRIVATE
	ZHMModSDK
)

install(TARGETS DebugRendering
	RUNTIME DESTINATION bin/mods
)