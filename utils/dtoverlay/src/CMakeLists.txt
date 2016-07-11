cmake_minimum_required(VERSION 2.6)

PROJECT(dtoverlay C)
ADD_DEFINITIONS(-Os -Wall --std=gnu99 -g3 -Ilibfdt -Idtovl)
IF (CMAKE_COMPILER_IS_GNUCC)
   ADD_DEFINITIONS (-ffunction-sections)
ENDIF ()

ADD_DEFINITIONS(-DFORCE_BOOT_DIR="/etc")

FILE(GLOB SOURCES dtovl/*.c libfdt/*.c)

ADD_EXECUTABLE(dtoverlay dtoverlay_main.c utils.c ${SOURCES})
ADD_EXECUTABLE(dtmerge dtmerge.c ${SOURCES})

SET(CMAKE_INSTALL_PREFIX /usr)

INSTALL(TARGETS dtoverlay dtmerge
	RUNTIME DESTINATION bin
)
