cmake_minimum_required(VERSION 2.6)

PROJECT(dtoverlay C)
ADD_DEFINITIONS(-Os -Wall --std=gnu99 -g3 -Ilibfdt -Idtovl -Ilibdtc)
IF (CMAKE_COMPILER_IS_GNUCC)
   ADD_DEFINITIONS (-ffunction-sections)
ENDIF ()

ADD_DEFINITIONS(-DFORCE_BOOT_DIR="/etc")

FILE(GLOB LIBFDT libfdt/*.c)
FILE(GLOB LIBDTOVL dtovl/*.c)
FILE(GLOB DTCSRC libdtc/*.c)

ADD_EXECUTABLE(dtoverlay dtoverlay_main.c utils.c ${LIBDTOVL} ${LIBFDT})
ADD_EXECUTABLE(dtmerge dtmerge.c ${LIBDTOVL} ${LIBFDT})
ADD_EXECUTABLE(dtc ${DTCSRC} ${LIBFDT})

SET(CMAKE_INSTALL_PREFIX /usr)

INSTALL(TARGETS dtoverlay dtmerge dtc
	RUNTIME DESTINATION bin
)
