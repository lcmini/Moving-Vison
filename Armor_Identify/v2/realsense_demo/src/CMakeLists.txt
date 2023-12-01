SET(OpenCV_DIR /home/nano/Opencv/lib/cmake/opencv4)
#执行文件存入bin文件夹
SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


#src文件夹所有源文件，创建SRC_IST变量
AUX_SOURCE_DIRECTORY (. SRC_LIST)

#opencv
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES({OpenCV_INCLUDE_DIRS})

#头文件
INCLUDE_DIRECTORIES(../include)

#生成可执行文件
ADD_EXECUTABLE(demo ${SRC_LIST})

#链接opencv lib库
TARGET_LINK_LIBRARIES(demo ${OpenCV_LIBS})

