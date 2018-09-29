# set project's name
PROJECT( AdaptiveIPM )

###############################################################################
# CMake settings
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

# OpenCV
#set(OpenCV_DIR "E:/3rdSoftWare/opencv/build")
FIND_PACKAGE(OpenCV REQUIRED)

FILE(GLOB_RECURSE HDRS_FILES "include/*.h" "include/*.hpp")
FILE(GLOB_RECURSE SRCS_FILES "src/*.c" "src/*.cpp")

ADD_EXECUTABLE(${PROJECT_NAME} ${SRCS_FILES} ${HDRS_FILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
