set(OpenCV_INCLUDE_DIRS "${opencv_vendor_DIR}/../../../include")
set(OpenCV_LIB_DIR "${opencv_vendor_DIR}/../../../lib")

set(OpenCV_LIBS
  opencv_core
  opencv_imgproc
  opencv_highgui
)

# Export variables for downstream packages
set(OpenCV_FOUND TRUE)
