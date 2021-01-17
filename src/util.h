/**
 * @file util.h
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief General utility header file
 * @version 0.1
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) Duncan Hamill 2021
 */

/* -------------------------------------------------------------------------
 * INCLUDES
 * ------------------------------------------------------------------------- */

#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

#include <kimera-vio/common/vio_types.h>

/* -------------------------------------------------------------------------
 * FUNCTIONS
 * ------------------------------------------------------------------------- */

/**
 * @brief Convert a depthai::ImageFrame to an OpenCV cv::Mat.
 * 
 * @param frame The frame to convert
 * @param data_type The format the ImgFrame is in.
 * @return cv::Mat Converted cv::Mat
 */
cv::Mat imgframe_to_mat(
    std::shared_ptr<dai::ImgFrame> frame, 
    int data_type=CV_8UC1
);

/**
 * @brief Convert a DepthAI::Timestamp into a Kimera Timestamp
 * 
 * @param ts_in 
 * @return VIO::Timestamp 
 */
VIO::Timestamp depthai_ts_to_kimera_ts(dai::Timestamp ts_in);