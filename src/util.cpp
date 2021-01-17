/**
 * @file util.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief General utilities for the experiments.
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

cv::Mat imgframe_to_mat(
    std::shared_ptr<dai::ImgFrame> frame, 
    int data_type=CV_8UC1
) {
    return cv::Mat(
        frame->getHeight(), 
        frame->getWidth(), 
        data_type, 
        frame->getData().data()
    );
}

VIO::Timestamp depthai_ts_to_kimera_ts(dai::Timestamp ts_in) {
    return (VIO::Timestamp)(
        ((int64_t)ts_in.sec * 1000000000) + (int64_t)ts_in.nsec
    );
}
