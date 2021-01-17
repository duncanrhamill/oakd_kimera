/**
 * @file main.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief OAK-D/Kimera Experiments main file.
 * 
 * @version 0.1
 * @date 2021-01-16
 * 
 * @copyright Copyright (c) Duncan R Hamill 2021
 */

/* -------------------------------------------------------------------------
 * INCLUDES
 * ------------------------------------------------------------------------- */

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <kimera-vio/pipeline/Pipeline.h>
#include <kimera-vio/frontend/Frame.h>
#include <kimera-vio/utils/Statistics.h>
#include <kimera-vio/utils/Timer.h>

#include "depthai/depthai.hpp"
#include "util.h"

/* -------------------------------------------------------------------------
 * CONSTANTS
 * ------------------------------------------------------------------------- */

// WLS parameters, taken from the OpenCV WLS filter docs recommended values.
#define WLS_LAMBDA (8000)
#define WLS_SIGMA (1.0)

// This defines a variable called FLAGS_params_folder_path, for some reason.
DEFINE_string(
    params_folder_path,
    "params/OAK-D",
    "Path to the folder containing the yaml files with the VIO parameters.");

/* -------------------------------------------------------------------------
 * STRUCTS
 * ------------------------------------------------------------------------- */

// A simple pose structure containing position vector and rotation matrix.
typedef struct _Pose {
    cv::Mat position;
    cv::Mat rotation;
} Pose;

/* -------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[]) {

    std::cout << "OAK-D/Kimera-VIO Experiment\n" << std::endl;

    // Init Google log and flags, which are used by Kimera
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // Parse VIO parameters from gflags.
    VIO::VioParams vio_params(FLAGS_params_folder_path);

    // A current issue (https://github.com/MIT-SPARK/Kimera-VIO/issues/48)
    // shows that using already rectified images is a little broken, we have to
    // patch the P and R_rectify values of each camera parameter to empty
    // matrices, because for some reason they're not set if we pass
    // images_rectified=true. 
    vio_params.camera_params_[0].P_ = cv::Mat::eye(3, 3, CV_32F);
    vio_params.camera_params_[0].R_rectify_ = cv::Mat::eye(3, 3, CV_32F);
    vio_params.camera_params_[1].P_ = cv::Mat::eye(3, 3, CV_32F);
    vio_params.camera_params_[1].R_rectify_ = cv::Mat::eye(3, 3, CV_32F);

    // Create the VIO pipeline
    VIO::Pipeline vio_pipeline(vio_params);

    // Create the pipeline that we're going to build. Pipelines are depthai's
    // way of chaining up different series or parallel process, sort of like
    // gstreamer. 
    //
    // Our pipeline is going to extract the left and right rectified images
    // from the cameras so we can pass these into the SLAM system, as well as
    // disparity maps for building a point cloud.
    dai::Pipeline pipeline;

    // We need to create all the nodes in our pipeline, which are:
    //  - the left and right monochrome (greyscale) stereo cameras of the OAK-D
    //  - a stereo depth node, which generates disparity maps and rectified
    //    images. The disparity map will be used in constructing the global 
    //    point cloud, and the rectified images for SLAM tracking.
    //  - output nodes, which allow us to get the rectified image data and
    //    disparity map to use outside the pipeline.
    auto mono_left = pipeline.create<dai::node::MonoCamera>();
    auto mono_right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xout_rectif_left = pipeline.create<dai::node::XLinkOut>();
    auto xout_rectif_right = pipeline.create<dai::node::XLinkOut>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();

    // And we set the names of each output node, so we can access them later as
    // output queues
    xout_rectif_left->setStreamName("rectified_left");
    xout_rectif_right->setStreamName("rectified_right");
    xout_disp->setStreamName("disparity");

    // Now we set which cameras are actually connected to the left and right
    // nodes, and set their resolution and framerate
    mono_left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    mono_left->setResolution(
        dai::MonoCameraProperties::SensorResolution::THE_720_P
    );
    mono_left->setFps(20.0);
    mono_right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    mono_right->setResolution(
        dai::MonoCameraProperties::SensorResolution::THE_720_P
    );
    mono_right->setFps(20.0);

    // Now we set the stereo node to output rectified images and disp maps. We
    // also set the rectify frames to not be mirrored, and to use black to fill
    // the edges of the rectified images. We need non-flipped images as we're
    // going to use them later down the line as input to the SLAM,
    // unfortunately this means our output disparity map will be flipped, so
    // we'll have to correct that later. We don't output depth as this would
    // disable the disparity map output.
    // 
    // We also enable extended disparity depth, which increases the maximum 
    // disparity and therefore provides a shorter minimum depth.
    stereo->setOutputRectified(true);
    stereo->setOutputDepth(false);
    stereo->setRectifyEdgeFillColor(0);
    stereo->setRectifyMirrorFrame(false);
    stereo->setExtendedDisparity(true);

    // We now link the cameras up to the stereo node
    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);

    // And the stereo rectified and disp outputs to the output nodes
    stereo->rectifiedLeft.link(xout_rectif_left->input);
    stereo->rectifiedRight.link(xout_rectif_right->input);
    stereo->disparity.link(xout_disp->input);

    // Now we can connect to the OAK-D device and start our pipeline
    dai::Device device(pipeline);
    device.startPipeline();

    // Finally to actually see the outputs we need to get their output queues
    // We use a max buffer size of 8 frames and set it into non-blocking mode.
    auto rectif_left_queue = device.getOutputQueue("rectified_left", 8, false);
    auto rectif_right_queue = device.getOutputQueue("rectified_right", 8, false);
    auto disp_queue = device.getOutputQueue("disparity", 8, false);
    
    // Create the WLS (weighted least squares) filter, which we use to improve
    // the quality of our disparity map. Also set the lambda and sigma values
    auto wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    wls_filter->setLambda(WLS_LAMBDA);
    wls_filter->setSigmaColor(WLS_SIGMA);

    // Formatter, for printing out matrices in a reasonable way.
    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);

    // We also want somewhere to store our pose data
    Pose pose;

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    auto slam_epoch = std::chrono::steady_clock::now();
    uint64_t frame_id = 0;

    // Now for the main loop
    while (1) {
        // Read the output frames from the OAK-D. These are blocking calls, so
        // they will wait until there's data available.
        auto rectif_left_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto rectif_right_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto disp_map_frame = disp_queue->get<dai::ImgFrame>();

        // Convert the frames into opencv images
        auto rectif_left = imgframe_to_mat(rectif_left_frame);
        auto rectif_right = imgframe_to_mat(rectif_right_frame);
        auto disp_map = imgframe_to_mat(disp_map_frame);

        // Create VIO frames, which are a bit more complicated than just an
        // image. The parameters here are:
        //  - a frame_id, which we just count sequentially, as they are used as
        //    indexes into GTSAM (so a timestamp _probably_ won't work)
        //  - a timestamp, which should be simple, but no. DepthAI and Kimera
        //    use different timestamp values, and Kimera doesn't say what their
        //    underlying int64_t represents. So we're going to go with
        //    DepthAI's number of nanoseconds + seconds as int64_t (~9bn
        //    seconds, should be fine). The timestamps of the left and right
        //    camera must match, so we just use the left camera timetsamps.
        //  - the camera parameters, which are stored in a vector in
        //    vio_params, and I'm assuming that left is 0 and right is 1.
        //  - finally, the actual image...
        //
        // Parse the frames into the pipeline. This is done using callbacks, so
        // we could create a fully callback based data source built on top of
        // the OAK-D, but for now we'll do it simply with standalone calls
        vio_pipeline.fillLeftFrameQueue(VIO::make_unique<VIO::Frame>(
            (VIO::FrameId)frame_id,
            depthai_ts_to_kimera_ts(rectif_left_frame->getTimestamp()),
            vio_params.camera_params_[0],
            rectif_left
        ));
        vio_pipeline.fillRightFrameQueue(VIO::make_unique<VIO::Frame>(
            (VIO::FrameId)frame_id,
            depthai_ts_to_kimera_ts(rectif_left_frame->getTimestamp()),
            vio_params.camera_params_[1],
            rectif_right
        ));

        
        vio_pipeline.fillSingleImuQueue(VIO::ImuMeasurement(
            depthai_ts_to_kimera_ts(rectif_left_frame->getTimestamp()),
            VIO::ImuAccGyr::Zero()
        ));

        // Spin the VIO pipeline. This performs a single calculation cycle of
        // the pipeline, effectively this is the "Run the SLAM" step.
        if (!vio_pipeline.spin()) {
            std::cout << "VIO Pipeline spin failed" << std::endl;
            return EXIT_FAILURE;
        }

        // std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

        // The output pose may be empty if the system was unable to track the
        // movement, so only get position and rotation if pose isn't empty. We
        // also put this info an a localisation fix available flag for later
        // use. 
        // bool loc_fix_available = !raw_pose.empty();
        // if (loc_fix_available) {
        //     // The pose matrix is a 4x4 extrinsic matrix, with the form:
        //     // [R_3x3 T_3x1; [0 0 0 1]], we can find the camera position with 
        //     // C = -R'T (R' = R transpose).
        //     pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
        //     cv::Mat T = raw_pose(cv::Rect(3, 0, 1, 3));
        //     pose.position = -pose.rotation.t()*T;

        //     // Print the updated position, but transpose it so that instead of
        //     // a column vector we have a row vector, which is easier to read.
        //     std::cout << 
        //         "position: " << 
        //         fmt->format(pose.position.t()) << 
        //         std::endl;
        // }
        // else {
        //     // If we didn't get a pose update log it.
        //     std::cout << "no pose update" << std::endl;
        // }

        // The raw disparity map is flipped, since we flipped the rectified
        // images, so we must flip it as well.
        cv::flip(disp_map, disp_map, 1);

        // Filter the disparity map
        cv::Mat filtered_disp_map;
        wls_filter->filter(disp_map, rectif_right, filtered_disp_map);

        // Apply a colormap to the filtered disparity map, but don't normalise
        // it. Normalising the map will mean that the color doesn't correspond
        // directly with disparity.
        cv::Mat colour_disp;
        cv::applyColorMap(filtered_disp_map, colour_disp, cv::COLORMAP_JET);
        cv::imshow("disparity", colour_disp);

        // Spin the VIO vizualizer, which displays the graphs
        // vio_pipeline.spinViz();

        // Increment the frame ID
        frame_id++;

        // See if q pressed, if so quit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Shutdown the VIO pipeline
    vio_pipeline.shutdown();

    return EXIT_SUCCESS;
}