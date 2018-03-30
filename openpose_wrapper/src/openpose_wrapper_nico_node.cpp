// Synchronous mode: ideal for performance. The user can add his own frames producer / post-processor / consumer to the OpenPose wrapper or use the
// default ones.

// This example shows the user how to use the OpenPose wrapper class:
    // 1. User reads images
    // 2. Extract and render keypoint / heatmap / PAF of that image
    // 3. Save the results on disk
    // 4. User displays the rendered pose
    // Everything in a multi-thread scenario
// In addition to the previous OpenPose modules, we also need to use:
    // 1. `core` module:
        // For the Array<float> class that the `pose` module needs
        // For the Datum struct that the `thread` module sends between the queues
    // 2. `utilities` module: for the error & logging functions, i.e. op::error & op::log respectively
// This file should only be used for the user to take specific examples.

// C++ std library dependencies
#include <iostream>
#include <chrono> // `std::chrono::` functions and classes, e.g. std::chrono::milliseconds
#include <thread> // std::this_thread
#include <unistd.h>
#include <string>
// Other 3rdparty dependencies
// GFlags: DEFINE_bool, _int32, _int64, _uint64, _double, _string
#include <gflags/gflags.h>
// Allow Google Flags in Ubuntu 14
#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif
// OpenPose dependencies
#include <openpose/headers.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pthread.h>
#include <array>
#include <openpose_wrapper/OpenPose.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <openpose_ros_wrapper_msgs/Persons.h>
#include <openpose_ros_wrapper_msgs/Persons3d.h>
#include <openpose_ros_wrapper_msgs/BodyPartDetection.h>
#include <openpose_ros_wrapper_msgs/BodyPartDetection3d.h>
#include <openpose_ros_wrapper_msgs/EyeDetection.h>
#include <openpose_ros_wrapper_msgs/PersonDetection3d.h>
#include <openpose_ros_wrapper_msgs/PersonDetection.h>
#include <image_transport/image_transport.h>
#include <openpose_ros_msgs/GetPersons.h>
#include <openpose_ros_msgs/BodypartDetection.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480

//#include <openpose_ros_msgs/.h>
static std_msgs::Header h;
static std_msgs::Header h_min;
static std_msgs::Header h_temp;
int a;
std::vector <pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
std::vector <std_msgs::Header> headers;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImage out_msg;
openpose_ros_wrapper_msgs::EyeDetection eye_msg;

//added from mk:ToDO
op::CvMatToOpInput *cvMatToOpInput;
op::CvMatToOpOutput *cvMatToOpOutput;
op::PoseExtractorCaffe *poseExtractorCaffe;
op::PoseRenderer *poseRenderer;
op::FaceDetector *faceDetector;
op::FaceExtractor *faceExtractor;
op::FaceRenderer *faceRenderer;
op::OpOutputToCvMat *opOutputToCvMat;

openpose_ros_msgs::BodypartDetection getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.confidence = NAN;
  return bodypart;
}

openpose_ros_msgs::BodypartDetection getBodyPartDetectionFromArrayAndIndex(const op::Array<float>& array, size_t idx)
{
  openpose_ros_msgs::BodypartDetection bodypart;

  bodypart.x = array[idx];
  bodypart.y = array[idx+1];
  bodypart.confidence = array[idx+2];
  return bodypart;
}

//std::map<unsigned int, std::string> getBodyPartMapFromPoseModel(const op::PoseModel& pose_model)
//{
  //if (pose_model == op::PoseModel::COCO_18)
  //{
    //return op::POSE_COCO_BODY_PARTS;
  //}
  //else if (pose_model == op::PoseModel::MPI_15 || pose_model == op::PoseModel::MPI_15_4)
  //{
    //return op::POSE_MPI_BODY_PARTS;
  //}
  //else
  //{
    //ROS_FATAL("Invalid pose model, not map present");
    //exit(1);
  //}
//}




class MyPublisher
{
    public:
    MyPublisher(void);
    ros::Publisher publisher;
    ros::Publisher image_skeleton_pub;
    ros::Publisher pose_pub;
    ros::Publisher pose_3d_pub;
    ros::Publisher keypoints_pub;
    ros::Publisher eye_pub;
    ros::Subscriber pcl_sub;

    openpose_ros_wrapper_msgs::Persons persons;
    openpose_ros_wrapper_msgs::Persons3d persons_3d;
    //point_cloud   
   
    sensor_msgs::PointCloud2 pCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; 

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

    void callback(const op::Array<float>&);

    std_msgs::Float32MultiArray msg;
    std::map<unsigned int, std::string> bodypartsmap;
};

pthread_mutex_t buf_mutex = PTHREAD_MUTEX_INITIALIZER;
bool buff_empty = true;
MyPublisher mp;

using namespace std;

// See all the available parameter options withe the `--help` flag. E.g. `build/examples/openpose/openpose.bin --help`
// Note: This command will show you flags for other unnecessary 3rdparty files. Check only the flags for the OpenPose
// executable. E.g. for `openpose.bin`, look for `Flags from examples/openpose/openpose.cpp:`.
// Debugging/Other
DEFINE_int32(logging_level,             3,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
DEFINE_bool(disable_multi_thread,       false,          "It would slightly reduce the frame rate in order to highly reduce the lag. Mainly useful"
                                                        " for 1) Cases where it is needed a low latency (e.g. webcam in real-time scenarios with"
                                                        " low-range GPU devices); and 2) Debugging OpenPose when it is crashing to locate the"
                                                        " error.");
DEFINE_int32(profile_speed,             1000,           "If PROFILER_ENABLED was set in CMake or Makefile.config files, OpenPose will show some"
                                                        " runtime statistics at this frame number.");
// Producer
DEFINE_string(image_dir,                "/hsrb/head_rgbd_sensor/rgb/image_raw",      "Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).");
// OpenPose
DEFINE_string(model_folder,             "/usr/share/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(output_resolution,        "-1x-1",        "The image resolution (display and output). Use \"-1x-1\" to force the program to use the"
                                                        " input image resolution.");
DEFINE_int32(num_gpu,                   -1,             "The number of GPU devices to use. If negative, it will use all the available GPUs in your"
                                                        " machine.");
DEFINE_int32(num_gpu_start,             0,              "GPU device start number.");
DEFINE_int32(keypoint_scale,            0,              "Scaling of the (x,y) coordinates of the final pose data array, i.e. the scale of the (x,y)"
                                                        " coordinates that will be saved with the `write_keypoint` & `write_keypoint_json` flags."
                                                        " Select `0` to scale it to the original source resolution, `1`to scale it to the net output"
                                                        " size (set with `net_resolution`), `2` to scale it to the final output size (set with"
                                                        " `resolution`), `3` to scale it in the range [0,1], and 4 for range [-1,1]. Non related"
                                                        " with `scale_number` and `scale_gap`.");
// OpenPose Body Pose
DEFINE_bool(body_disable,               false,          "Disable body keypoint detection. Option only possible for faster (but less accurate) face"
                                                        " keypoint detection.");
DEFINE_string(model_pose,               "COCO",         "Model to be used. E.g. `COCO` (18 keypoints), `MPI` (15 keypoints, ~10% faster), "
                                                        "`MPI_4_layers` (15 keypoints, even faster but less accurate).");
DEFINE_string(net_resolution,           "-1x368",       "Multiples of 16. If it is increased, the accuracy potentially increases. If it is"
                                                        " decreased, the speed increases. For maximum speed-accuracy balance, it should keep the"
                                                        " closest aspect ratio possible to the images or videos to be processed. Using `-1` in"
                                                        " any of the dimensions, OP will choose the optimal aspect ratio depending on the user's"
                                                        " input value. E.g. the default `-1x368` is equivalent to `656x368` in 16:9 resolutions,"
                                                        " e.g. full HD (1980x1080) and HD (1280x720) resolutions.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
// OpenPose Body Pose Heatmaps and Part Candidates
DEFINE_bool(heatmaps_add_parts,         false,          "If true, it will fill op::Datum::poseHeatMaps array with the body part heatmaps, and"
                                                        " analogously face & hand heatmaps to op::Datum::faceHeatMaps & op::Datum::handHeatMaps."
                                                        " If more than one `add_heatmaps_X` flag is enabled, it will place then in sequential"
                                                        " memory order: body parts + bkg + PAFs. It will follow the order on"
                                                        " POSE_BODY_PART_MAPPING in `src/openpose/pose/poseParameters.cpp`. Program speed will"
                                                        " considerably decrease. Not required for OpenPose, enable it only if you intend to"
                                                        " explicitly use this information later.");
DEFINE_bool(heatmaps_add_bkg,           false,          "Same functionality as `add_heatmaps_parts`, but adding the heatmap corresponding to"
                                                        " background.");
DEFINE_bool(heatmaps_add_PAFs,          false,          "Same functionality as `add_heatmaps_parts`, but adding the PAFs.");
DEFINE_int32(heatmaps_scale,            2,              "Set 0 to scale op::Datum::poseHeatMaps in the range [-1,1], 1 for [0,1]; 2 for integer"
                                                        " rounded [0,255]; and 3 for no scaling.");
DEFINE_bool(part_candidates,            false,          "Also enable `write_json` in order to save this information. If true, it will fill the"
                                                        " op::Datum::poseCandidates array with the body part candidates. Candidates refer to all"
                                                        " the detected body parts, before being assembled into people. Note that the number of"
                                                        " candidates is equal or higher than the number of final body parts (i.e. after being"
                                                        " assembled into people). The empty body parts are filled with 0s. Program speed will"
                                                        " slightly decrease. Not required for OpenPose, enable it only if you intend to explicitly"
                                                        " use this information.");
// OpenPose Face
DEFINE_bool(face,                       true,          "Enables face keypoint detection. It will share some parameters from the body pose, e.g."
                                                        " `model_folder`. Note that this will considerable slow down the performance and increse"
                                                        " the required GPU memory. In addition, the greater number of people on the image, the"
                                                        " slower OpenPose will be.");
DEFINE_string(face_net_resolution,      "368x368",      "Multiples of 16 and squared. Analogous to `net_resolution` but applied to the face keypoint"
                                                        " detector. 320x320 usually works fine while giving a substantial speed up when multiple"
                                                        " faces on the image.");
// OpenPose Hand
DEFINE_bool(hand,                       false,          "Enables hand keypoint detection. It will share some parameters from the body pose, e.g."
                                                        " `model_folder`. Analogously to `--face`, it will also slow down the performance, increase"
                                                        " the required GPU memory and its speed depends on the number of people.");
DEFINE_string(hand_net_resolution,      "368x368",      "Multiples of 16 and squared. Analogous to `net_resolution` but applied to the hand keypoint"
                                                        " detector.");
DEFINE_int32(hand_scale_number,         1,              "Analogous to `scale_number` but applied to the hand keypoint detector. Our best results"
                                                        " were found with `hand_scale_number` = 6 and `hand_scale_range` = 0.4");
DEFINE_double(hand_scale_range,         0.4,            "Analogous purpose than `scale_gap` but applied to the hand keypoint detector. Total range"
                                                        " between smallest and biggest scale. The scales will be centered in ratio 1. E.g. if"
                                                        " scaleRange = 0.4 and scalesNumber = 2, then there will be 2 scales, 0.8 and 1.2.");
DEFINE_bool(hand_tracking,              false,          "Adding hand tracking might improve hand keypoints detection for webcam (if the frame rate"
                                                        " is high enough, i.e. >7 FPS per GPU) and video. This is not person ID tracking, it"
                                                        " simply looks for hands in positions at which hands were located in previous frames, but"
                                                        " it does not guarantee the same person ID among frames");
// OpenPose Rendering
DEFINE_int32(part_to_show,              0,              "Prediction channel to visualize (default: 0). 0 for all the body parts, 1-18 for each body"
                                                        " part heat map, 19 for the background heat map, 20 for all the body part heat maps"
                                                        " together, 21 for all the PAFs, 22-40 for each body part pair PAF");
DEFINE_bool(disable_blending,           false,          "If enabled, it will render the results (keypoint skeletons or heatmaps) on a black"
                                                        " background, instead of being rendered into the original image. Related: `part_to_show`,"
                                                        " `alpha_pose`, and `alpha_pose`.");
// OpenPose Rendering Pose
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_int32(render_pose,              -1,             "Set to 0 for no rendering, 1 for CPU rendering (slightly faster), and 2 for GPU rendering"
                                                        " (slower but greater functionality, e.g. `alpha_X` flags). If -1, it will pick CPU if"
                                                        " CPU_ONLY is enabled, or GPU if CUDA is enabled. If rendering is enabled, it will render"
                                                        " both `outputData` and `cvOutputData` with the original image and desired body part to be"
                                                        " shown (i.e. keypoints, heat maps or PAFs).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");
DEFINE_double(alpha_heatmap,            0.7,            "Blending factor (range 0-1) between heatmap and original frame. 1 will only show the"
                                                        " heatmap, 0 will only show the frame. Only valid for GPU rendering.");
// OpenPose Rendering Face
DEFINE_double(face_render_threshold,    0.4,            "Analogous to `render_threshold`, but applied to the face keypoints.");
DEFINE_int32(face_render,               -1,             "Analogous to `render_pose` but applied to the face. Extra option: -1 to use the same"
                                                        " configuration that `render_pose` is using.");
DEFINE_double(face_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to face.");
DEFINE_double(face_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to face.");
// OpenPose Rendering Hand
DEFINE_double(hand_render_threshold,    0.2,            "Analogous to `render_threshold`, but applied to the hand keypoints.");
DEFINE_int32(hand_render,               -1,             "Analogous to `render_pose` but applied to the hand. Extra option: -1 to use the same"
                                                        " configuration that `render_pose` is using.");
DEFINE_double(hand_alpha_pose,          0.6,            "Analogous to `alpha_pose` but applied to hand.");
DEFINE_double(hand_alpha_heatmap,       0.7,            "Analogous to `alpha_heatmap` but applied to hand.");
// Result Saving
DEFINE_string(write_images,             "",             "Directory to write rendered frames in `write_images_format` image format.");
DEFINE_string(write_images_format,      "png",          "File extension and format for `write_images`, e.g. png, jpg or bmp. Check the OpenCV"
                                                        " function cv::imwrite for all compatible extensions.");
DEFINE_string(write_video,              "",             "Full file path to write rendered frames in motion JPEG video format. It might fail if the"
                                                        " final path does not finish in `.avi`. It internally uses cv::VideoWriter.");
DEFINE_string(write_json,               "",             "Directory to write OpenPose output in JSON format. It includes body, hand, and face pose"
                                                        " keypoints, as well as pose candidates (if `--part_candidates` enabled).");
DEFINE_string(write_coco_json,          "",             "Full file path to write people pose data with JSON COCO validation format.");
DEFINE_string(write_heatmaps,           "",             "Directory to write body pose heatmaps in PNG format. At least 1 `add_heatmaps_X` flag"
                                                        " must be enabled.");
DEFINE_string(write_heatmaps_format,    "png",          "File extension and format for `write_heatmaps`, analogous to `write_images_format`."
                                                        " For lossless compression, recommended `png` for integer `heatmaps_scale` and `float` for"
                                                        " floating values.");
DEFINE_string(write_keypoint,           "",             "(Deprecated, use `write_json`) Directory to write the people pose keypoint data. Set format"
                                                        " with `write_keypoint_format`.");
DEFINE_string(write_keypoint_format,    "yml",          "(Deprecated, use `write_json`) File extension and format for `write_keypoint`: json, xml,"
                                                        " yaml & yml. Json not available for OpenCV < 3.0, use `write_keypoint_json` instead.");
DEFINE_string(write_keypoint_json,      "",             "(Deprecated, use `write_json`) Directory to write people pose data in JSON format,"
                                                        " compatible with any OpenCV version.");


// If the user needs his own variables, he can inherit the op::Datum struct and add them
// UserDatum can be directly used by the OpenPose wrapper because it inherits from op::Datum, just define
// Wrapper<UserDatum> instead of Wrapper<op::Datum>
struct UserDatum : public op::Datum
{
    bool boolThatUserNeedsForSomeReason;
    int index;

    UserDatum(const bool boolThatUserNeedsForSomeReason_ = false) :
        boolThatUserNeedsForSomeReason{boolThatUserNeedsForSomeReason_}
    {}
};

// The W-classes can be implemented either as a template or as simple classes given
// that the user usually knows which kind of data he will move between the queues,
// in this case we assume a std::shared_ptr of a std::vector of UserDatum

// This worker will just read and return all the jpg files in a directory
class WUserInput : public op::WorkerProducer<std::shared_ptr<std::vector<UserDatum>>>
{
public:
    WUserInput(const std::string& directoryPath) :
        mCounter{0}
    {
    }

    void initializationOnThread() {}

    std::shared_ptr<std::vector<UserDatum>> workProducer()
    {

        pthread_mutex_lock(&buf_mutex);
        if ( !buff_empty )
        {
            pthread_mutex_unlock(&buf_mutex);
            return nullptr;
        }
        pthread_mutex_unlock(&buf_mutex);

        ros::spinOnce();

        try
        {
            // Close program when empty frame
            if ( cv_ptr == nullptr )
            {
//                op::log("Last frame read and added to queue. Closing program after it is processed.",
//                        op::Priority::High);
                // This funtion stops this worker, which will eventually stop the whole thread system once all the
                // frames have been processed
//                this->stop();
                sleep(1);
                return nullptr;
            }
            else
            {
                // Create new datum
                auto datumsPtr = std::make_shared<std::vector<UserDatum>>();
                datumsPtr->emplace_back();
                auto& datum = datumsPtr->at(0);

                // Fill datum
                datum.index = mCounter++;

                datum.cvInputData = cv_ptr->image; 
               
          //float cur_time_stamp = cv_ptr->header.stamp;
            ROS_INFO("h stamp is : %.3lf" , h.stamp);
            std::cout<<h.stamp<< " is the stamp of h seee !!!!!!!"<<std::endl;
            ros::Duration d(0.25);
            for (int it = 0 ; it < headers.size(); ++it)
             {std::cout<<it<< " eme stamp value in the headers vector : "<< headers[it].stamp<<std::endl;
        
             if(headers[it].stamp - h.stamp < d)
               {
                 cloud_temp = clouds[it];
                 h_temp.stamp = ros::Time::now();
                 break;
               }
             }
 

                // If empty frame -> return nullptr
                if (datum.cvInputData.empty())
                {
//                    op::log("Empty frame detected on path: " + mImageFiles.at(mCounter-1) + ". Closing program.",
//                       op::Priority::High);
//                    this->stop();
                    datumsPtr = nullptr;
                }

                pthread_mutex_lock(&buf_mutex);
                buff_empty = false;
                pthread_mutex_unlock(&buf_mutex);

                return datumsPtr;
            }
        }
        catch (const std::exception& e)
        {
            op::log("Some kind of unexpected error happened.");
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return nullptr;
        }
    }

private:
    const std::vector<std::string> mImageFiles;
    unsigned long long mCounter;
};

// This worker will just invert the image
class WUserPostProcessing : public op::Worker<std::shared_ptr<std::vector<UserDatum>>>
{
public:
    WUserPostProcessing()
    {
        // User's constructor here
    }

    void initializationOnThread() {}

    void work(std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
    {
        // User's post-processing (after OpenPose processing & before OpenPose outputs) here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
#if 0
        try
        {
            if (datumsPtr != nullptr && !datumsPtr->empty())
                for (auto& datum : *datumsPtr)
                    cv::bitwise_not(datum.cvOutputData, datum.cvOutputData);
        }
        catch (const std::exception& e)
        {
            op::log("Some kind of unexpected error happened.");
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
#endif
    }
};

int _count = 0;
// This worker will just read and return all the jpg files in a directory
class WUserOutput : public op::WorkerConsumer<std::shared_ptr<std::vector<UserDatum>>>
{
public:
    void initializationOnThread() {}

    void workConsumer(const std::shared_ptr<std::vector<UserDatum>>& datumsPtr)
    {
        pthread_mutex_lock(&buf_mutex);
        if ( buff_empty )
        {
            pthread_mutex_unlock(&buf_mutex);
            return;
        }
        pthread_mutex_unlock(&buf_mutex);
        try
        {
            // User's displaying/saving/other processing here
                // datum.cvOutputData: rendered frame with pose or heatmaps
                // datum.poseKeypoints: Array<float> with the estimated pose
            if (datumsPtr != nullptr && !datumsPtr->empty())
            {
                // Show in command line the resulting pose keypoints for body, face and hands
                //op::log("\nKeypoints for " + std::to_string(datumsPtr->at(0).index) + " :");
                // Accesing each element of the keypoints
                const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
                 op::log("Person pose keypoints:");
                 for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
                 {
                     op::log("Person " + std::to_string(person) + " (x, y, score):");
                     for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                     {
                         std::string valueToPrint;
                         for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                         {
                             valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                         }
                         op::log(valueToPrint);
                     }
                 }
                //op::log(" ");
                // Alternative: just getting std::string equivalent
                //op::log("Face keypoints: " + datumsPtr->at(0).faceKeypoints.toString());

                const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
                //op::log("Person face keypoints:");

                for (auto face = 0 ; face < faceKeypoints.getSize(0) ; face++)
                {
                    //op::log("face " + std::to_string(face) + " (x, y, score):");
                    for (auto bodyPart = 0 ; bodyPart < faceKeypoints.getSize(1) ; bodyPart++)
                    {

                        if((bodyPart >35 ) && (bodyPart <48)){
                            std::string valueToPrint;
                            for (auto xyscore = 0 ; xyscore < faceKeypoints.getSize(2) ; xyscore++)
                            {
                                valueToPrint += std::to_string(faceKeypoints[{face, bodyPart, xyscore}]   ) + " ";
                            }
                            //op::log(valueToPrint);
                        }
                        else if(bodyPart ==68) //left eye
                        {
                            float confidence_left = faceKeypoints[{face, bodyPart, 2}];
                            //std::string valueToPrint2;
                            //valueToPrint2 = "left eye confidence: ";
                            //valueToPrint2 += std::to_string(faceKeypoints[{face, bodyPart, 2}]) + " ";
                            //op::log(valueToPrint2);
                            eye_msg.confidence_left = confidence_left;
                            if(confidence_left>0.7)
                                eye_msg.left_eye=true;
                            else
                                eye_msg.left_eye=false;
                        }
                        else if(bodyPart ==69) //right eye
                        {
                            //std::string valueToPrint2;
                            //valueToPrint2 = "right eye confidence: ";
                            //valueToPrint2 += std::to_string(faceKeypoints[{face, bodyPart, 2}]) + " ";
                            //op::log(valueToPrint2);
                            float confidence_right= faceKeypoints[{face, bodyPart, 2}];
                            eye_msg.confidence_right= confidence_right;
                            if(confidence_right>0.7)
                                eye_msg.right_eye=true;
                            else
                                eye_msg.right_eye=false;

                        }
                        else{
                        
                        }
                    }
                }
                //op::log(" ");

                mp.callback(poseKeypoints);
                pthread_mutex_lock(&buf_mutex);
                buff_empty = true;
                pthread_mutex_unlock(&buf_mutex);

                // Display rendered output image
                //cv::imshow("User worker GUI", datumsPtr->at(2).cvOutputData);
                out_msg.image=datumsPtr->at(0).cvOutputData;
                // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
                const char key = (char)cv::waitKey(1);
                _count++;
                if (key == 27)
//              sleep(5);
                    this->stop();
            }
        }
        catch (const std::exception& e)
        {
            op::log("Some kind of unexpected error happened.");
            this->stop();
            op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
};

int openPoseTutorialWrapper2()
{
    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    op::log("Starting pose estimation demo.", op::Priority::High);
    const auto timerBegin = std::chrono::high_resolution_clock::now();

    // Applying user defined configuration - Google flags to program variables
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
    // faceNetInputSize
    const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
    // handNetInputSize
    const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // JSON saving
    const auto writeJson = (!FLAGS_write_json.empty() ? FLAGS_write_json : FLAGS_write_keypoint_json);
    if (!FLAGS_write_keypoint.empty() || !FLAGS_write_keypoint_json.empty())
        op::log("Flags `write_keypoint` and `write_keypoint_json` are deprecated and will eventually be removed."
                " Please, use `write_json` instead.", op::Priority::Max);
    // keypointScale
    const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    // Enabling Google Logging
    const bool enableGoogleLogging = true;
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    // Initializing the user custom classes
    // Frames producer (e.g. video, webcam, ...)
    auto wUserInput = std::make_shared<WUserInput>(FLAGS_image_dir);
    // Processing
    auto wUserPostProcessing = std::make_shared<WUserPostProcessing>();
    // GUI (Display)
    auto wUserOutput = std::make_shared<WUserOutput>();

    op::Wrapper<std::vector<UserDatum>> opWrapper;
    // Add custom input
    const auto workerInputOnNewThread = false;
    opWrapper.setWorkerInput(wUserInput, workerInputOnNewThread);
    // Add custom processing
    const auto workerProcessingOnNewThread = false;
    opWrapper.setWorkerPostProcessing(wUserPostProcessing, workerProcessingOnNewThread);
    // Add custom output
    const auto workerOutputOnNewThread = true;
    opWrapper.setWorkerOutput(wUserOutput, workerOutputOnNewThread);
    // Configure OpenPose
    op::log("Configuring OpenPose wrapper.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    const op::WrapperStructPose wrapperStructPose{!FLAGS_body_disable, netInputSize, outputSize, keypointScale,
                                                  FLAGS_num_gpu, FLAGS_num_gpu_start, FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose),
                                                  poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, FLAGS_model_folder,
                                                  heatMapTypes, heatMapScale, FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold, enableGoogleLogging};
    // Face configuration (use op::WrapperStructFace{} to disable it)
    const op::WrapperStructFace wrapperStructFace{FLAGS_face, faceNetInputSize,
                                                  op::flagsToRenderMode(FLAGS_face_render, FLAGS_render_pose),
                                                  (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap,
                                                  (float)FLAGS_face_render_threshold};
    // Hand configuration (use op::WrapperStructHand{} to disable it)
    const op::WrapperStructHand wrapperStructHand{FLAGS_hand, handNetInputSize, FLAGS_hand_scale_number,
                                                  (float)FLAGS_hand_scale_range, FLAGS_hand_tracking,
                                                  op::flagsToRenderMode(FLAGS_hand_render, FLAGS_render_pose),
                                                  (float)FLAGS_hand_alpha_pose, (float)FLAGS_hand_alpha_heatmap,
                                                  (float)FLAGS_hand_render_threshold};
    // Consumer (comment or use default argument to disable any output)
    const bool displayGui = false;
    const bool guiVerbose = false;
    const bool fullScreen = false;
    const op::WrapperStructOutput wrapperStructOutput{displayGui, guiVerbose, fullScreen, FLAGS_write_keypoint,
                                                      op::stringToDataFormat(FLAGS_write_keypoint_format),
                                                      writeJson, FLAGS_write_coco_json,
                                                      FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,
                                                      FLAGS_write_heatmaps, FLAGS_write_heatmaps_format};
    // Configure wrapper
    opWrapper.configure(wrapperStructPose, wrapperStructFace, wrapperStructHand, op::WrapperStructInput{},
                        wrapperStructOutput);
    // Set to single-thread running (to debug and/or reduce latency)
    if (FLAGS_disable_multi_thread)
       opWrapper.disableMultiThreading();

    op::log("Starting thread(s)", op::Priority::High);
    // Two different ways of running the program on multithread environment
    // // Option a) Recommended - Also using the main thread (this thread) for processing (it saves 1 thread)
    // // Start, run & stop threads
    opWrapper.exec();  // It blocks this thread until all threads have finished

    // Option b) Keeping this thread free in case you want to do something else meanwhile, e.g. profiling the GPU memory
    // // VERY IMPORTANT NOTE: if OpenCV is compiled with Qt support, this option will not work. Qt needs the main
    // // thread to plot visual results, so the final GUI (which uses OpenCV) would return an exception similar to:
    // // `QMetaMethod::invoke: Unable to invoke methods with return values in queued connections`
    // // Start threads
    // opWrapper.start();
    // // Profile used GPU memory
    //     // 1: wait ~10sec so the memory has been totally loaded on GPU
    //     // 2: profile the GPU memory
    // std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    // op::log("Random task here...", op::Priority::High);
    // // Keep program alive while running threads
    // while (opWrapper.isRunning())
    //     std::this_thread::sleep_for(std::chrono::milliseconds{33});
    // // Stop and join threads
    // op::log("Stopping thread(s)", op::Priority::High);
    // opWrapper.stop();

    // Measuring total time
    const auto now = std::chrono::high_resolution_clock::now();
    const auto totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(now-timerBegin).count()
                            * 1e-9;
    const auto message = "Real-time " + std::to_string(_count) + " pose estimation demo successfully finished. Total time: "
                       + std::to_string(totalTimeSec) + " seconds. (" + std::to_string(_count / totalTimeSec ) + " Hz)";
    op::log(message, op::Priority::High);

    
//op::POSE_COCO_BODY_PARTS
    mp.bodypartsmap = op::getPoseBodyPartMapping(static_cast<op::PoseModel>(poseModel));
    //mp.bodypartsmap = op::POSE_COCO_BODY_PARTS;
    mp.bodypartsmap = op::getPoseBodyPartMapping(op::flagsToPoseModel(FLAGS_model_pose));
    std::map<unsigned int, std::string>::iterator mapiter = mp.bodypartsmap.begin();
    for(mapiter=mp.bodypartsmap.begin();mapiter !=mp.bodypartsmap.end();++mapiter)
        std::cout<<mapiter->second<<std::endl;
    //for(size_t idx=0;i<bodypartsmap.size();idx++)
        //cout<<bodypar

    return 0;
}





void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

    //ROS_INFO("cloud callback");
    //pcl::fromROSMsg(*msg, *cloud);


}






void callback(const sensor_msgs::Image &img)
{
    
    try
    {
        ROS_INFO("hello");
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        cv_ptr->header=img.header;
         h.stamp = cv_ptr->header.stamp;
         
        std::cout<<"stamp of h "<<h.stamp<<std::endl;
        
        std::cout<<"stamp of header of cv ptr "<<cv_ptr->header.stamp<<std::endl;
        //cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    /*ToDo 
    op::Array<float> netInputArray;
    std::vector<float> scaleRatios;
    op::Array<float> outputArray;

    // process
    std::tie(netInputArray, scaleRatios) = cvMatToOpInput->format(cv_ptr->image);
    double scaleInputToOutput;
    std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput->format(cv_ptr->image);
    // Step 3 - Estimate poseKeypoints
    poseExtractorCaffe->forwardPass(netInputArray, {cv_ptr->image.cols, cv_ptr->image.rows}, scaleRatios);
    const auto poseKeypoints1 = poseExtractorCaffe->getPoseKeypoints();
    const auto faces = faceDetector->detectFaces(poseKeypoints1, scaleInputToOutput);
    faceExtractor->forwardPass(faces, cv_ptr->image, scaleInputToOutput);
    const auto faceKeypoints = faceExtractor->getFaceKeypoints();
    */



//  cout << "New image " << img.width << "x" << img.height << " " << img.data.size() << endl;
}

MyPublisher::MyPublisher(void):cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    
    //bodypartsmap = op::POSE_COCO_BODY_PARTS;
    //bodypartsmap = op::getPoseBodyPartMapping(pose_model);
}


void MyPublisher::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){

    ROS_INFO("clouuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuud callback");
    pcl::fromROSMsg(*msg, *cloud);

    h_min.stamp = msg->header.stamp;
    std::cout << h_min.stamp << " was h_min and here is h_temp "<< h_temp.stamp<<std::endl;
    if (h_min.stamp >h_temp.stamp &&  a ==0 )
    {
        a=1;
        for (int it = 0 ; it < clouds.size(); ++it)
           {
             clouds[it]->clear();
           } 
        clouds.clear();
        headers.clear();
    }
    else
    {
        a=0;
        h_temp.stamp = ros::Time::now();
        clouds.push_back(cloud);
        headers.push_back(h_min);
    }
    //ROS_INFO("cloud size: %d  callback", static_cast<int>(cloud->points.size()));
}

void MyPublisher::callback(const op::Array<float> &poseKeypoints)
{
    ros::Time t = ros::Time::now();
    ROS_INFO("My publisher callback");
//  openpose_wrapper::OpenPose msg;
    //openpose_ros_wrapper_msgs::Persons persons;
    
    persons=openpose_ros_wrapper_msgs::Persons();
    persons.rostime = t;
    persons.image_w = 640;
    persons.image_h = 480;

    persons_3d=openpose_ros_wrapper_msgs::Persons3d();
    persons_3d.rostime = t;
    persons_3d.image_w = 640;
    persons_3d.image_h = 480;
         
  
    if(cloud_temp->points.size() == 0)
    { std::cout<<" Cloud temp is empty " << std::endl;
         std::cout << " size of headers is " << headers.size()<<std::endl;
        return;}
//copied 

// original msgs_mk
//#####################################################################################3
    const int num_people = poseKeypoints.getSize(0);
    const int num_bodyparts = poseKeypoints.getSize(1);

    std::vector<int> num_count(3,0);
    std::vector<float> mean_dist(3,0.0);
    //int z_count =0;
    //float mean_z =0.0;

    for(size_t person_idx = 0; person_idx < num_people; person_idx++) {
        openpose_ros_wrapper_msgs::PersonDetection person;
        openpose_ros_wrapper_msgs::PersonDetection3d person_3d;


        for(size_t c_idx =0 ;c_idx<num_count.size();c_idx++)
        {
        
            num_count[c_idx]=0;
            mean_dist[c_idx]=0.0;
        
        }

        for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++) {

            size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);
            openpose_ros_wrapper_msgs::BodyPartDetection bodypart;
            bodypart.part_id = bodypart_idx;
            bodypart.x = poseKeypoints[final_idx];
            bodypart.y = poseKeypoints[final_idx+1];
            bodypart.confidence = poseKeypoints[final_idx+2];
            person.body_part.push_back(bodypart);

            openpose_ros_wrapper_msgs::BodyPartDetection3d bodypart_3d;
            bodypart_3d.part_id = bodypart_idx;
            bodypart_3d.x = poseKeypoints[final_idx];
            bodypart_3d.y = poseKeypoints[final_idx+1];
            //bodypart_3d.z = 0 ;
            bodypart_3d.confidence = poseKeypoints[final_idx+2];

            //extract z information
            if(bodypart_3d.x>0 && bodypart_3d.y>0){

                int point_idx = bodypart.x+bodypart.y*CAMERA_PIXEL_WIDTH;
                float point_z=0.0;
                float point_x=0.0;
                float point_y=0.0;

                if ((point_idx<0) || (!pcl::isFinite(cloud_temp->points[point_idx]))){
                    continue;
                }
                else{

                   if(cloud_temp->points[point_idx].z) 
                   {
                       point_x = cloud_temp->points[point_idx].x;
                       point_y = cloud_temp->points[point_idx].y;
                       point_z = cloud_temp->points[point_idx].z;

                   }
                    //ROS_INFO("bodyparpoint x : %d , y: %d , point idx :%d , point_z : %.3f ",bodypart_3d.x, bodypart_3d.y, point_idx, point_z);
                    bodypart_3d.x =point_x; 
                    bodypart_3d.y =point_y; 
                    bodypart_3d.z =point_z; 

                    for(size_t c_idx =0 ;c_idx<num_count.size();c_idx++)
                        num_count[c_idx]++;
                    
                    mean_dist[0]+=bodypart_3d.x;
                    mean_dist[1]+=bodypart_3d.y;
                    mean_dist[2]+=point_z;
                    
                }
                //ROS_INFO("bodyparpoint x : %d , y: %d , point idx :%d , point_z : %d ",bodypart_3d.x, bodypart_3d.y, point_idx, point_3d.z);
            }
            else{
                bodypart_3d.z=0;
            }
            
            person_3d.body_part.push_back(bodypart_3d);

        }
        
        //calculate average_distance of body parts
        for(size_t c_idx =0 ;c_idx<num_count.size();c_idx++)
        {
        
            if(num_count[c_idx]!=0)
                mean_dist[c_idx] =static_cast<float>(mean_dist[c_idx]/num_count[c_idx]);
            else
                mean_dist[c_idx]=0.0;

        }

        //mean_z =s;tatic_cast<float>(mean_z/z_count);
        //person_3d.avg_pose.header.frame_id = (*cloud).header.frame_id;
        person_3d.avg_pose.header.frame_id = "head_rgbd_sensor_rgb_frame";
        person_3d.avg_pose.pose.position.x= mean_dist[0];
        person_3d.avg_pose.pose.position.y= mean_dist[1];
        person_3d.avg_pose.pose.position.z= mean_dist[2];
        person_3d.avg_pose.pose.orientation.x= 0.0;
        person_3d.avg_pose.pose.orientation.y= 0.0;
        person_3d.avg_pose.pose.orientation.z= 0.0;
        person_3d.avg_pose.pose.orientation.w= 1.0;
        
        persons.persons.push_back(person);
        persons_3d.persons.push_back(person_3d);
    }

    pose_pub.publish(persons);
    pose_3d_pub.publish(persons_3d);
 

    //int numHuman = poseKeypoints.getSize(0);
    //int numPart  = poseKeypoints.getSize(1);
    //int numInfo  = poseKeypoints.getSize(2);

    //mp.msg.layout.dim.clear();
    //mp.msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    //mp.msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    //mp.msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    //mp.msg.layout.dim[0].size = numHuman;
    //mp.msg.layout.dim[0].stride = numPart*numInfo;
    //mp.msg.layout.dim[0].label = "human";
    //mp.msg.layout.dim[1].size = numPart;
    //mp.msg.layout.dim[1].stride = numInfo;
    //mp.msg.layout.dim[1].label = "body";
    //mp.msg.layout.dim[2].size = numInfo;
    //mp.msg.layout.dim[2].stride = 1;
    //mp.msg.layout.dim[2].label = "info";
    //mp.msg.data.resize(numHuman*numPart*numInfo);

    //op::log("DIM " + std::to_string(numHuman) + std::to_string(numPart) + std::to_string(numInfo));
    //for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
    //{
        //op::log("Person " + std::to_string(person) + ": (x, y, score):");
        //for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
        //{
            //float x, y, p;

            //x = poseKeypoints[{person, bodyPart, 0}];
            //y = poseKeypoints[{person, bodyPart, 1}];
            //p = poseKeypoints[{person, bodyPart, 2}];

//#if 0
//#else
            //mp.msg.data[person*numPart*numInfo + bodyPart*numInfo + 0 ] = x;
            //mp.msg.data[person*numPart*numInfo + bodyPart*numInfo + 1 ] = y;
            //mp.msg.data[person*numPart*numInfo + bodyPart*numInfo + 2 ] = p;
//#endif
        //}
    //}
    //publisher.publish(mp.msg);

//#####################################################################################3

    //publish image
    sensor_msgs::Image ros_image;
    out_msg.encoding =sensor_msgs::image_encodings::RGB8;
    image_skeleton_pub.publish(out_msg.toImageMsg());

    eye_pub.publish(eye_msg);

}

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "openpose_wrapper");
    ros::NodeHandle nh;

    h_min.stamp = ros::Time::now();
    h.stamp = ros::Time::now();
    a=1;
    ros::start();
//  mp.publisher = nh.advertise<openpose_wrapper::OpenPose>("openpose_human_body", 1000);
    mp.publisher = nh.advertise<std_msgs::Float32MultiArray>("openpose_human_body", 1000);
    mp.image_skeleton_pub = nh.advertise<sensor_msgs::Image>( "/openpose_ros/detected_poses_image", 1 );  
    mp.pose_pub = nh.advertise<openpose_ros_wrapper_msgs::Persons>("/openpose/pose", 2);
    mp.pose_3d_pub = nh.advertise<openpose_ros_wrapper_msgs::Persons3d>("/openpose/pose_3d", 2);
    mp.keypoints_pub = nh.advertise<openpose_ros_msgs::PersonDetection>( "/openpose_ros/skeleton_3d/detected_poses_keypoints" , 0 );
    mp.eye_pub = nh.advertise<openpose_ros_wrapper_msgs::EyeDetection>( "/openpose_ros/eye_detections" , 0 );
    mp.pcl_sub= nh.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10,
                                       &MyPublisher::cloud_callback, &mp);
    //ros::Subscriber subscriber = nh.subscribe( FLAGS_image_dir, 1, callback);
    ros::Subscriber subscriber = nh.subscribe( "/hsrb/head_rgbd_sensor/rgb/image_raw", 1, callback);
    //ros::Subscriber subscriber = nh.subscribe( FLAGS_image_dir, 1, callback);

    cout << "Subscribed" << endl;

    // Parsing command line flags

    // Running openPoseTutorialWrapper2
    openPoseTutorialWrapper2();

    ros::shutdown();

    return 0;
}
