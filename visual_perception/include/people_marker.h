#ifndef HUMAN_MARKER_H
#define HUMAN_MARKER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <openpose_ros_wrapper_msgs/Persons3d.h>

// #include <people_msgs/People.h>
// #include <people_msgs/Person.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <string.h>
#include <vector>
#include <math.h>


#define BASE_LINK "/base_link"

class PeopleMarker
{
public:
    PeopleMarker();

private:
    void createVisualisation(std::vector<geometry_msgs::Pose> points);
    std::vector<double> cartesianToPolar(geometry_msgs::Point point);
    void human_boxes_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void openpose3d_callback(const openpose_ros_wrapper_msgs::Persons3d::ConstPtr& msg);
    void publish_poses(std::vector<geometry_msgs::Pose> points);
    void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    geometry_msgs::Point generate_position(geometry_msgs::Point centre, double angle, double dx, double dy);
    geometry_msgs::Pose generate_extremity_position(geometry_msgs::Pose centre, double dx, double dy, double z);
    visualization_msgs::Marker createHead( int id, int action, geometry_msgs::Pose pose);
    visualization_msgs::Marker createBody( int id, int action, geometry_msgs::Pose pose);
    std::vector<visualization_msgs::Marker> createLegs(int idl, int idr,int action, geometry_msgs::Pose pose);
    std::vector<visualization_msgs::Marker> createArms(int idl, int idr,int action, geometry_msgs::Pose pose);
    std::vector<visualization_msgs::Marker> createHuman(int id,geometry_msgs::Pose pose);
    visualization_msgs::Marker createMarker(int id,int type, int action, 
                                geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color);

    tf::TransformListener listener;
    ros::Publisher  people_marker_pub;
    ros::Publisher  people_pose_pub;
    ros::Subscriber people_boxes_sub;
    ros::Subscriber op_people_sub;
    ros::Subscriber global_pos_sub;
    //tf::TransformListener* listener;
    std::string target_frame;
    unsigned long detect_seq;
    unsigned long marker_seq;
    double startup_time;
    std::string startup_time_str;
   
	std::vector<double> global_pose;
   
    // boost::uuids::uuid dns_namespace_uuid;

    // SimpleTracking<EKFilter> *ekf = NULL;
    // SimpleTracking<UKFilter> *ukf = NULL;
    //std::map<std::pair<std::string, std::string>, ros::Subscriber> subscribers;
};

#endif // HUMAN_MARKER_H
