#include <string>
#include <boost/thread/mutex.hpp>


// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>



// messages
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// log files
#include <fstream>
#include <vector>


#define FOVW 60       //field of view width
#define MATH_PI 3.14159265359
#define Target_Dist_person 1.0
#define LASER_Dist_person  2.5

class visible_manager
{
public:
  /// constructor
  visible_manager(ros::NodeHandle nh);

  /// destructor
  virtual ~visible_manager();

  /// callback for messages

  void publish_cameraregion();
  void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void human_yolo_callback(const visualization_msgs::MarkerArray::ConstPtr& message);
  void scaled_static_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  bool Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);
  bool check_staticObs(float x_pos,float y_pos);
  bool getlinevalue(int line_type,double input_x, double input_y);
  double getDistance_from_Vec(std::vector<double> origin, double _x, double _y);
  bool check_cameraregion(float x_pos,float y_pos);

  int  globalcoord_To_SScaled_map_index(float x_pos,float y_pos);
  int  CoordinateTransform_Global2_staticMap(float global_x, float global_y);
    
  void getCameraregion();
  void spin();


private:

  ros::NodeHandle nh_;

  ros::Publisher camera_visible_region_pub;
  ros::Publisher human_target_pub;

  ros::Subscriber people_yolo_sub;
  ros::Subscriber globalpose_sub;
  ros::Subscriber Scaled_static_map_sub;
  ros::Subscriber joint_state_sub;
  
  // tf listener
  tf::TransformListener     robot_state_;
  tf::TransformListener     listener;

  std::string fixed_frame_;
  boost::mutex filter_mutex_;

  std::vector<int> visiblie_idx_set;

  std::vector<double> global_pose;
  std::vector< std::vector< double > > cur_yolo_people;
  std::vector<double> Head_Pos; 

  int num_of_detected_human;
  nav_msgs::OccupancyGrid Scaled_map;
  nav_msgs::OccupancyGrid camera_visible_region;
  
  

}; // class


