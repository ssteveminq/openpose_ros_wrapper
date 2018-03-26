#include "filter_people_marker.h"

filter_PeopleMarker::filter_PeopleMarker() :
    detect_seq(0),
    marker_seq(0),
    IsHeadMoving(false),
    IsRobotMoving(false)
{
    ros::NodeHandle n;

    target_frame="map";
    //target_frame="head_rgbd_sensor_rgb_frame";

    // listener = new tf::TransformListener();
    //op_people_sub=n.subscribe<openpose_ros_wrapper_msgs::Persons3d>("/openpose/pose_3d", 10, &filter_PeopleMarker::openpose3d_callback,this);
    //people_pose_pub=n.advertise<geometry_msgs::PoseArray>("/openpose_pose_array", 10, true);
    filter_people_sub=n.subscribe<geometry_msgs::PoseArray>("/openpose_filter_pose_array", 10, &filter_PeopleMarker::filter_people_callback,this);
    
    people_marker_pub=n.advertise<visualization_msgs::MarkerArray>("/people_filter_marker_array", 10, true);
    //people_pose_pub=n.advertise<geometry_msgs::PoseArray>("/openpose_pose_array", 10, true);
    global_pos_sub= n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10,&filter_PeopleMarker::global_pose_callback, this);
    joint_states_sub= n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10,&filter_PeopleMarker::joint_states_callback, this);

    global_pose.resize(3,0.0);
    pre_global_pose.resize(3,0.0);
    Head_Pos.resize(2,0.0);
    Head_vel.resize(2,0.0);

    ros::spin();
}


void filter_PeopleMarker::createVisualisation(std::vector<geometry_msgs::Pose> poses) {
    ROS_DEBUG("Creating markers");
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < poses.size(); i++) {
        std::vector<visualization_msgs::Marker> human = createHuman(i*10, poses[i]);
        marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
    }
    people_marker_pub.publish(marker_array);
}

std::vector<double> filter_PeopleMarker::cartesianToPolar(geometry_msgs::Point point) {
    ROS_DEBUG("cartesianToPolar: Cartesian point: x: %f, y: %f, z %f", point.x, point.y, point.z);
    std::vector<double> output;
    double dist = sqrt(pow(point.x,2) + pow(point.y,2));
    double angle = atan2(point.y, point.x);
    output.push_back(dist);
    output.push_back(angle);
    ROS_DEBUG("cartesianToPolar: Polar point: distance: %f, angle: %f", dist, angle);
    return output;
}


void filter_PeopleMarker::filter_people_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    
    marker_seq=0;
    int num_of_detected_human=msg->poses.size();
    std::vector<geometry_msgs::Pose> poseVector;

    //ROS_INFO("openpose_callback : people_size : %d", num_of_detected_human);

    if(num_of_detected_human>0)
       poseVector.clear();
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {

      poseVector.push_back(msg->poses[i]);
    }
  
    createVisualisation(poseVector);
    //publish_poses(poseVector);
}

void filter_PeopleMarker::human_boxes_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    marker_seq=0;

    int num_of_detected_human=msg->markers.size();
    std::vector<geometry_msgs::Pose> poseVector;

    if(num_of_detected_human>0)
       poseVector.clear();
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {
      poseVector.push_back( msg->markers[i].pose);

    }
  
    createVisualisation(poseVector);
  
}

void filter_PeopleMarker::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

	Head_Pos[0]=msg->position[9];			//pan
	Head_Pos[1]=msg->position[10];			//tilt
	
	Head_vel[0]=msg->velocity[9];
	Head_vel[1]=msg->velocity[10];


    double head_vel =0.0;
    head_vel = pow(Head_vel[0],2)+pow(Head_vel[1],2);
    head_vel = sqrt(head_vel);

	//ROS_INFO("Head moving : %.3lf",head_vel);

    if(head_vel>0.015)
        IsHeadMoving=true;
    else
        IsHeadMoving=false;

}

void filter_PeopleMarker:: publish_poses(std::vector<geometry_msgs::Pose> input_poses){
    
    if(IsHeadMoving || IsRobotMoving )
        return;
    else
    {
        geometry_msgs::PoseArray poses_array_msg;

        poses_array_msg.header.frame_id =target_frame;
        poses_array_msg.header.stamp = ros::Time::now();

        for(int i(0);i<input_poses.size();i++)
            poses_array_msg.poses.push_back(input_poses[i]);

        people_pose_pub.publish(poses_array_msg);
    }
}

void filter_PeopleMarker::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    global_pose.resize(3);

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 
	
   //update current global pose
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;
	global_pose[2]=yaw_tf;

   //check distance with previous pose
    double move_distance =0.0;
    for(size_t idx(0);idx<global_pose.size();idx++)
        move_distance+=pow((pre_global_pose[idx]-global_pose[idx]),2);
    move_distance=sqrt(move_distance);

    //ROS_INFO("robot is moving : %.3lf", move_distance);

    if(move_distance>0.008)
        IsRobotMoving=true;
    else
        IsRobotMoving=false;
    

    //save to previous global pose
    for(size_t idx(0);idx<global_pose.size();idx++)
        pre_global_pose[idx]=global_pose[idx];

}



geometry_msgs::Point filter_PeopleMarker::generate_position(geometry_msgs::Point centre, double angle, double dx, double dy)
{
      float s = sin(angle);
      float c = cos(angle);

      // rotate point
      geometry_msgs::Point res;
      res.x = dx * c - dy * s;
      res.y = dx * s + dy * c;

      // translate point back:
      res.x += centre.x;
      res.y += centre.y;
      res.z  = centre.z;
      return res;
}

geometry_msgs::Pose filter_PeopleMarker::generate_extremity_position(geometry_msgs::Pose centre, double dx, double dy, double z) {
    double angle = tf::getYaw(centre.orientation) + 3.141592/2;
    geometry_msgs::Point p = centre.position;
    p.z = z;
    centre.position = generate_position(p, angle, dx, dy);
    return centre;
}

visualization_msgs::Marker filter_PeopleMarker::createMarker(
        int id,
        int type,
        int action,
        geometry_msgs::Pose pose,
        geometry_msgs::Vector3 scale,
        std_msgs::ColorRGBA color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = ++marker_seq;
    marker.ns = "people_tracker";
    marker.id = id;
    marker.type = type;
    marker.action = action;
    marker.pose = pose;
    marker.scale = scale;
    marker.color = color;
    marker.lifetime = ros::Duration(1);
    return marker;
}

visualization_msgs::Marker filter_PeopleMarker::createHead(
        int id,
        int action,
        geometry_msgs::Pose pose) {
    geometry_msgs::Vector3 scale;
    scale.x = 0.22;
    scale.y = 0.22;
    scale.z = 0.22;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 233.0F/255.0F;
    color.g = 150.0F/255.0F;
    color.b = 122.0F/255.0F;
    pose.position.z = 1.1;
    return createMarker(id, visualization_msgs::Marker::SPHERE, action, pose, scale, color);
}

visualization_msgs::Marker filter_PeopleMarker::createBody(
        int id,
        int action,
        geometry_msgs::Pose pose) {
    geometry_msgs::Vector3 scale;
    scale.x = 0.25;
    scale.y = 0.25;
    scale.z = 0.55;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 59.0F/255.0F;
    color.g = 10.0F/255.0F;
    color.b = 100.0F/255.0F;
    pose.position.z = 0.7;
    return createMarker(id, visualization_msgs::Marker::CYLINDER, action, pose, scale, color);
}

std::vector<visualization_msgs::Marker> filter_PeopleMarker::createLegs(
        int idl, int idr,
        int action,
        geometry_msgs::Pose pose) {
    std::vector<visualization_msgs::Marker> legs;
    geometry_msgs::Vector3 scale;
    scale.x = 0.1;
    scale.y = 0.16;
    scale.z = 0.6;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.0F/255.0F;
    color.g = 0.0F/255.0F;
    color.b = 139.0F/255.0F;
    legs.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.1, 0.0, 0.25), scale, color));
    legs.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.1, 0.0, 0.25), scale, color));
    return legs;
}

std::vector<visualization_msgs::Marker> filter_PeopleMarker::createArms(
        int idl, int idr,
        int action,
        geometry_msgs::Pose pose) {
    std::vector<visualization_msgs::Marker> arms;
    geometry_msgs::Vector3 scale;
    scale.x = 0.1;
    scale.y = 0.1;
    scale.z = 0.45;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 139.0F/255.0F;
    color.g = 0.0F/255.0F;
    color.b = 0.0F/255.0F;
    arms.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.2, 0.0, 0.76), scale, color));
    arms.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.2, 0.0, 0.76), scale, color));
    return arms;
}

std::vector<visualization_msgs::Marker> filter_PeopleMarker::createHuman(int id,geometry_msgs::Pose pose) {
    std::vector<visualization_msgs::Marker> human;
    human.push_back(createHead(id++, visualization_msgs::Marker::ADD, pose));
    human.push_back(createBody(id++, visualization_msgs::Marker::ADD, pose));
    std::vector<visualization_msgs::Marker> legs = createLegs(id++, id++, visualization_msgs::Marker::ADD, pose);
    human.insert(human.end(), legs.begin(), legs.end());
    std::vector<visualization_msgs::Marker> arms = createArms(id++, id++, visualization_msgs::Marker::ADD, pose);
    human.insert(human.end(), arms.begin(), arms.end());
    return human;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_people_marker");
    filter_PeopleMarker* pl = new filter_PeopleMarker();
    return 0;
}
