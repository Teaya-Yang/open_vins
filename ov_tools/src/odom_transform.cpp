#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>


/*
This is used to convert the rtabmap odom of camera in odom frame to the drone odom in map frame.
Use default rtabmap VO, the /rtabmap/odom header.frame_id should be odom, child_frame_id should be d455_link (use d455 camera).
The /tf tree should provide the map->odom->d455_link transformation and we assume STATIC camera to drone transformation.
*/



#define MAP_FRAME_ID            "mocap"
#define ODOM_PARENT_FRAME_ID    "global"
#define ODOM_CHILD_FRAME_ID     "imu"
#define CAMERA_FRAME_ID         "d455_link"
#define DRONE_FRAME_ID          "drone_link"

#define ODOM_SUB_TOPIC          "/ov_msckf/odomimu"
#define ODOM_PUB_TOPIC          "/odom_data"

// global variables
tf2::Vector3 displacement_vec_in_odomchild; // "arm"/displacement/position offset vector pointing from drone to odomchild, represent in odomchild frame
tf2::Quaternion rotation_odomchild_to_drone;
ros::Publisher odom_pub;

/*
* get the static displacement vector pointing from the COM/geometry center to the odomchild frame origin, represented in the odomchild frame
* get the static transformation between odomchild frame (/imu) to drone frame (default /drone_link)
*/
bool getStaticTransformations(){
    try{
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);

        // get static tf from odom child to drone
        // the position of the tf is the vector pointing from odomchild to drone, represent in odomchild frame
        geometry_msgs::TransformStamped transform_drone_to_odomchild = tfBuffer.lookupTransform(
            ODOM_CHILD_FRAME_ID,         // // Target frame
            DRONE_FRAME_ID,             // Source frame,       
            ros::Time(0)        // use the latest avaible tf
        );
        
        // displacement pointing from drone to odomchild origin, reprerset in odomchild frame, 
        displacement_vec_in_odomchild.setX(-transform_drone_to_odomchild.transform.translation.x);
        displacement_vec_in_odomchild.setY(-transform_drone_to_odomchild.transform.translation.y);
        displacement_vec_in_odomchild.setZ(-transform_drone_to_odomchild.transform.translation.z);

        // get static tf from odom child to drone
        geometry_msgs::TransformStamped transform_odomchild_to_drone = tfBuffer.lookupTransform(
            DRONE_FRAME_ID, 
            ODOM_CHILD_FRAME_ID,    
            ros::Time(0)
        );

        tf2::fromMsg(transform_odomchild_to_drone.transform.rotation, rotation_odomchild_to_drone);

        
        return true;
    }catch(tf2::TransformException& ex){
        ROS_WARN("Failed to get transform: %s", ex.what());
        return false;  // Transform retrieval failed
    }
}


/*
* /rtabmap/odom callback
* tf tree should look like map_frame_id (map)-->odom_parent_frame_id (global)-->odom_child_frame_id(imu)-->camera_frame_id(d455_link)-->drone_frame_id (drone_link)
* pose, linear velocity in map fixed frame and angular velocity in drone body fixed frame 
*/
void openvins_odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    try {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);

        // obtain the tf from the drone to the map frame 
        geometry_msgs::TransformStamped transform_drone_to_map = tfBuffer.lookupTransform(
            MAP_FRAME_ID,       // Target frame
            DRONE_FRAME_ID,     // Source frame
            ros::Time(0)        // use the latest avaible tf
        );

        // obtain the tf from odom child (should be "imu" from default openvins odom) to the map frame
        geometry_msgs::TransformStamped transform_odomchild_to_map = tfBuffer.lookupTransform(
            MAP_FRAME_ID,           // Target frame
            ODOM_CHILD_FRAME_ID,    // Source frame
            ros::Time(0)            // use the latest avaible tf
        );

        // construct odom message of the drone in the map frame
        nav_msgs::Odometry drone_odom_in_map;

        // represeting pose and twist (excep the angular velocity still in body fixed frame) of the drone in the map frame, 
        // TODO: probably better to change it into a customized data struct since this is not following the standard ROS odometry message definition
        drone_odom_in_map.header.frame_id = MAP_FRAME_ID;
        drone_odom_in_map.child_frame_id = MAP_FRAME_ID;

        // fill the pose of odometry message
        drone_odom_in_map.pose.pose.position.x = transform_drone_to_map.transform.translation.x;
        drone_odom_in_map.pose.pose.position.y = transform_drone_to_map.transform.translation.y;
        drone_odom_in_map.pose.pose.position.z = transform_drone_to_map.transform.translation.z;
        drone_odom_in_map.pose.pose.orientation = transform_drone_to_map.transform.rotation;

        // twist of the drone frame in the map frame (same as the twist of the camera frame in the map frame)
        tf2::Quaternion rotation_odomchild_to_map;
        tf2::fromMsg(transform_odomchild_to_map.transform.rotation, rotation_odomchild_to_map);
        
        // extract twist data
        tf2::Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);    // linear velocity in odom_child frame
        tf2::Vector3 w(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z); // angular velocity in odom_child frame

        // compute the the linear velocity cause by the rotation
        tf2::Vector3 v_induced = w.cross(displacement_vec_in_odomchild);
        
        // for debug
        //printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",displacement_vec_in_odomchild.x(),displacement_vec_in_odomchild.y(),displacement_vec_in_odomchild.z(),w.x(),w.y(),w.z(),v.x(),v.y(),v.z(),v_induced.x(),v_induced.y(),v_induced.z());
        
        // corrected linear velocity of the COM represent in odom_child frame
        v -= v_induced; 

        // transfrom into the designed frame
        tf2::Vector3 v_rotated = tf2::quatRotate(rotation_odomchild_to_map, v); //  linear velocity in the map frame
        tf2::Vector3 w_rotated = tf2::quatRotate(rotation_odomchild_to_drone, w); //  angular velocity in the drone body fix frame

        // fill the twist part of the odometry message
        drone_odom_in_map.twist.twist.linear.x = v_rotated.x();
        drone_odom_in_map.twist.twist.linear.y = v_rotated.y();
        drone_odom_in_map.twist.twist.linear.z = v_rotated.z();

        drone_odom_in_map.twist.twist.angular.x = w_rotated.x();
        drone_odom_in_map.twist.twist.angular.y = w_rotated.y();
        drone_odom_in_map.twist.twist.angular.z = w_rotated.z();

        drone_odom_in_map.header.stamp = msg->header.stamp;
        
        odom_pub.publish(drone_odom_in_map);

    } catch (tf2::LookupException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    } catch (tf2::ExtrapolationException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_transform");
    ros::NodeHandle nh;

    // wait until we get the static tf information:
    // odomchild to COM's displacement vector and the tf between the odomchild and the drone frame, which are all defined externally 
    while(!getStaticTransformations()){
        ROS_WARN("Transform not available yet, retrying...");
        ros::Duration(1.0).sleep();
    }

    ros::Subscriber odom_sub = nh.subscribe(ODOM_SUB_TOPIC, 10, openvins_odometry_cb);
    odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_PUB_TOPIC, 10);
    
    ros::spin();
    return 0;
}