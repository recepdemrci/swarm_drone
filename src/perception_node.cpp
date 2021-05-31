#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>


class Perception {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string namespace_;

    ros::Subscriber color_image_raw_sub;
    ros::Subscriber depth_image_raw_sub;
    ros::Subscriber depth_points_sub;
    ros::Publisher positions_of_detected_uavs_pub;
    ros::Publisher goal_direction_pub;

    double goal_x;
    double goal_y;
    double goal_z;
    int swarm_size_;
    std::string mav_name_;


public:
    Perception(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) {
        nh_ = nh;
        private_nh_ = private_nh;
        namespace_ = nh_.getNamespace();
        mav_name_ = namespace_.substr(1, namespace_.find('_'));
        
        positions_of_detected_uavs_pub = nh_.advertise<geometry_msgs::PoseArray>(
            "detected_uavs_positions", 10);
        goal_direction_pub = nh_.advertise<geometry_msgs::PointStamped>(
            "goal_direction", 10);      

        initParameters();
    }

    void run() {
        publishGoalDirection();
        publishMavPositions();
    }

private:
    // Initialize ros parameters
    void initParameters() {
        private_nh_.param<int>("swarm_size", swarm_size_, 5);
        private_nh_.param<double>("goal_x", goal_x, -5.0);
        private_nh_.param<double>("goal_y", goal_y, 0.0);
        private_nh_.param<double>("goal_z", goal_z, 1.5);

    }

    void publishGoalDirection() {
        geometry_msgs::PointStamped goal_direction;

        goal_direction.header.stamp = ros::Time::now();
        goal_direction.header.frame_id = "world";
        goal_direction.point.x = goal_x;
        goal_direction.point.y = goal_y;
        goal_direction.point.z = goal_z;
        goal_direction_pub.publish(goal_direction);
    }

    // TODO: calculate uav_position using camera 
    void publishMavPositions() {
        geometry_msgs::Pose temp_pose;
        geometry_msgs::PoseArray pose_array;
        geometry_msgs::TransformStamped transformStamped[(swarm_size_ - 1)];          
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        int id = namespace_.back() - '0';
        try{
            pose_array.header.stamp = ros::Time::now();
            pose_array.header.frame_id = namespace_.substr(1) + "/base_link";
            
            for (int i = 0; i < (swarm_size_ - 1); i ++) {
                // Transform poistion of UAVs based on current UAV frame
                transformStamped[i] = tfBuffer.lookupTransform(
                                                mav_name_ + std::to_string(id) + "/base_link", 
                                                mav_name_ + std::to_string((id+i+1) % swarm_size_) + "/base_link",
                                                ros::Time(0), ros::Duration(3.0));
            
                // Add UAV position into PoseArray
                temp_pose.position.x = transformStamped[i].transform.translation.x;
                temp_pose.position.y = transformStamped[i].transform.translation.y;
                temp_pose.position.z = transformStamped[i].transform.translation.z;
                pose_array.poses.push_back(temp_pose);
            }
    
            positions_of_detected_uavs_pub.publish(pose_array);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
};
    

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    Perception perception_node(nh, private_nh);
    
    while (ros::ok())
    {
        perception_node.run();
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}