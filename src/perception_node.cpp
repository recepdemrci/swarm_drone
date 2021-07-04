#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <vision_msgs/BoundingBox3DArray.h>

// Percept UAV's and straits position
class Perception {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string namespace_;

    ros::Publisher detected_straits_pub;
    ros::Publisher positions_of_detected_uavs_pub;
    ros::Subscriber color_image_raw_sub;
    ros::Subscriber depth_image_raw_sub;

    int swarm_size_;
    std::string mav_name_;


public:
    Perception(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) {
        nh_ = nh;
        private_nh_ = private_nh;
        namespace_ = nh_.getNamespace();
        mav_name_ = namespace_.substr(1, namespace_.find('_'));
        
        detected_straits_pub = nh_.advertise<vision_msgs::BoundingBox3DArray>(
            "detected_straits", 10);
        positions_of_detected_uavs_pub = nh_.advertise<geometry_msgs::PoseArray>(
            "detected_uavs_positions", 10);       
        initParameters();
    }

    void run() {
        publishStraits();
        publishUAVPositions();
    }

private:
    // Initialize node parameters
    void initParameters() {
        private_nh_.param<int>("swarm_size", swarm_size_, 5);
    }

    // TODO: calculate uav_position using camera 
    void publishUAVPositions() {
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

    // TODO: detect straits using camera
    void publishStraits() {
        vision_msgs::BoundingBox3D temp_box;
        vision_msgs::BoundingBox3DArray detected_straits;
        geometry_msgs::TransformStamped transformStamped[1];          
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        int id = namespace_.back() - '0';
        try {
            detected_straits.header.stamp = ros::Time::now();
            // for (int i = 0; i < straits_number; i++) {
            transformStamped[0] = tfBuffer.lookupTransform(
                                            mav_name_ + std::to_string(id) + "/base_link",
                                            "world",
                                            ros::Time(0), ros::Duration(3.0));
            
            
            // strait_1
            temp_box.center.position.x = transformStamped[0].transform.translation.x - 3.5;
            temp_box.center.position.y = transformStamped[0].transform.translation.y + 1.6;
            temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            temp_box.center.orientation.x = 0.0;
            temp_box.center.orientation.y = 0.0;
            temp_box.center.orientation.z = 0.0;
            temp_box.size.x = 0.0;
            temp_box.size.y = 1.0;
            temp_box.size.z = 0.5;
            detected_straits.boxes.push_back(temp_box);

            // strait_2
            // temp_box.center.position.x = transformStamped[0].transform.translation.x + 1.1;
            // temp_box.center.position.y = transformStamped[0].transform.translation.y + 1.1;
            // temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            // temp_box.center.orientation.x = 0.0;
            // temp_box.center.orientation.y = 0.0;
            // temp_box.center.orientation.z = 0.0;
            // temp_box.size.x = 0.0;
            // temp_box.size.y = 1.0;
            // temp_box.size.z = 0.5;
            // detected_straits.boxes.push_back(temp_box);
            
            // strait_3
            temp_box.center.position.x = transformStamped[0].transform.translation.x + 1.5;
            temp_box.center.position.y = transformStamped[0].transform.translation.y - 2.1;
            temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            temp_box.center.orientation.x = 0.0;
            temp_box.center.orientation.y = 0.0;
            temp_box.center.orientation.z = 0.0;
            temp_box.size.x = 0.0;
            temp_box.size.y = 1.0;
            temp_box.size.z = 0.5;
            detected_straits.boxes.push_back(temp_box);

            // strait_4
            temp_box.center.position.x = transformStamped[0].transform.translation.x + 1.5;
            temp_box.center.position.y = transformStamped[0].transform.translation.y + 2.1;
            temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            temp_box.center.orientation.x = 0.0;
            temp_box.center.orientation.y = 0.0;
            temp_box.center.orientation.z = 0.0;
            temp_box.size.x = 0.0;
            temp_box.size.y = 1.0;
            temp_box.size.z = 0.5;
            detected_straits.boxes.push_back(temp_box);

            // strait_5
            // temp_box.center.position.x = transformStamped[0].transform.translation.x - 0.6;
            // temp_box.center.position.y = transformStamped[0].transform.translation.y + 1.8;
            // temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            // temp_box.center.orientation.x = 0.0;
            // temp_box.center.orientation.y = 0.0;
            // temp_box.center.orientation.z = 0.0;
            // temp_box.size.x = 0.0;
            // temp_box.size.y = 1.0;
            // temp_box.size.z = 0.5;
            // detected_straits.boxes.push_back(temp_box);

            // strait_6
            // temp_box.center.position.x = transformStamped[0].transform.translation.x - 0.8;
            // temp_box.center.position.y = transformStamped[0].transform.translation.y + 2.0;
            // temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            // temp_box.center.orientation.x = 0.0;
            // temp_box.center.orientation.y = 0.0;
            // temp_box.center.orientation.z = 0.0;
            // temp_box.size.x = 0.0;
            // temp_box.size.y = 1.0;
            // temp_box.size.z = 0.5;
            // detected_straits.boxes.push_back(temp_box);

            // strait_7
            // temp_box.center.position.x = transformStamped[0].transform.translation.x - 6.0;
            // temp_box.center.position.y = transformStamped[0].transform.translation.y + 4.0;
            // temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            // temp_box.center.orientation.x = 0.0;
            // temp_box.center.orientation.y = 0.0;
            // temp_box.center.orientation.z = 0.0;
            // temp_box.size.x = 0.0;
            // temp_box.size.y = 1.0;
            // temp_box.size.z = 0.5;
            // detected_straits.boxes.push_back(temp_box);

            // strait_8
            temp_box.center.position.x = transformStamped[0].transform.translation.x - 10.0;
            temp_box.center.position.y = transformStamped[0].transform.translation.y + 1.0;
            temp_box.center.position.z = transformStamped[0].transform.translation.z + 1.6;
            temp_box.center.orientation.x = 0.0;
            temp_box.center.orientation.y = 0.0;
            temp_box.center.orientation.z = 0.0;
            temp_box.size.x = 0.0;
            temp_box.size.y = 1.0;
            temp_box.size.z = 0.5;
            detected_straits.boxes.push_back(temp_box);

            detected_straits_pub.publish(detected_straits);
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