#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>


// Provide goal_position for swarm (if leader selection is active it will only control leader)
class Control {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle global_nh_;
    std::string namespace_;

    ros::Publisher goal_direction_pub;

    double goal_x_;
    double goal_y_;
    double goal_z_;


public:
    Control(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const ros::NodeHandle& global_nh) {
        nh_ = nh;
        private_nh_ = private_nh;
        global_nh_ = global_nh;
        namespace_ = nh_.getNamespace();
        
        goal_direction_pub = global_nh_.advertise<geometry_msgs::PointStamped>(
            "goal_direction_main", 10);
        
        initParameters();
    }

    void run() {
        publishGoalDirection();
    }

private:
    // Initialize node parameters
    void initParameters() {
        private_nh_.param<double>("goal_x", goal_x_, -5.0);
        private_nh_.param<double>("goal_y", goal_y_, 0.0);
        private_nh_.param<double>("goal_z", goal_z_, 1.5);

    }

    // TODO: Provide goal position using remote control (roll-pitch-yaw)
    void publishGoalDirection() {
        geometry_msgs::PointStamped goal_direction;

        goal_direction.header.stamp = ros::Time::now();
        goal_direction.header.frame_id = "world";
        goal_direction.point.x = goal_x_;
        goal_direction.point.y = goal_y_;
        goal_direction.point.z = goal_z_;
        goal_direction_pub.publish(goal_direction);
    }
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::NodeHandle global_nh("/swarm");
    
    Control control_node(nh, private_nh, global_nh);
    
    while (ros::ok())
    {
        control_node.run();
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}