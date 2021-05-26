#include <math.h>

#include <ros/ros.h>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define MIN_DIST 100000
#define GOAL_THRESHOLD 0.4

class Prediction {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string namespace_;

    ros::Publisher trajectory_pub;
    ros::Subscriber odometry_sub;
    ros::Subscriber goal_direction_sub;
    ros::Subscriber positions_of_detected_uavs_sub;

    float goal_factor_;
    float uniform_distance_;
    float unification_factor_;
    Eigen::Vector3d odometry_;
    Eigen::Vector3d goal_vector_;
    Eigen::Vector3d center_point_;
    Eigen::Vector3d maintenance_vector_;
    Eigen::Vector3d unification_vector_;


public:
        Prediction(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) {
        // Definition of ros nodehandler
        nh_ = nh;
        private_nh_ = private_nh;
        namespace_ = nh_.getNamespace();

        // Definition of subscribers       
        odometry_sub = nh_.subscribe(
            "ground_truth/odometry", 1,
            &Prediction::odometryCallback, this);
        goal_direction_sub = nh_.subscribe(
            "goal_direction", 1, 
            &Prediction::goalCallback, this);
        positions_of_detected_uavs_sub = nh_.subscribe(
            "detected_uavs_positions", 1, 
            &Prediction::positionOfDetectedUAVsCallback, this);

        // Definition of publishers
        trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);   

        // Defination of other variable
        goal_vector_ = Eigen::Vector3d::Zero();
        maintenance_vector_ = Eigen::Vector3d::Zero();
        unification_vector_ = Eigen::Vector3d::Zero();
        initParameters();
    }

private:       
    // Initialize ros parameters
    void initParameters() {
        private_nh_.param<float>("uniform_distance", uniform_distance_, 2.0);
        private_nh_.param<float>("goal_factor", goal_factor_, 1.0);
        private_nh_.param<float>("unification_factor", unification_factor_, 0.1);
    }

    // Calback Functions ------------------------------------------------------------------------------------------------------------------------------------------------
    void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
        Eigen::Vector3d temp_odom;
        temp_odom(0) = odometry_msg->pose.pose.position.x;
        temp_odom(1) = odometry_msg->pose.pose.position.y;
        temp_odom(2) = odometry_msg->pose.pose.position.z;
        odometry_ = temp_odom;
    }

    // Set goal_vector each time it is called
    void goalCallback(const geometry_msgs::PointStampedConstPtr& goal_direction) {
        Eigen::Vector3d temp_vector;
        
        if (odometry_.isZero()) {
            return;
        }

        // Set goal_vector based on UAV frame
        temp_vector(0) = goal_direction->point.x - odometry_.x();
        temp_vector(1) = goal_direction->point.y - odometry_.y();
        temp_vector(2) = goal_direction->point.z - odometry_.z();
        temp_vector.normalize();
        goal_vector_ = temp_vector * uniform_distance_;
    }

    // TODO: Control if gola is reached, save previous trajectory message and compare them, no big difference)
    // Calculate target_position of UAV itself based on world frame each time other UAVs detected
    void positionOfDetectedUAVsCallback(const geometry_msgs::PoseArrayConstPtr& positions_of_uavs) {
        double target_yaw = 0.0;
        Eigen::Vector3d target_vector;
        Eigen::Vector3d target_position_w;
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

        // Control if goal_vector_ is set
        if (goal_vector_.isZero() || odometry_.isZero()) {
            return;
        }

        // Calculate maintenance, partition, and unification vectors
        calculateMaintenance(*positions_of_uavs);
        calculateUnification(*positions_of_uavs);

        // Calculate target_vector and convert it into target_position related to the world frame
        target_vector = maintenance_vector_ + unification_vector_ * unification_factor_;
        target_position_w = transform_vector(target_vector);

        // Create trajectory message and publish it
        trajectory_msg.header.stamp = positions_of_uavs->header.stamp;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position_w, target_yaw, &trajectory_msg);
        trajectory_pub.publish(trajectory_msg);
    }


    // Other Functions ------------------------------------------------------------------------------------------------------------------------------------------------
    void setMaintenanceVector(double x, double y, double z) {
        Eigen::Vector3d temp_vector;
        temp_vector(0) = x;
        temp_vector(1) = y;
        temp_vector(2) = z;
        maintenance_vector_ = temp_vector;
    }

    void calculateMaintenance(geometry_msgs::PoseArray positions_of_uavs) {
        int sign;
        double m_angle;
        bool found = false;
        Eigen::Vector3d target; 
        Eigen::Vector3d neighbor_1; 
        Eigen::Vector3d neighbor_2;
        Eigen::Vector3d center_position;
        Eigen::Vector3d current_position;

        // Find 1st neighbour
        current_position = Eigen::Vector3d::Zero();
        double temp_dist, min_dist = MIN_DIST;
        for (geometry_msgs::Pose uav_pose : positions_of_uavs.poses) {
            if (inGoalArea(uav_pose.position)) {                    
                temp_dist = dist(current_position, uav_pose.position);
                if (temp_dist < min_dist) {
                    min_dist = temp_dist;
                    neighbor_1(0) = uav_pose.position.x;
                    neighbor_1(1) = uav_pose.position.y;
                    neighbor_1(2) = uav_pose.position.z;
                    found = true;
                }
            }
        }
        if (!found) {
            neighbor_1.array() = goal_vector_.array();
        }
        // Find 2st neighbour
        min_dist = MIN_DIST;
        for (geometry_msgs::Pose uav_pose : positions_of_uavs.poses) {
            temp_dist = dist(neighbor_1, uav_pose.position) + dist(current_position, uav_pose.position);
            if (temp_dist < min_dist) {
                min_dist = temp_dist;
                neighbor_2(0) = uav_pose.position.x;
                neighbor_2(1) = uav_pose.position.y;
                neighbor_2(2) = uav_pose.position.z;
            }
        }
        // Find center position of three uav
        center_position = (current_position + neighbor_1 + neighbor_2) / 3;

        // Find angle between neighbor1 neighbor2 and uav's local x axis
        m_angle = atan((neighbor_2(1) - neighbor_1(1)) / (neighbor_2(0) - neighbor_1(0)));
        // Find sign(+-) of the PI/2 for add to angle (if center_position is behind of the uav, use opposite sign of the angle, otherwise use same sign with angle)
        if (center_position.x() * m_angle < 0) {
            sign = -1;
        }
        else {
            sign = 1;
        }
        // Calculate maintenance vector
        target(0) = center_position(0) + uniform_distance_ * (cos(m_angle + sign *(M_PI/2)) / sqrt(3));
        target(1) = center_position(1) + uniform_distance_ * (sin(m_angle + sign *(M_PI/2)) / sqrt(3));
        target(2) = center_position(2);
        setMaintenanceVector(target(0), target(1), target(2));
    }
    
    // TODO: Implement this function
    void calculatePartition() {

    }
    
    void calculateUnification(geometry_msgs::PoseArray positions_of_uavs) {
        for (geometry_msgs::Pose uav_pose : positions_of_uavs.poses) {
            center_point_(0) += uav_pose.position.x;
            center_point_(1) += uav_pose.position.y;
            center_point_(2) += uav_pose.position.z;
        }
        center_point_(0) /= positions_of_uavs.poses.size() + 1; 
        center_point_(1) /= positions_of_uavs.poses.size() + 1;
        center_point_(2) /= positions_of_uavs.poses.size() + 1;
        unification_vector_ = goal_vector_ * goal_factor_ + center_point_;
    }
    
    // Transform a vector based on world frame
    Eigen::Vector3d transform_vector(Eigen::Vector3d input_vector) {
        Eigen::Vector3d target_position_w;
        target_position_w(0) = input_vector.x() + odometry_.x();
        target_position_w(1) = input_vector.y() + odometry_.y();
        target_position_w(2) = input_vector.z() + odometry_.z();
        return target_position_w;
    }

    // Control if given uav is in the goal area (+90 -90)
    bool inGoalArea(geometry_msgs::Point uav_position) {
        double angle = atan2(goal_vector_.y(), goal_vector_.x()) - atan2(uav_position.y, uav_position.x);
        if ((angle < (M_PI/2)) && (angle > (-M_PI/2))) {
            return true;
        }
        else {
            return false;
        }
    }

    double dist(Eigen::Vector3d point_1, geometry_msgs::Point point_2) {
        double result = pow(point_1.x() - point_2.x,2)
                        + pow(point_1.y() - point_2.y,2)
                        + pow(point_1.z() - point_2.z,2);
        result = sqrt(result);
        if (result == 0.0) {
            return MIN_DIST;
        }
        return result;
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "prediction_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Prediction prediction_node(nh, private_nh);
    ros::spin();
    return 0;
}