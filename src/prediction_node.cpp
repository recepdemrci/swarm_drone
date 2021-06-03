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

#define MIN_DIST 100000                                     // Enormous distance for calculate min distance between two UAV
#define SAFE_ALTITUDE 0.5                                   // Safe altitude for each UAV (UAVs can't be below this altitude)
#define UNIT_NUMBER 4                                       // Divide trajectory UNIT_NUMBER value parts to prevent big movement(unit_distance 2, UNIT_NUMBER 4)
static const int64_t kNanoSecondsInSecond = 1000000000;


class Prediction {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    std::string namespace_;

    ros::Publisher trajectory_pub;
    ros::Subscriber odometry_sub;
    ros::Subscriber goal_direction_sub;
    ros::Subscriber positions_of_detected_uavs_sub;

    bool active_;
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
        active_ = false;
        goal_vector_ = Eigen::Vector3d::Zero();
        maintenance_vector_ = Eigen::Vector3d::Zero();
        unification_vector_ = Eigen::Vector3d::Zero();        
        initParameters();
    }

private:       
    // Initialize ros parameters
    void initParameters() {
        private_nh_.param<float>("uniform_distance", uniform_distance_, 4.0);
        private_nh_.param<float>("unification_factor", unification_factor_, 0.14);
    }

    // TODO: We can make 'pow(uniform_distance_, 3)' -> a constant MAX VALUE 
    // Set goal_factor based on proximity to goal 
    void setGoalFactor(Eigen::Vector3d goal_vector) {
        geometry_msgs::Point current_point;

        // Calculate goal_factor based on distance to the goal
        current_point.x = 0.0;
        current_point.y = 0.0;
        current_point.z = 0.0;
        goal_factor_ = (float) dist(goal_vector, current_point);
        if (goal_factor_ > pow(uniform_distance_, 3)) {
            goal_factor_ = pow(uniform_distance_, 3);
        }
    }

    // Set maintenance_vector with parameters
    void setMaintenanceVector(double x, double y, double z) {
        Eigen::Vector3d temp_vector;
        temp_vector(0) = x;
        temp_vector(1) = y;
        temp_vector(2) = z;
        maintenance_vector_ = temp_vector;
    }

    void takeoff() {
        float desired_yaw = 0.0;
        Eigen::Vector3d desired_position(odometry_.x(), odometry_.y(), SAFE_ALTITUDE);
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

        active_ = true;
        trajectory_msg.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
        trajectory_pub.publish(trajectory_msg);

        ROS_ERROR("[%s] : Active ", namespace_.c_str());
        ros::Duration(3.0).sleep();
    }


    // Calback Functions ------------------------------------------------------------------------------------------------------------------------------------------------
    // Update odometry each time
    void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
        Eigen::Vector3d temp_odom;
        temp_odom(0) = odometry_msg->pose.pose.position.x;
        temp_odom(1) = odometry_msg->pose.pose.position.y;
        temp_odom(2) = odometry_msg->pose.pose.position.z;
        odometry_ = temp_odom;
    }

    // Update goal_vector and goal_factor each time
    void goalCallback(const geometry_msgs::PointStampedConstPtr& goal_direction) {
        Eigen::Vector3d temp_vector;

        // Control if odometry is set
        if (odometry_.isZero()) {
            return;
        }

        // Set goal_vector based on UAV frame
        temp_vector(0) = (goal_direction->point.x - odometry_.x());
        temp_vector(1) = (goal_direction->point.y - odometry_.y());
        temp_vector(2) = (goal_direction->point.z - odometry_.z());

        // Set goal_factor
        setGoalFactor(temp_vector);        

        // Normalize goal_vector based on uniform distance
        temp_vector.normalize();
        goal_vector_ = temp_vector * uniform_distance_;
    }

    // Calculate target_position of UAV itself based on world frame each time other UAVs detected
    void positionOfDetectedUAVsCallback(const geometry_msgs::PoseArrayConstPtr& positions_of_uavs) {
        Eigen::Vector3d target_vector;
        trajectory_msgs::MultiDOFJointTrajectoryPtr trajectory_msg;

        // Control if goal_vector_ is set
        if (goal_vector_.isZero() || odometry_.isZero()) {
            return;
        }
        if (!active_) {
            takeoff();
        }

        // Calculate maintenance, partition, and unification vectors
        calculateMaintenance(*positions_of_uavs);
        calculateUnification(*positions_of_uavs);

        // Calculate target_vector and convert it into target_position related to the world frame
        target_vector = maintenance_vector_ + unification_vector_ * unification_factor_;
        trajectory_msg = createTrajectory(target_vector);
        trajectory_pub.publish(trajectory_msg);
    }

    
    // Adaptive Flocking Algorithm Functions --------------------------------------------------------------------------------------------------------------------------
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
            temp_dist = dist(neighbor_1, uav_pose.position);
            if (temp_dist > 0) {
                temp_dist += dist(current_position, uav_pose.position);
                if (temp_dist < min_dist) {
                    min_dist = temp_dist;
                    neighbor_2(0) = uav_pose.position.x;
                    neighbor_2(1) = uav_pose.position.y;
                    neighbor_2(2) = uav_pose.position.z;
                }
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
        center_point_(0) /= (positions_of_uavs.poses.size() + 1); 
        center_point_(1) /= (positions_of_uavs.poses.size() + 1);
        center_point_(2) /= (positions_of_uavs.poses.size() + 1);
        unification_vector_ = goal_vector_ * goal_factor_ + center_point_;
    }
    

    // Other Functions ------------------------------------------------------------------------------------------------------------------------------------------------
    // Control if given uav is in the goal area (+90 -90)
    bool inGoalArea(geometry_msgs::Point uav_position) {
        double angle_uav;
        double angle_goal;

        angle_uav = atan2(uav_position.y, uav_position.x);
        angle_goal = atan2(goal_vector_.y(), goal_vector_.x());
        if (angle_goal < 0) {
            angle_goal += (2 * M_PI);
        }
        if (angle_uav < 0) {
            angle_uav += (2 * M_PI);
        }
        double angle = abs(angle_uav - angle_goal);

        if (angle < (M_PI/2)) {
            return true;
        }
        return false;
    }

    // Transform a vector based on world frame
    Eigen::Vector3d transform_vector(Eigen::Vector3d target_position) {
        Eigen::Vector3d target_position_w;
        target_position_w(0) = target_position.x() + odometry_.x();
        target_position_w(1) = target_position.y() + odometry_.y();
        target_position_w(2) = target_position.z() + odometry_.z();
        return target_position_w;
    }

    // Divide target vector into unit vectors, then transform that based on world frame. Publish it as trajectory message. 
    // time_from_star_ns: When we divide target_vector into parts, each parts will take time based on this parameter
    //                    If it gets bigger, there will be lag. If it gets smaller, there will be crashed
    // unit_number: Number of divided parts for one trajectory. It will arrange based on uniform_distance
    trajectory_msgs::MultiDOFJointTrajectoryPtr createTrajectory(Eigen::Vector3d target_vector) {
        int unit_number;
        Eigen::Vector3d unit_target;
        Eigen::Vector3d unit_target_w;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_msgs::MultiDOFJointTrajectoryPtr trajectory_msg(new trajectory_msgs::MultiDOFJointTrajectory);
        
        // Initialize unit_number based on uniform_distance
        // unit_number = UNIT_NUMBER;
        unit_number = uniform_distance_ *2;
        
        // Initialize trajectory_msg 
        trajectory_msg->header.stamp = ros::Time::now();
        trajectory_msg->points.resize(unit_number);
        trajectory_msg->joint_names.push_back("base_link");
        int64_t time_from_start_ns = 0;
        
        // Divide target_vector into units
        unit_target = target_vector * (1.0 / unit_number);
        for (size_t i = 0; i < unit_number; i++) {
            // Transform each unit based on world frame, and control if any anomally trajectory
            unit_target_w = transform_vector(unit_target * (i+1) );
            controlTarget(&unit_target_w);
            
            // Create trajectory point and add it into trajectory_msg
            trajectory_point.position_W = unit_target_w;
            trajectory_point.setFromYaw(0.0);
            trajectory_point.time_from_start_ns = time_from_start_ns;
            // 
            time_from_start_ns += static_cast<int64_t>(0.1 * kNanoSecondsInSecond);
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_msg->points[i]);
        }
        return trajectory_msg;
    }

    void controlTarget(Eigen::Vector3d *target_position_w) {
        // Keep uav safe altitude
        if ((*target_position_w)(2) < SAFE_ALTITUDE) {
            (*target_position_w)(2) = SAFE_ALTITUDE + 0.2;
        }
    }

    // Calculate distance between two point
    double dist(Eigen::Vector3d point_1, geometry_msgs::Point point_2) {
        double result = pow(point_1.x() - point_2.x,2)
                        + pow(point_1.y() - point_2.y,2)
                        + pow(point_1.z() - point_2.z,2);
        result = sqrt(result);
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