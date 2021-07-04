#include <math.h>

#include <ros/ros.h>
#include <Eigen/Core>
#include <std_msgs/UInt16MultiArray.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <vision_msgs/BoundingBox3DArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#define MIN_DIST 100000                                     // Enormous distance for calculate min distance between two UAV
#define SAFE_ALTITUDE 0.5                                   // Safe altitude for each UAV (UAVs can't be below this altitude)
#define UNIT_NUMBER 4                                       // Divide trajectory UNIT_NUMBER value parts to prevent big movement(unit_distance 2, UNIT_NUMBER 4)
#define MAX_PARTITION_FACTOR 5                              // While passing throug an strait, max factor for velocity 
static const int64_t kNanoSecondsInSecond = 1000000000;


// Prediction for movement of the UAV
class Prediction {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle global_nh_;
    std::string namespace_;

    ros::Timer heartbeat;
    ros::Publisher trajectory_pub;
    ros::Publisher authority_pub;
    ros::Publisher goal_direction_pub;
    ros::Subscriber authority_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber goal_direction_sub;
    ros::Subscriber goal_direction_main_sub;
    ros::Subscriber detected_straits_sub;
    ros::Subscriber positions_of_detected_uavs_sub;

    uint16_t id_;
    bool active_;
    bool crashed_;
    bool leader_selection_;
    float goal_factor_;
    float uniform_distance_;
    float unification_factor_;
    int authority_expire_time;
    std::vector<uint16_t> authority_;
    Eigen::Vector3d odometry_;
    Eigen::Vector3d goal_vector_;
    Eigen::Vector3d center_point_;
    Eigen::Vector3d maintenance_vector_;
    Eigen::Vector3d unification_vector_;
    Eigen::Vector3d partition_vector_;
    Eigen::Vector3d avoid_vector_;
    geometry_msgs::Point saved_goal_position_;
    std::vector<vision_msgs::BoundingBox3D> detected_straits_;    


public:
    Prediction(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const ros::NodeHandle& global_nh) {
        // Definition of ros nodehandler
        nh_ = nh;
        private_nh_ = private_nh;
        global_nh_ = global_nh;
        namespace_ = nh_.getNamespace();

        // Definition of Timer
        heartbeat = nh_.createTimer(
            ros::Duration(1.0), 
            &Prediction::heartbeatCallback, this);

        // Definition of subscribers       
        odometry_sub = nh_.subscribe(
            "ground_truth/odometry", 1,
            &Prediction::odometryCallback, this);
        goal_direction_sub = global_nh_.subscribe(
            "goal_direction", 1, 
            &Prediction::goalCallback, this);
        goal_direction_main_sub = global_nh_.subscribe(
            "goal_direction_main", 1, 
            &Prediction::goalMainCallback, this);
        positions_of_detected_uavs_sub = nh_.subscribe(
            "detected_uavs_positions", 1, 
            &Prediction::positionOfDetectedUAVsCallback, this);
        detected_straits_sub = nh_.subscribe(
            "detected_straits", 1,
            &Prediction::detectedStraitsCallback, this);
        authority_sub = global_nh_.subscribe(
            "authority", 1,
            &Prediction::authorityCallback, this);

        // Definition of publishers
        trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        authority_pub = global_nh_.advertise<std_msgs::UInt16MultiArray>(
            "authority", 10);
        goal_direction_pub = global_nh_.advertise<geometry_msgs::PointStamped>(
            "goal_direction", 10);      


        // Defination of other variable
        active_ = false;
        crashed_ = false;
        id_ = namespace_.back() - '0';
        authority_.push_back(id_);
        authority_expire_time = 10;
        goal_vector_ = Eigen::Vector3d::Zero();
        maintenance_vector_ = Eigen::Vector3d::Zero();
        unification_vector_ = Eigen::Vector3d::Zero();
        partition_vector_ = Eigen::Vector3d::Zero();
        initParameters();
    }

private:
    // Setter/Getter Functions ------------------------------------------------------------------------------------------------------------------------------------------
    // Initialize node parameters
    void initParameters() {
        private_nh_.param<bool>("leader_selection", leader_selection_, false);
        private_nh_.param<float>("uniform_distance", uniform_distance_, 3.0);
        private_nh_.param<float>("unification_factor", unification_factor_, 0.14);
    }

    // Set uniform_distance (control min value while changing)
    bool setUniformDistance(float uniform_distance) {
        if ( uniform_distance < 1.5) {
            uniform_distance_ = 1.5;
            return false;
        }
        else {
            uniform_distance_ = uniform_distance;
            return true;
        }
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

    // Set goal_vector related to the UAV base_link from given goal_posiiton
    void setGoalVector(geometry_msgs::Point goal_position, bool world=true) {
        Eigen::Vector3d temp_vector;

        if (world) {
            // Control if odometry is set
            if (odometry_.isZero()) {
                return;
            }

            // Set goal_vector based on UAV frame
            temp_vector(0) = (goal_position.x - odometry_.x());
            temp_vector(1) = (goal_position.y - odometry_.y());
            temp_vector(2) = (goal_position.z - odometry_.z());
        }
        else {
            // Set goal_vector which is already in UAV frame
            temp_vector(0) = goal_position.x;
            temp_vector(1) = goal_position.y;
            temp_vector(2) = goal_position.z;
        }

        // Set goal_factor
        setGoalFactor(temp_vector);    

        // Normalize goal_vector based on uniform distance
        temp_vector.normalize();
        goal_vector_ = temp_vector * uniform_distance_;
    }

    // Set maintenance_vector with parameters
    void setMaintenanceVector(double x, double y, double z) {
        Eigen::Vector3d temp_vector;
        temp_vector(0) = x;
        temp_vector(1) = y;
        temp_vector(2) = z;
        maintenance_vector_ = temp_vector;
    }

    // Set partition_vector
    void setPartitionVector(geometry_msgs::Point strait_position) {
        float temp_factor;

        temp_factor = uniform_distance_ / dist(Eigen::Vector3d::Zero(), strait_position);
        if (temp_factor > MAX_PARTITION_FACTOR) {
            temp_factor = MAX_PARTITION_FACTOR;
        }
        partition_vector_ = temp_factor * goal_vector_;
        partition_vector_(2) = strait_position.z;
    }

    void takeoff(float altitude) {
        float desired_yaw = 0.0;
        Eigen::Vector3d desired_position(odometry_.x(), odometry_.y(), altitude);
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

        trajectory_msg.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
        trajectory_pub.publish(trajectory_msg);
        ros::Duration(2.0).sleep();
    }

    // Calback Functions ------------------------------------------------------------------------------------------------------------------------------------------------
    // Every second call this function as heartbeat
    void heartbeatCallback(const ros::TimerEvent& event) {
        std_msgs::UInt16MultiArray authority_msg;
        if (!leader_selection_) {
            return;
        }

        if (crashed_) {
            authority_ = std::vector<uint16_t>();
            authority_.push_back(id_);
        }
        else {
            authority_expire_time -= 1;
            if (authority_expire_time < 0) {
                authority_expire_time = 10;
                authority_ = std::vector<uint16_t>();
                authority_.push_back(id_);
            }

            authority_msg.data = authority_;
            authority_pub.publish(authority_msg);
        }
    }

    // Update odometry each time
    void authorityCallback(const std_msgs::UInt16MultiArrayConstPtr& authority_msg) {
        // Dicards any messages which its own ID appears in it
        if (std::find(authority_msg->data.begin(), authority_msg->data.end(), id_) != authority_msg->data.end() && authority_msg->data.size() > 1 ) {
            return;
        }

        // If authority that comming from message is higher than own authority, use it as your authority and append id_ end of it
        if (compareAuthority(authority_msg->data) == 1) {
            std::vector<uint16_t> temp_authority = authority_msg->data; 
            temp_authority.push_back(id_);
            authority_ = temp_authority;
        }
    }

    void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
        Eigen::Vector3d temp_odom;
        temp_odom(0) = odometry_msg->pose.pose.position.x;
        temp_odom(1) = odometry_msg->pose.pose.position.y;
        temp_odom(2) = odometry_msg->pose.pose.position.z;
        odometry_ = temp_odom;
        
        if (odometry_.z() < 0.1) {
            crashed_ = true;
        }
        else {
            crashed_ = false;
        }
    }

    // Update goal_vector and goal_factor each time (This one is only for follower)
    void goalCallback(const geometry_msgs::PointStampedConstPtr& goal_direction) {
        if (leader_selection_ && authority_.at(0) != id_) {
            saved_goal_position_ = goal_direction->point;
        }
    }
    
    // Update goal_vector and goal_factor each time (This one is only for leader)
    void goalMainCallback(const geometry_msgs::PointStampedConstPtr& goal_direction_main) {
        geometry_msgs::PointStamped goal_direction;

        if (!leader_selection_) {
            saved_goal_position_ = goal_direction_main->point;
            return;
        }

        if (authority_.at(0) == id_ && !crashed_) {
            saved_goal_position_ = goal_direction_main->point;
            
            goal_direction.header.stamp = ros::Time::now();
            goal_direction.header.frame_id = "world";
            goal_direction.point.x = odometry_.x();
            goal_direction.point.y = odometry_.y();
            goal_direction.point.z = odometry_.z();
            goal_direction_pub.publish(goal_direction);
        }
    }

    // Callback for detecting straits
    void detectedStraitsCallback(const vision_msgs::BoundingBox3DArrayConstPtr& detected_straits) {
        detected_straits_ = detected_straits->boxes;      
    }
  
    // Calculate target_position of UAV itself based on world frame each time other UAVs detected
    void positionOfDetectedUAVsCallback(const geometry_msgs::PoseArrayConstPtr& positions_of_uavs) {
        Eigen::Vector3d target_vector;
        trajectory_msgs::MultiDOFJointTrajectoryPtr trajectory_msg;

        // Control if goal_vector_ is set
        if (odometry_.isZero()) {
            return;
        }
        if (!active_) {
            takeoff(SAFE_ALTITUDE);
            active_ = true;
            ROS_ERROR("[%s] : Active ", namespace_.c_str());
        }

        // Calculate maintenance, partition, and unification vectors
        calculateAvoidVector(*positions_of_uavs);
        if (!calculatePartition()) {
            calculateMaintenance(*positions_of_uavs);    
            calculateUnification(*positions_of_uavs);
            target_vector = maintenance_vector_ + (unification_vector_ * unification_factor_) + avoid_vector_;
        }
        else {
            target_vector = partition_vector_ + avoid_vector_;
        }

        // Convert target_vector into target_position related to the world frame
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


        // Control if UAV detected 2 or more UAVs
        if (positions_of_uavs.poses.size() < 2){
            setMaintenanceVector(0.0, 0.0, 0.0);
            return;
        }

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
        if (!found || authority_.size() == 1) {
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

    bool calculatePartition() {   
        float temp_factor;
        geometry_msgs::Point strait_center_position;
        vision_msgs::BoundingBox3D the_strait;

        // Set goal vector as final destination
        setGoalVector(saved_goal_position_);
        // If there is no strait then return false
        if (detected_straits_.empty()) {
            return false;
        }

        // Choose closest strait if exists. It will return center position of strait. If there is none, return zero 
        the_strait = chooseStrait();
        if (the_strait.center.position.x == 0.0 && 
            the_strait.center.position.y == 0.0 &&
            the_strait.center.position.z == 0.0) {
    
            if (isJustPassedStrait()) {
                setPartitionVector(the_strait.center.position);
                return true;
            }
            return false;
        }

        // If close enough to strait, calculate final vector and increase it while closing the strait
        // Set goal_vector as strait position
        if(isCloseToStrait(the_strait.center.position)) {
            setGoalVector(the_strait.center.position, false);
            setPartitionVector(the_strait.center.position);
            return true;
        }
        
        if (isJustPassedStrait()) {
            setPartitionVector(the_strait.center.position);
            return true;
        }
        return false;
    }
    
    void calculateAvoidVector(geometry_msgs::PoseArray positions_of_uavs) {
        float r;
        float m = 0.2;
        float s = uniform_distance_ / 2;
        float avoid_vector_magnitude;
        float object_r;
        avoid_vector_.setZero();
        
        // Avoid from detected UAVs
        for (geometry_msgs::Pose uav_pose : positions_of_uavs.poses) {
            r = dist(Eigen::Vector3d::Zero(), uav_pose.position);
            if (r <= m) {
                avoid_vector_magnitude = -15 * uniform_distance_;
            } else if (m < r && r <= s) {
                avoid_vector_magnitude = -2 * uniform_distance_ * abs((s - r) / (s - m));
            } else {
                avoid_vector_magnitude = 0.0;
            }
            avoid_vector_(0) += uav_pose.position.x * avoid_vector_magnitude;
            avoid_vector_(1) += uav_pose.position.y * avoid_vector_magnitude;
        }
    }


    // Other Functions ------------------------------------------------------------------------------------------------------------------------------------------------
    // Compare authorities and return highest authority
    int compareAuthority(std::vector<uint16_t> authority){
        // Compare root IDs
        if (authority.at(0) > authority_.at(0)) {
            return 1;
        }

        // Compare length if root IDs is equal
        if (authority.at(0) == authority_.at(0) && authority.size() < authority_.size()-1) {
            return 1;
        }

        // Compare last IDs if root IDs equal and sizes equal
        if (authority.at(0) == authority_.at(0) &&
            authority.size() == authority_.size()-1 &&
            authority.at(authority.size() - 1) > authority_.at(authority_.size() - 2)) {
            return 1;
        }

        return 0;
    }
    
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

        if (angle < (M_PI/2) - 0.02) {
            return true;
        }
        return false;
    }

    // Control if UAV is close enough to the goal
    bool isCloseToStrait(geometry_msgs::Point goal_position) {
        // if (dist(Eigen::Vector3d::Zero(), goal_position) < 1.0) {
        if (dist(Eigen::Vector3d::Zero(), goal_position) < 2 * uniform_distance_) {
            return true;
        }
        else {
            return false;
        }
    }

    // Control if UAV just passed the strait
    bool isJustPassedStrait() {
        Eigen::Vector3d fav_vector;
        Eigen::Vector3d max_fav_vector;
        vision_msgs::BoundingBox3D the_strait;

        // Find closest strait to the UAV
        max_fav_vector.setZero();
        for (vision_msgs::BoundingBox3D strait : detected_straits_) {
            fav_vector = findFavVector(strait);
            if (fav_vector.norm() > max_fav_vector.norm()) {
                max_fav_vector = fav_vector;
                the_strait = strait;
            }
        }

        // If closest strait is not in the goal_area, then it is the strait which we passed. 
        if (! inGoalArea(the_strait.center.position) && dist(Eigen::Vector3d::Zero(), the_strait.center.position) < 0.5 * uniform_distance_) {
            return true;
        }
        return false;
    }

    // Choose closest strait for passing through
    vision_msgs::BoundingBox3D chooseStrait() {
        Eigen::Vector3d fav_vector;
        Eigen::Vector3d max_fav_vector;
        vision_msgs::BoundingBox3D the_strait;

        // Find closest strait to the UAV, which is in the goal direction area
        max_fav_vector.setZero();
        for (vision_msgs::BoundingBox3D strait : detected_straits_) {
            fav_vector = findFavVector(strait);
            if (inGoalArea(strait.center.position) && fav_vector.norm() > max_fav_vector.norm()) {
                max_fav_vector = fav_vector;
                the_strait = strait;
            }
        }

        // Return strait center position if you found the appripriate strait
        if (!max_fav_vector.isZero()) {
            return the_strait;
        }
        else {
            the_strait.center.position.x = 0.0;
            the_strait.center.position.y = 0.0;
            the_strait.center.position.z = 0.0;
        }
        return the_strait;
    }

    // Find favorite vecctor into straits for partition
    Eigen::Vector3d findFavVector(vision_msgs::BoundingBox3D strait) {
        float width;
        float distance;
        double fav_norm;
        Eigen::Vector3d fav_vector;

        // Find width of the straits and distance to it's center
        width = (abs(strait.size.y) + abs(strait.size.x)) * 2;
        distance = (float) dist(Eigen::Vector3d::Zero(), strait.center.position);

        // Calculate direction and magnitude of the favorite vector
        fav_norm = width / pow(distance, 2);
        fav_vector(0) = strait.center.position.x;
        fav_vector(1) = strait.center.position.y;
        fav_vector(2) = strait.center.position.z;
        // fav_vector = fav_vector.normalized() * uniform_distance_;
        fav_vector = fav_vector.normalized() * fav_norm;
        return fav_vector;
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
        unit_number = uniform_distance_ * 2;
        
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

    // Keep UAV always safe altitude
    void controlTarget(Eigen::Vector3d *target_position_w) {
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
    ros::NodeHandle global_nh("/swarm");

    Prediction prediction_node(nh, private_nh, global_nh);
    ros::spin();
    return 0;
}