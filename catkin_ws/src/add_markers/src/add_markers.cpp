#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath> // For std::sqrt and std::pow

// Pickup location (should match pick_objects.cpp)
const double PICKUP_X = 5.0;
const double PICKUP_Y = 1.0;
// Drop Off location (should match pick_objects.cpp)
const double DROPOFF_X = -5.0; 
const double DROPOFF_Y = -6.0; 

// Radius within which the robot is considered to have reached the goal (in meters)
const double REACH_RADIUS = 0.5;
const double PICKUP_DELAY_SEC = 5.0; // Wait time after pickup

// Marker States
enum MarkerState {
    STATE_AT_PICKUP = 0,             // Marker is shown at pickup, robot has not arrived yet.
    STATE_WAITING_FOR_DROPOFF = 1,   // Marker hidden, waiting for 5 seconds to pass, then proceed.
    STATE_MOVING_TO_DROPOFF = 2,     // Robot is moving to drop off, marker is hidden.
    STATE_AT_DROPOFF = 3             // Marker is shown at drop off, mission complete.
};

// Global state variables
MarkerState current_marker_state = STATE_AT_PICKUP;
ros::Publisher marker_pub;

// Variable to store the time when the pickup action started
ros::Time pickup_start_time; 

/**
 * @brief Publishes a marker with the given action.
 * (Implementation remains the same as it was correct)
 * @param x X-coordinate.
 * @param y Y-coordinate.
 * @param action Marker action (ADD or DELETE).
 */
void publish_marker(double x, double y, int action)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "virtual_object";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = action; 
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Set scale (small cube)
    marker.scale.x = 0.5; 
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration(); 
    marker_pub.publish(marker);
}

bool is_close_enough(double robot_x, double robot_y, double target_x, double target_y)
{
    double distance = std::sqrt(std::pow(robot_x - target_x, 2) + std::pow(robot_y - target_y, 2));
    return distance <= REACH_RADIUS;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double robot_x = msg->pose.pose.position.x;
    double robot_y = msg->pose.pose.position.y;

    // Logic based on the current state
    switch (current_marker_state)
    {
        case STATE_AT_PICKUP:
            // The marker is visible at the pickup point. Check if the robot arrived.
            if (is_close_enough(robot_x, robot_y, PICKUP_X, PICKUP_Y))
            {
                ROS_INFO("Pickup Location reached. Hiding marker and simulating pickup.");
                
                // 1. HIDE THE MARKER (Delete action)
                publish_marker(PICKUP_X, PICKUP_Y, visualization_msgs::Marker::DELETE);
                
                // 2. STORE START TIME for non-blocking wait
                pickup_start_time = ros::Time::now();

                // 3. CHANGE STATE to start the waiting phase
                current_marker_state = STATE_WAITING_FOR_DROPOFF; 
                ROS_INFO("Item is now 'picked up' (marker hidden). Starting 5 second wait...");
            }
            break;

        case STATE_WAITING_FOR_DROPOFF:
            // Non-blocking check if 5 seconds have passed since pickup
            if ((ros::Time::now() - pickup_start_time).toSec() >= PICKUP_DELAY_SEC)
            {
                ROS_INFO("Wait finished. Robot is now moving to Drop Off location.");
                // Change state to allow checking for the drop off location
                current_marker_state = STATE_MOVING_TO_DROPOFF;
            }
            // Marker remains hidden, doing nothing else while waiting
            break;


        case STATE_MOVING_TO_DROPOFF:
            // The marker is hidden. Check if the robot arrived at the drop off point.
            if (is_close_enough(robot_x, robot_y, DROPOFF_X, DROPOFF_Y))
            {
                ROS_INFO("Drop Off Location reached. Placing marker.");
                
                // 4. SHOW THE MARKER AT DROPOFF (Add action)
                publish_marker(DROPOFF_X, DROPOFF_Y, visualization_msgs::Marker::ADD);
                
                // 5. CHANGE STATE
                current_marker_state = STATE_AT_DROPOFF;
                ROS_INFO("Mission Complete. Virtual object has been delivered.");
            }
            break;

        case STATE_AT_DROPOFF:
            // Mission is finished. We continuously publish the marker to keep it visible.
            publish_marker(DROPOFF_X, DROPOFF_Y, visualization_msgs::Marker::ADD);
            break;
    }
}


int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;

    // Create a publisher for the visualization marker
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Create a subscriber for the odometry messages to track the robot's pose
    ros::Subscriber sub_odom = nh.subscribe("/odom", 1, odom_callback);
    
    // Initial Marker Publication: Show the marker at the pickup zone immediately.
    ROS_INFO("Initialising. Marker published at Pickup Point.");
    // Publish Marker 10 times in first second
    for (int i = 0; i < 10; ++i) {
        publish_marker(PICKUP_X, PICKUP_Y, visualization_msgs::Marker::ADD);
        ros::Duration(0.1).sleep(); // wait 100ms
    }

    // Enter the main ROS event loop. This loop processes the odometry messages via the callback.
    ros::spin();

    return 0;
}

