#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

void publish_marker(ros::Publisher& pub, double x, double y, int action)
{
    // Define the marker message
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; // IMPORTANT: Should match the frame used by move_base
    marker.header.stamp = ros::Time::now();

    // Set the namespace and ID
    marker.ns = "virtual_object";
    marker.id = 0;

    // Set marker type (Cube) and action
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = action; 

    // Set position
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    
    // Set orientation (no rotation)
    marker.pose.orientation.w = 1.0;

    // Set scale (small cube)
    marker.scale.x = 0.5; 
    marker.scale.y = 0.5; 
    marker.scale.z = 0.5;

    // Set color (blue with full opacity)
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration(); 

    // Publish the marker
    pub.publish(marker);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;

    // Create a publisher for the visualization marker
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Rate object to control the loop execution frequency (1 Hz is enough)
    ros::Rate loop_rate(1); 
    

    // Must match the coordinates used in pick_objects.cpp!
    const double PICKUP_X = 5.0;
    const double PICKUP_Y = 1.0;
    const double DROPOFF_X = -5.0;
    const double DROPOFF_Y = -6.0;


    ROS_INFO("Starting Add Markers node for Task 2 (timed sequence).");

    // Give time for Rviz to subscribe to the topic
    ros::Duration(1.0).sleep();

    // --- ALGORITHM START ---

    // 1. Publish the marker at the pickup zone
    ROS_INFO("1. Publishing marker at Pickup zone (x:%.1f, y:%.1f)", PICKUP_X, PICKUP_Y);
    publish_marker(marker_pub, PICKUP_X, PICKUP_Y, visualization_msgs::Marker::ADD);

    // 2. Pause 5 seconds
    ROS_INFO("2. Pausing 5 seconds...");
    ros::Duration(5.0).sleep();

    // 3. Hide the marker (Delete action)
    ROS_INFO("3. Hiding marker (simulating pickup).");
    publish_marker(marker_pub, PICKUP_X, PICKUP_Y, visualization_msgs::Marker::DELETE);

    // 4. Pause 5 seconds
    ROS_INFO("4. Pausing 5 seconds...");
    ros::Duration(5.0).sleep();

    // 5. Publish the marker at the drop off zone
    ROS_INFO("5. Publishing marker at Drop Off zone (x:%.1f, y:%.1f). Sequence complete!", DROPOFF_X, DROPOFF_Y);
    publish_marker(marker_pub, DROPOFF_X, DROPOFF_Y, visualization_msgs::Marker::ADD);
    
    // Keep the node running to ensure the marker stays visible in Rviz
    ros::spin();

    return 0;
}
