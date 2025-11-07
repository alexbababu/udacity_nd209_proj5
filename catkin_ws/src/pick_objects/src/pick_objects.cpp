#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h> // Required for checking goal validity via the make_plan service
#include <geometry_msgs/PoseStamped.h>

// Define a client for sending goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(MoveBaseClient& ac, double x, double y, double w, const std::string& description)
{
    // Define the goal structure
    move_base_msgs::MoveBaseGoal goal;

    // Set the frame parameters. 'map' is the fixed frame.
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define position and orientation
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = w; 

    // Send the goal
    ROS_INFO("Sending Goal: %s (x:%.1f, y:%.1f)", description.c_str(), x, y);
    ac.sendGoal(goal);

    // Wait for result indefinitely
    ac.waitForResult();
}


// Check if pick up or drop off are valied coordinates or inside an obstacle
bool check_goal_validity(ros::ServiceClient& client, double x, double y)
{
    nav_msgs::GetPlan srv;

    // Set a starting position for the plan. We use map origin (0,0) as a safe bet for a validity check.
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = 0.0;
    srv.request.start.pose.orientation.w = 1.0;

    // Set the goal position
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = x;
    srv.request.goal.pose.position.y = y;
    srv.request.goal.pose.orientation.w = 1.0; 

    // Tolerance: 0.1m, means the plan must end within this distance of the goal
    srv.request.tolerance = 0.1;

    // Call the service
    if (client.call(srv))
    {
        // Check if the returned plan has any poses. An empty plan means the goal is blocked.
        if (!srv.response.plan.poses.empty())
        {
            ROS_INFO("Goal (x:%.1f, y:%.1f) is valid and reachable.", x, y);
            return true;
        }
        else
        {
            ROS_ERROR("Goal (x:%.1f, y:%.1f) is not reachable or is inside an obsticle.", x, y);
            return false;
        }
    }
    else
    {
        ROS_ERROR("Failed to call the make_plan serviece. Is move_base running?");
        return false;
    }
}


int main(int argc, char** argv)
{
    // Initialize the ROS node with the name "pick_objects"
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle nh; // Need a node handle for the service client

    // Tell the action client to spin a thread by defualt
    MoveBaseClient ac("move_base", true);

    // Create a ServiceClient for the make_plan service to validate goals
    ros::ServiceClient make_plan_client = nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");


    ROS_INFO("Starting 'pick_objects' Node. Waiting for the move_base Action Server...");

    // Wait up to 5 secomds for the server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base Action Server...");
    }

    // Coordinates for PickUp and Drop off
    const double pickup_x = 5.0;
    const double pickup_y = 1.0;
    const double dropoff_x = -5.0;
    const double dropoff_y = -6.0;
    // W-component for simple orientation (1.0 means no rotation from map's x-axis)
    const double goal_w_co = 1.0; 

    // 1. GOAL: PICKUP LOCATION
    ROS_INFO("--- Beginning Mission: Driving to Pickup Location ---");
    
    // Check validity before sending the goal
    if (check_goal_validity(make_plan_client, pickup_x, pickup_y))
    {
        send_goal(ac, pickup_x, pickup_y, goal_w_co, "Pickup Point");

        // Check if the pickup goal was succesfully reached
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("+++ Pickup Point reached. Starting 'item grab' process +++");
            
            // Wait 5 seconds as required
            ROS_INFO("Waiting 5 secomds...");
            ros::Duration(5.0).sleep();
            ROS_INFO("Wait finished. Proceeding to Drop Off.");
            
            
            // 2. GOAL: DROP OFF LOCATION           
            ROS_INFO("--- Driving to Drop Off Location ---");

            // Check validity before sending the second goal
            if (check_goal_validity(make_plan_client, dropoff_x, dropoff_y))
            {
                send_goal(ac, dropoff_x, dropoff_y, goal_w_co, "Drop Off Point");

                // Check the final result
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("+++ Drop Off Point reached. Mission completed! +++");
                } else {
                     ROS_ERROR("Failed to reach the Drop Off Goal. Last state: %s", ac.getState().toString().c_str());
                }
            }
            else 
            {
                ROS_ERROR("Drop Off Goal failed validity check. Aborting mission sequence.");
            }
        }
        else
        {
            // Target 1 failed, aborting
            ROS_ERROR("Failed to reach the first Goal (Pickup Point). Aborting Mission.");
        }
    }
    else 
    {
        ROS_ERROR("Pickup Goal failed validity check. Aborting mission sequence.");
    }
    
    return 0;
}
