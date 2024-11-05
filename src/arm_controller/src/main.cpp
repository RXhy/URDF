#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <math.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate loop_rate(30);  // Higher rate for smoother motion (30 Hz)

    // Define the joint positions for each target pose
    std::vector<std::vector<double>> joint_positions = {
        {0.0, 0.7, 0, 0},
        {0.0, -1.57, 0.0, 0.0},     
        {1.57, -1.57, 0.0, 0.0},      
        {1.57, -1.57, 1.0, 0.0},        
        {0.0, -1.57, 1.57, 0.0},
        {1.57, -0.4, 1.57, 0.0},

        {1.57, 0.70, 0.0, 0.0},
        {1.57, -1.57, 0.0, 0.0},
        {-1.57, -1.57, 0.0, 0.0},
        {-1.57, 0.70, 0.0, 0.0},
        {0.0, 0.1, 0.0, 0.0},

        {0.0, 0.7, 0.0, 0.0},
        {0.0, -0.35, 0.0, 0.0},
        {0.6, -1.2, 0.0, 0.0},
        {0.0, -0.35, 0.0, 0.0},
        {-0.6, -1.2, 0.0, 0.0},

        {-0.7, 0.55, 0.0, 0.0},
        {0.0, -1.4, 0.0, 0.0},
        {0.7, 0.55, 0.0, 0.0},
        {0.55, 0, 0.0, 0.0},
        {-0.55, 0, 0.0, 0.0},

        {-0.5, 0.6, 0.0, 0.0},
        {-0.5, -1.4, 0.0, 0.0},
        {0.5, 0.6, 0.0, 0.0},
        {0.5, -1.4, 0.0, 0.0}
    };

    size_t current_position = 0;  // Start with the first position
    size_t next_position = 1;     // The next target position

    double interpolation_duration = 1.0;  // Duration in seconds for each transition
    double time_step = 0.0;  // Time elapsed in the current transition

    while (ros::ok()) {
        // Calculate how much time has passed
        time_step += 1.0 / 30.0;  // Assuming loop_rate is 30 Hz

        // Interpolate between the current and next joint positions
        std::vector<double> current_pos = joint_positions[current_position];
        std::vector<double> target_pos = joint_positions[next_position];

        std::vector<double> interpolated_position(4);  // 4 joints
        for (size_t i = 0; i < 4; ++i) {
            // Linear interpolation: move from current_pos[i] to target_pos[i]
            interpolated_position[i] = current_pos[i] + (target_pos[i] - current_pos[i]) * (time_step / interpolation_duration);
        }

        // If the time_step exceeds the duration, move to the next position
        if (time_step >= interpolation_duration) {
            time_step = 0.0;  // Reset time_step for the next transition
            current_position = next_position;
            next_position = (next_position + 1) % joint_positions.size();  // Loop back to the first position when we reach the end
        }

        // Create and publish the joint state message
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"hip", "shoulder", "elbow", "wrist"};
        joint_state.position = interpolated_position;
        joint_state_pub.publish(joint_state);

        // Print the current joint state (for debugging)
        ROS_INFO_STREAM("Publishing joint state: "); 

        ros::spinOnce();
        loop_rate.sleep();  // Sleep to maintain the loop rate
    }

    return 0;
}
