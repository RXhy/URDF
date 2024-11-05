#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Rate loop_rate(30); 

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

    size_t current_position = 0; 
    size_t next_position = 1;     

    double interpolation_duration = 1.0;  
    double time_step = 0.0;  

    while (ros::ok()) {
        
        time_step += 1.0 / 30.0;  

        
        std::vector<double> current_pos = joint_positions[current_position];
        std::vector<double> target_pos = joint_positions[next_position];

        std::vector<double> interpolated_position(4);  
        for (size_t i = 0; i < 4; ++i) {
            
            interpolated_position[i] = current_pos[i] + (target_pos[i] - current_pos[i]) * (time_step / interpolation_duration);
        }

        if (time_step >= interpolation_duration) {
            time_step = 0.0;  
            current_position = next_position;
            next_position = (next_position + 1) % joint_positions.size();  
        }

        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"hip", "shoulder", "elbow", "wrist"};
        joint_state.position = interpolated_position;
        joint_state_pub.publish(joint_state);

        ROS_INFO_STREAM("Publishing joint state: "); 

        ros::spinOnce();
        loop_rate.sleep();  
    }

    return 0;
}
