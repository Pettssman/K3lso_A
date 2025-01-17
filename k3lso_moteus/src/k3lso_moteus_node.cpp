#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "k3lso_msgs/srv/motors_set_torque.hpp"
#include "k3lso_msgs/srv/motors_test.hpp"
#include "k3lso_msgs/srv/bezier_test.hpp"


#include "moteus_pcan/moteus_pcan_controller.h"

using namespace std::chrono_literals;

struct MotorInfo{       // Struct for motorinfo
    std::string joint_name;
    std::string can_interface;
    int can_id;
    double offset;
    bool invert;
};


std::vector<MotorInfo> motors_info = {      // Define motor_info struct
    // Front Right
    {"torso_to_abduct_fr_j",    "/dev/pcan-pcie_fd/devid=18", 1,  -0.00, true}, // Hip
    {"abduct_fr_to_thigh_fr_j", "/dev/pcan-pcie_fd/devid=12", 2,  0.92, true}, // Leg
    {"thigh_fr_to_knee_fr_j",   "/dev/pcan-pcie_fd/devid=12", 3,  -1.483529864, true}, // Low Leg
    // Front Left
    {"torso_to_abduct_fl_j",    "/dev/pcan-pcie_fd/devid=20", 4,  -0.00, true}, // Hip
    {"abduct_fl_to_thigh_fl_j", "/dev/pcan-pcie_fd/devid=10", 6, 0.92, false}, // Leg
    {"thigh_fl_to_knee_fl_j",   "/dev/pcan-pcie_fd/devid=10", 5, -1.483529864, false}, // Low Leg ATTN!!!! Changed Devid on ID6 to the same of ID5 and connected the moteus cards via CAN cable 
    // Rear Right
    {"torso_to_abduct_hr_j",    "/dev/pcan-pcie_fd/devid=17", 12, -0.00, false}, // Hip
    {"abduct_hr_to_thigh_hr_j", "/dev/pcan-pcie_fd/devid=15", 11, 1.047197551, true}, // Leg 1.047197551
    {"thigh_hr_to_knee_hr_j",   "/dev/pcan-pcie_fd/devid=17", 10, -1.483529864, true}, // Low Leg
    // Rear Left
    {"torso_to_abduct_hl_j",    "/dev/pcan-pcie_fd/devid=16", 9,  -0.00, false}, // Hip
    {"abduct_hl_to_thigh_hl_j", "/dev/pcan-pcie_fd/devid=14", 8,  1.047197551, false}, // Leg
    {"thigh_hl_to_knee_hl_j",   "/dev/pcan-pcie_fd/devid=16", 7,  -1.483529864, false}  // Low Leg
};

MoteusInterfaceMotorsMap interface_motors_map = {       // Assign CAN to motor_ID
    {"/dev/pcan-pcie_fd/devid=18", {1}}, // Hip FR
    {"/dev/pcan-pcie_fd/devid=12", {2,3}}, // Leg FR
    //{"/dev/pcan-pcie_fd/devid=11", {3}}, // Low Leg FR
    {"/dev/pcan-pcie_fd/devid=20", {4}}, // Hip FL
    {"/dev/pcan-pcie_fd/devid=10", {5,6}}, // Leg FL and Low Leg FL DAISY CHAIN!
    {"/dev/pcan-pcie_fd/devid=15", {11}}, // Leg RR
    {"/dev/pcan-pcie_fd/devid=17", {10,12}}, // Hip RR and Low Leg RR
    {"/dev/pcan-pcie_fd/devid=14", {8}}, // Leg RL
    {"/dev/pcan-pcie_fd/devid=16", {7,9}}  // Hip RL and Low Leg RL
};

MoteusPcanController controller(interface_motors_map);      // Create object of MoteusPcanController
 
bool running = true;

rclcpp::Node::SharedPtr node;
rclcpp::TimerBase::SharedPtr timer_joint_states;                                        // Create callback timer for Joint_states
rclcpp::TimerBase::SharedPtr timer_freqs;                                               // Create callback timer for freqs
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states;      // Create publisher for Joint_states
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_freqs;           // Create publisher for freq
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_pose;      // Create subscriber node for Pose

void timer_joint_states_callback(){
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->get_clock()->now();
    for(const auto& motor_info: motors_info){
        int motor_id = motor_info.can_id;
        float pos, vel, tor;
        controller._motors[motor_id]->get_feedback(pos, vel, tor);
        //double position = (int)(pos*10000.0)/10000.0; // Round to 4 decimals for easier readability
        //std::cout<<motor_id <<": " << pos<< std::endl;
        if(!motor_info.invert){
            msg.position.push_back((-6.28319*pos) - motor_info.offset); // (-6.28319*pos) - motor_info.offset
        }else{
            msg.position.push_back((6.28319*pos) - motor_info.offset); // position
        }
        msg.velocity.push_back(vel);
        msg.effort.push_back(tor);
        msg.name.push_back(motor_info.joint_name);
    }
    publisher_joint_states->publish(msg);
}

void timer_freqs_callback(){
    std_msgs::msg::Int32MultiArray msg;
    msg.data = controller.get_freqs();
    publisher_freqs->publish(msg);
}

void set_torque_error(std::shared_ptr<k3lso_msgs::srv::MotorsSetTorque::Response> response, uint32_t error, 
                      const std::string& error_str){
    response->error = error;                          
    response->error_str = error_str;
    RCLCPP_ERROR(node->get_logger(), error_str);
}

void set_torque_callback(const std::shared_ptr<k3lso_msgs::srv::MotorsSetTorque::Request> request,
          std::shared_ptr<k3lso_msgs::srv::MotorsSetTorque::Response> response)
{
    bool ids_mode = false;
    if(request->ids.empty() && request->joint_names.empty()){
        set_torque_error(response, 1, "Neither ids nor joint_names were selected.");
        return;
    }
    if(!request->ids.empty() && !request->joint_names.empty()){
        set_torque_error(response, 2, "Only one ids or joint_names can be selected.");
        return;
    }
    if(request->ids.empty()){ // Joint Names mode
        RCLCPP_INFO(node->get_logger(), "Setting torques by Joints Names.");
        if(request->joint_names.size() != request->state.size()){
            set_torque_error(response, 3, "joints_names and states must be the same size.");
            return;
        }
    }else{ // IDs mode
        RCLCPP_INFO(node->get_logger(), "Setting torques by IDs.");
        if(request->ids.size() != request->state.size()){
            set_torque_error(response, 4, "ids and states must be the same size.");
            return;
        }
        ids_mode = true;
    }
    if(ids_mode){
        for(const auto& id: request->ids){
            // FIX!!!
            if(id<1 || id>12){
                set_torque_error(response, 5, "ID not valid." );
                return;
            }
        }
        for(size_t i=0; i<request->ids.size(); i++){
            auto id = request->ids[i];
            auto state = request->state[i];
            controller._motors[id]->set_torque_ena(state);
        }
    }else{
        RCLCPP_INFO(node->get_logger(), "Joint Names mode not implemented yet.");
    }
    response->error = 0;
}

        // Test callback for setting positions to motors with service
void test_callback(const std::shared_ptr<k3lso_msgs::srv::MotorsTest::Request> request,
          std::shared_ptr<k3lso_msgs::srv::MotorsTest::Response> response)
{
	for(size_t i=0; i<request->ids.size(); i++){
       	auto id = request->ids[i];
       	auto position = request->position[i]/6.28319; // Delat på 2*pi
            	//controller._motors[id]->set_torque_ena(true);
            	controller._motors[id]->set_commands(position);
   }
}
        // Callback for pose subscriber to set positions from th controller to all motors
void subcallback(std_msgs::msg::Float32MultiArray::SharedPtr msg){
  for(int i=1; i <= 12; i++){
        auto id = i;
        auto position = msg->data[i-1];
        
        //std::cout<<id <<": " << position<< std::endl;
         //controller._motors[id]->set_torque_ena(true); //Need to activate torque before running subscriber
        controller._motors[id]->set_commands(position);
   }
}

void Bezier_callback(const std::shared_ptr<k3lso_msgs::srv::BezierTest::Request> request,
          std::shared_ptr<k3lso_msgs::srv::BezierTest::Response> response)
{
    controller._motors[1]->set_commands(0);
    //Values motor id 2
    float values_2[] = {0.014857776, 0.005181676, -0.00474404, -0.014735285, -0.02448103, -0.033567059, -0.04149111, -0.047671153, -0.051450441, -0.052159067, -0.049716063, -0.044484123, -0.036925177, -0.027617579, -0.01725928, -0.006651486, 0.003330843, 0.011780346, 0.018287082, 0.023270747, 0.027278421, 0.030902232, 0.034770746, 0.039545015, 0.045916361, 0.05460248, 0.066117294, 0.080063743, 0.095735172, 0.112349022, 0.129088445, 0.145149407, 0.159779391, 0.17230065, 0.182163468, 0.189402263, 0.194396238, 0.197504483, 0.199043092, 0.199276336, 0.198413281, 0.196608692, 0.19395693, 0.189857879, 0.182731373, 0.171014317, 0.15330277, 0.128495227, 0.096127958, 0.057086148, 0.014857776};
    //Values motor id 3
    float values_3[] = {0.007296926, 0.003657911, 0.003935861, 0.007664787, 0.01430073, 0.023251155, 0.033893088, 0.04557696, 0.057615367, 0.069289136, 0.08016221, 0.089998843, 0.098600423, 0.105854938, 0.111774603, 0.116514921, 0.120372793, 0.123761029, 0.127040998, 0.130337575, 0.133662627, 0.136957981, 0.140112798, 0.142967179, 0.14530188, 0.146814368, 0.147111043, 0.145838263, 0.14275007, 0.137736258, 0.130838953, 0.122245375, 0.112259952, 0.101261004, 0.089649613, 0.077813696, 0.066071625, 0.054679817, 0.043870347, 0.033879567, 0.024968151, 0.017433099, 0.011613836, 0.007982773, 0.007059712, 0.00905669, 0.013621111, 0.019478242, 0.023948948, 0.022346539, 0.007296926};
    for(size_t i=0; i<50; i++){
           auto position_2 = values_2[i];
                controller._motors[2]->set_commands(position_2);
            auto position_3 = values_3[i]; 
                controller._motors[3]->set_commands(position_3);
        //controller._motors[3]->set_commands(values_3[i]);n
        usleep(300000);   // Sleep for 1 second
        std::cout<<position_3<< std::endl;
   }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("k3lso_moteus_node");

    if(!controller.is_initialized()){
        RCLCPP_FATAL(node->get_logger(), "Could not initialize Moteus controllers.");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moteus controllers intialized.");

    // PUBLISHERS
    publisher_joint_states = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    publisher_freqs = node->create_publisher<std_msgs::msg::Int32MultiArray>("/moteus/freqs", 1);
    // SUBSCRIBERS
    subscriber_pose = node->create_subscription<std_msgs::msg::Float32MultiArray>("/pose",10,&subcallback); // Define what topic and callback to run
    // TIMERS
    timer_joint_states = node->create_wall_timer(10ms, &timer_joint_states_callback);
    timer_freqs = node->create_wall_timer(1s, &timer_freqs_callback);
    // SERVICES
    rclcpp::Service<k3lso_msgs::srv::MotorsSetTorque>::SharedPtr torque_service = 
                        node->create_service<k3lso_msgs::srv::MotorsSetTorque>("/k3lso_moteus/set_torque", &set_torque_callback);
    rclcpp::Service<k3lso_msgs::srv::MotorsTest>::SharedPtr test_service =                       
    			 node->create_service<k3lso_msgs::srv::MotorsTest>("/k3lso_moteus/motors_test", &test_callback);
    rclcpp::Service<k3lso_msgs::srv::BezierTest>::SharedPtr bezier_service =
                 node->create_service<k3lso_msgs::srv::BezierTest>("/k3lso_moteus/bezier_test", &Bezier_callback);

    RCLCPP_INFO(node->get_logger(), "Node running.");
    controller.start();
    
    while(rclcpp::ok()){
        if(!controller.all_running()){
            RCLCPP_FATAL(node->get_logger(), "Moteus controller stopped working.");
            break;
        }
        rclcpp::spin_some(node);
    }

    return 0; 
}
