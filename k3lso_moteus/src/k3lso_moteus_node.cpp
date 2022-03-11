#include <iostream>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "k3lso_msgs/srv/motors_set_torque.hpp"
#include "k3lso_msgs/srv/motors_test.hpp"


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
    {"thigh_fr_to_knee_fr_j",   "/dev/pcan-pcie_fd/devid=11", 3,  -1.483529864, true}, // Low Leg
    // Front Left
    {"torso_to_abduct_fl_j",    "/dev/pcan-pcie_fd/devid=20", 4,  -0.00, true}, // Hip
    {"abduct_fl_to_thigh_fl_j", "/dev/pcan-pcie_fd/devid=10", 5,  0.92, false}, // Leg
    {"thigh_fl_to_knee_fl_j",   "/dev/pcan-pcie_fd/devid=10", 6,  -1.483529864, false}, // Low Leg ATTN!!!! Changed Devid on ID6 to the same of ID5 and connected the moteus cards via CAN cable 
    // Rear Right
    {"torso_to_abduct_hr_j",    "/dev/pcan-pcie_fd/devid=19", 10, -0.00, false}, // Hip
    {"abduct_hr_to_thigh_hr_j", "/dev/pcan-pcie_fd/devid=15", 11, 1.047197551, true}, // Leg
    {"thigh_hr_to_knee_hr_j",   "/dev/pcan-pcie_fd/devid=17", 12, -1.483529864, true}, // Low Leg
    // Rear Left
    {"torso_to_abduct_hl_j",    "/dev/pcan-pcie_fd/devid=21", 7,  -0.00, false}, // Hip
    {"abduct_hl_to_thigh_hl_j", "/dev/pcan-pcie_fd/devid=14", 8,  1.047197551, false}, // Leg
    {"thigh_hl_to_knee_hl_j",   "/dev/pcan-pcie_fd/devid=16", 9,  -1.483529864, false}  // Low Leg
};

MoteusInterfaceMotorsMap interface_motors_map = {       // Assign CAN to motor_ID
    {"/dev/pcan-pcie_fd/devid=18", {1}}, // Hip FR
    {"/dev/pcan-pcie_fd/devid=12", {2}}, // Leg FR
    {"/dev/pcan-pcie_fd/devid=11", {3}}, // Low Leg FR
    {"/dev/pcan-pcie_fd/devid=20", {4}}, // Hip FL
    {"/dev/pcan-pcie_fd/devid=10", {5,6}}, // Leg FL and Low Leg FL DAISY CHAIN!
    {"/dev/pcan-pcie_fd/devid=19", {10}}, // Hip RR
    {"/dev/pcan-pcie_fd/devid=15", {11}}, // Leg RR
    {"/dev/pcan-pcie_fd/devid=17", {12}}, // Low Leg RR
    {"/dev/pcan-pcie_fd/devid=21", {7}}, // Hip RL
    {"/dev/pcan-pcie_fd/devid=14", {8}}, // Leg RL
    {"/dev/pcan-pcie_fd/devid=16", {9}}  // Low Leg RL
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
        double position = (int)(pos*10000.0)/10000.0; // Round to 4 decimals for easier readability
        if(!motor_info.invert){
            msg.position.push_back((-position)); // -6.28319*pos - motor_info.offset
        }else{
            msg.position.push_back((position)); // 6.28319*pos - motor_info.offset
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
    std::vector<float> act_positions (12);
    std::vector<float> goal_positions (12);
    float act_pos, act_vel, act_tor, position, tolerance, step, sleeptime;
    int count = 0;
    bool done[12];
    bool allDone = false;
    
    step = 0.01;
    sleeptime = 0.01;
    tolerance = 0.01;

    // Set torque true if not true, divide positions by 2*Pi
    for(size_t i=0; i < request->ids.size(); i++){
        auto id = request->ids[i];
        goal_positions[i] = (request->position[i])/6.28319; // Divide by 2*Pi
        if(controller._motors[id]->_torque_ena != true){
            controller._motors[id]->set_torque_ena(true); // Enable torque
        }
    }

    while(!allDone){ // THIS IS NOT SOLVED YET!!!!!!!!!!!!!!!
        sleep(sleeptime); // IS THIS RIGHT??? Time between different positions
        count++;
 
        if(count = 10) // To not get stuck in loop
            break;
 
        for(size_t i=0; i < request->ids.size(); i++){ // for every id
            auto id = request->ids[i];
            controller._motors[id]->get_feedback(act_pos,act_vel,act_tor); // Returns actual position in moteus rotations
            act_pos = (int)(act_pos*10000.0)/10000.0; // Round to 4 decimals
 
            if(abs(act_pos-goal_positions[i]) <= tolerance) // Check if position close enough to goal position
                done[i] = true; continue;
            
            // TODO: Compare actual pos to goal pos to know direction of change
            if(act_pos < goal_positions[i]){ // Check if goal is bigger or smaller than zero to go to the right direction
                position = act_pos + step;
            }else{
                position = act_pos - step;
            }
            controller._motors[id]->set_commands(position); // Set new position with extrastep
        }

        // it is a WORKING!!!!
        if(std::all_of(std::begin(done), std::end(done), [](bool k){ return k; } )) {
            allDone = true;
        }
    }
}
        // Callback for pose subscriber to set positions from th controller to all motors
void subscriber_callback(const std_msgs::msg::Float32MultiArray msg){ 
  for(int i=1; i <= 12; i++){
        auto id = i;
        auto position = msg.data[i];
        // controller._motors[id]->set_torque_ena(true); Need to activate torque before running subscriber
        controller._motors[id]->set_commands(position);
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
    subscriber_pose = node->create_subscription<std_msgs::msg::Float32MultiArray>("/pose",10, &subscriber_callback); // Define what topic and callback to run
    // TIMERS
    timer_joint_states = node->create_wall_timer(20ms, &timer_joint_states_callback);
    timer_freqs = node->create_wall_timer(1s, &timer_freqs_callback);
    // SERVICES
    rclcpp::Service<k3lso_msgs::srv::MotorsSetTorque>::SharedPtr torque_service = 
                        node->create_service<k3lso_msgs::srv::MotorsSetTorque>("/k3lso_moteus/set_torque", &set_torque_callback);
    rclcpp::Service<k3lso_msgs::srv::MotorsTest>::SharedPtr test_service =                       
    			 node->create_service<k3lso_msgs::srv::MotorsTest>("/k3lso_moteus/motors_test", &test_callback);

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
