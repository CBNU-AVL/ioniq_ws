/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include "../include/gui_tool/qnode.hpp"
#include "QObject"

#define CanBus PCAN_USBBUS1
const std::string topic_speed = "speed";

namespace gui_tool {
QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
    init_argv(argv), kp(1), kd(0), ki(0){
    conf_msg.ID = 0x156;
    conf_msg.MSGTYPE = MSGTYPE_STANDARD;
    conf_msg.LEN = 8;
    memset(conf_msg.DATA, 0, sizeof(uint8_t)*8);

    val_msg.ID = 0x157;
    val_msg.MSGTYPE = MSGTYPE_STANDARD;
    val_msg.LEN = 8;
    memset(val_msg.DATA, 0, sizeof(uint8_t)*8);

    stat = CAN_Initialize(CanBus, PCAN_BAUD_500K);
    flag = true;
    speed.set_constant(kp,kd,ki);
    isPidMode = false;
    req_max = 0;
    isSpeed = false;
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui_tool");
    if (!ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle nh("~");
    speed_sub =nh.subscribe(topic_speed, 10, &QNode::speed_msg_callback, this);
	start();
	return true;
}

void QNode::speed_msg_callback(vehicle_msgs::speed::Ptr msg){
    if(!isSpeed)
        isSpeed = true;
    currSpeed = (int)msg->speed;
    Q_EMIT speed_changed(currSpeed);
}

void QNode::run() {
    ros::Rate loop_rate(100);
    uint8_t count = 0;
	while ( ros::ok() ) {
        if(flag){
            conf_msg.DATA[7] = count++;
            stat = CAN_Write(CanBus, &conf_msg);
            flag = false;
        }
        else{
            if(isPidMode && isSpeed){
                double val = speed.get_PID_value(tarSpeed, currSpeed);
                val += 10.23;
                val_msg.DATA[3] = (uint8_t)((uint16_t)(val*100) & 0xff);
                val_msg.DATA[4] = (uint8_t)(((uint16_t)(val*100) & 0xff00)>>8);
            }
            else{
                double reqMax = req_max + 10.23;
                val_msg.DATA[3] = (uint8_t)((uint16_t)(reqMax*100) & 0xff);
                val_msg.DATA[4] = (uint8_t)(((uint16_t)(reqMax*100) & 0xff00)>>8);
            }
            stat = CAN_Write(CanBus, &val_msg);
            flag = true;
        }
        ROS_INFO("%2X\n", conf_msg.DATA[2]);
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::aeb_run(){
    conf_msg.DATA[2] |= 0x10;
    val_msg.DATA[5] = 40;
}
void QNode::aeb_stop(){
    conf_msg.DATA[2] &= 0x0F;
}
void QNode::apm_enable(bool _state){
    if(_state){
        conf_msg.DATA[0] |= 0x01;
    }
    else{
        conf_msg.DATA[0] &= 0x4;
    }
}
void QNode::asm_enable(bool _state){
    if(_state)
        conf_msg.DATA[2] |= 0x02;
    else
        conf_msg.DATA[2] &= 0xf0;
}

void QNode::set_steer_angle(int _val){
    val_msg.DATA[0] = (uint8_t)((_val * 10) & 0xff);
    val_msg.DATA[1] = (uint8_t)(((_val * 10) & 0xff00)>>8);
}
void QNode::set_slevel(int _val){
    conf_msg.DATA[1] = (uint8_t)_val;
}

void QNode::set_req_max(double _val){
    req_max = _val;
}
void QNode::set_disp_speed(int _val){
    val_msg.DATA[2] = (uint8_t)_val;
}
void QNode::set_pid_gain(double* _kp, double* _ki, double* _kd){
    if(_kp)
        kp = *_kp;
    if(_ki)
        ki = *_ki;
    if(_kd)
        kd = *_kd;

    speed.set_constant(kp, ki, kd);

}
void QNode::set_speed_mode(bool _mode){
    isPidMode = _mode;
}

void QNode::set_target_speed(int _tarSpeed){
    tarSpeed = _tarSpeed;
}

}  // namespace gui_tool
