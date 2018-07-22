#ifndef gui_tool_QNODE_HPP_
#define gui_tool_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <PCANBasic.h>
#include <cpid.hpp>
#include <vehicle_msgs/speed.h>

namespace gui_tool {

class QNode : public QThread{
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
    void aeb_run();
    void aeb_stop();
    void apm_enable(bool _state);
    void asm_enable(bool _state);
    void set_slevel(int _val);
    void set_steer_angle(int _val);
    void set_req_max(double _val);
    void set_disp_speed(int _val);
    void set_pid_gain(double* _kp, double* _ki, double* _kd);
    void set_speed_mode(bool _mode);
    void speed_msg_callback(vehicle_msgs::speed::Ptr msg);
    void set_target_speed(int _tarSpeed);

Q_SIGNALS:
    void rosShutdown();
    void speed_changed(int);


private:
	int init_argc;
	char** init_argv;
    TPCANMsg conf_msg, val_msg;
    TPCANStatus stat;
    bool flag;
    CPid speed;
    double kp, kd, ki;
    bool isPidMode;
    int currSpeed, tarSpeed;
    double req_max;
    ros::Subscriber speed_sub;
    bool isSpeed;

};

}

#endif
