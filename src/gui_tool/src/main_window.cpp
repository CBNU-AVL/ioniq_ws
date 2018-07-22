#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gui_tool/main_window.hpp"

namespace gui_tool {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    , qnode(argc,argv), isAebRun(false),isEnApm(false)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.png"));
    qnode.init();
    connect(&qnode, SIGNAL(rosShutdown()),this, SLOT(close()));
    connect(&qnode, SIGNAL(speed_changed(int)), this, SLOT(setCurrSpeed(int)));
    ui.HSlider_Steer_Angle->setEnabled(false);
    ui.SpinBox_Steer_Angle->setEnabled(false);
    ui.HSlider_Req_Max->setEnabled(false);
    ui.SpinBox_Req_Max->setEnabled(false);

    ui.Group_PID->setEnabled(false);
    ui.Group_PID_Control->setEnabled(false);
}
MainWindow::~MainWindow() {}


void MainWindow::ReadSettings() {
}

void MainWindow::WriteSettings() {

}
void MainWindow::setCurrSpeed(int _currSpeed){
    ui.LCD_Curr_Speed->display(_currSpeed);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::on_Btn_AEB_clicked()
{
    if(!isAebRun){
        qnode.aeb_run();
        ui.Btn_AEB->setText(QString("Stop"));
        isAebRun = true;
    }
    else{
        qnode.aeb_stop();
        ui.Btn_AEB->setText(QString("AEB Run"));
        isAebRun = false;
    }
}

void MainWindow::on_RdoBtn_Apm_Enable_toggled(bool checked)
{
    qnode.apm_enable(checked);
    ui.HSlider_Steer_Angle->setEnabled(checked);
    ui.SpinBox_Steer_Angle->setEnabled(checked);
}

void MainWindow::on_RdoBtn_Asm_Enable_toggled(bool checked)
{
    qnode.asm_enable(checked);
    ui.HSlider_Req_Max->setEnabled(checked);
    ui.SpinBox_Req_Max->setEnabled(checked);
    ui.Group_PID->setEnabled(checked);
}
void MainWindow::on_ChkBox_Apm_Ignore_toggled(bool checked)
{

}

void MainWindow::on_SpinBox_Steer_Level_valueChanged(int arg1)
{
    ui.HSlider_Steer_Level->setValue(arg1);
    qnode.set_slevel(arg1);
}

void MainWindow::on_HSlider_Steer_Level_sliderMoved(int position)
{
    ui.SpinBox_Steer_Level->setValue(position);
    qnode.set_slevel(position);
}

void MainWindow::on_SpinBox_Steer_Angle_valueChanged(int arg1)
{
    ui.HSlider_Steer_Angle->setValue(arg1);
    qnode.set_steer_angle(arg1);
}

void MainWindow::on_HSlider_Steer_Angle_sliderMoved(int position)
{
    ui.SpinBox_Steer_Angle->setValue(position);
    qnode.set_steer_angle(position);
}

void MainWindow::on_SpinBox_Req_Max_valueChanged(double arg1)
{
    ui.HSlider_Req_Max->setValue((int)(arg1*10));
    qnode.set_req_max(arg1);
}

void MainWindow::on_HSlider_Req_Max_sliderMoved(int position)
{
    double d_val = (double)position / 10.0;
    ui.SpinBox_Req_Max->setValue(d_val);
    qnode.set_req_max(d_val);
}

void MainWindow::on_SpinBox_SetDispSpeed_valueChanged(int arg1)
{
    qnode.set_disp_speed(arg1);
}

void MainWindow::on_buttonBox_clicked(QAbstractButton *button)
{
    close();
}
}

void gui_tool::MainWindow::on_ChkBox_PID_Control_toggled(bool checked)
{
    ui.Group_PID_Control->setEnabled(checked);
    qnode.set_speed_mode(checked);
    ui.Str_Req_Max->setEnabled(!checked);
    ui.SpinBox_Req_Max->setEnabled(!checked);
    ui.HSlider_Req_Max->setEnabled(!checked);
}

void gui_tool::MainWindow::on_SpinBox_PGain_valueChanged(double arg1)
{
    qnode.set_pid_gain(&arg1, NULL, NULL);
}

void gui_tool::MainWindow::on_SpinBox_IGain_valueChanged(double arg1)
{
    qnode.set_pid_gain(NULL, &arg1, NULL);
}

void gui_tool::MainWindow::on_SpinBox_DGain_valueChanged(double arg1)
{
    qnode.set_pid_gain(NULL, NULL, &arg1);
}

void gui_tool::MainWindow::on_SpinBox_Tar_Speed_valueChanged(int arg1)
{
    qnode.set_target_speed(arg1);
}
