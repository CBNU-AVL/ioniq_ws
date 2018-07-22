/**
 * @file /include/gui_tool/main_window.hpp
 *
 * @brief Qt based gui for gui_tool.
 *
 * @date November 2010
 **/
#ifndef gui_tool_MAIN_WINDOW_H
#define gui_tool_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui_tool {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow{
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing
	void closeEvent(QCloseEvent *event); // Overloaded function
    void setCurrSpeed(int _currSpeed);


public Q_SLOTS:
    void on_Btn_AEB_clicked();
    void on_RdoBtn_Apm_Enable_toggled(bool checked);
    void on_HSlider_Steer_Level_sliderMoved(int position);
    void on_SpinBox_Steer_Level_valueChanged(int arg1);
    void on_ChkBox_Apm_Ignore_toggled(bool checked);
    void on_RdoBtn_Asm_Enable_toggled(bool checked);
    void on_HSlider_Req_Max_sliderMoved(int position);
    void on_SpinBox_Req_Max_valueChanged(double arg1);
    void on_HSlider_Steer_Angle_sliderMoved(int position);
    void on_SpinBox_Steer_Angle_valueChanged(int arg1);
    void on_SpinBox_SetDispSpeed_valueChanged(int arg1);
    void on_buttonBox_clicked(QAbstractButton *button);
    void on_ChkBox_PID_Control_toggled(bool checked);
    void on_SpinBox_Tar_Speed_valueChanged(int arg1);
    void on_SpinBox_Tar_Speed_valueChanged(const QString &arg1);
    void on_SpinBox_DGain_valueChanged(double arg1);
    void on_SpinBox_IGain_valueChanged(double arg1);
    void on_SpinBox_PGain_valueChanged(double arg1);

private:
        Ui::MainWindowDesign ui;
        QNode qnode;
        bool isAebRun;
        bool isEnApm;
};

}  // namespace gui_tool

#endif // gui_tool_MAIN_WINDOW_H
