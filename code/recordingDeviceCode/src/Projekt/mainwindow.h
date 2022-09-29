#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QStringList>
#include <QDebug>
#include <QTimer>
#include <QImage>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "/home/termalesubuntu/catkin_ws/devel/include/optris_drivers/Flag.h"

/**
 * @author: Timo Burger
 */

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow{
    Q_OBJECT

private:
    Ui::MainWindow *ui;

    /**
     * @brief rosMaster, rviz, realsense, optris
     * all QProcesses to start rosnodes or rosmaster(if needed)
     */
    QProcess* rvizProc;
    QProcess* camerasProc = nullptr;
    QProcess* rosBagProc = nullptr;

    /**
     * @brief isRecordingBag, isRunningCams, bagTime
     * farther see: void recordBag()
     */
    bool isRecordingBag = false;
    bool isRunningCams = false;
    QTimer* bagTime;
    int bagTimeCounter = 0;

    double minT = 10, maxT = 40;

    /**
     * @brief setup interface(qtGui + rviz)
     * farther see(below), rviz as a node
     */
    void startRviz();

    /**
      * @brief thread to inform user about a optris calibration procedure
      */
    QTimer* flagTimer;
    ros::NodeHandle rosNodeHandle;
    ros::Subscriber flagSub;

    /**
     * @brief callbackFlag
     * @param image
     */
    void callbackFlag(const optris_drivers::FlagConstPtr &flag                      );
    QImage oldImage;

private slots:

    /**
     * @brief start realsense and optris camera
     * images are displayed in rviz
     */
    void startCameras();

    /**
     * @brief recordBag
     * starts/stops recording a bagfile
     *
     * only starts: (isRunningRealsense && isRunningOptris) --> true
     * start: isRecording --> false
     * stops: isRecording --> true
     */
    void recordBag();

    /**
     * @brief set temperatur ranges during runtime
     */
    void increaseMaxTemp();
    void decreaseMaxTemp();
    void increaseMinTemp();
    void decreaseMinTemp();

    /**
      * @brief trigger ros callback to receive optris flag states
      */
    void triggerCallback();
    /**
     * @brief sets the bag recording time in seconds
     */
    void updateBagTime();

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
};

#endif // MAINWINDOW_H
