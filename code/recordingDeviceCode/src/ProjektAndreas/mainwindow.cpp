#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow){

    ui->setupUi(this);
    this->setWindowFlags(Qt::WindowType::WindowMaximizeButtonHint);
    this->setWindowTitle("Thermal Pointcloud - Recorder for Bagfiles");


    startRviz();

    connect(ui->camsButton, SIGNAL(clicked(bool)), this, SLOT(startCameras()));

    connect(ui->bagfileButton, SIGNAL(clicked(bool)), this, SLOT(recordBag()));

    connect(ui->increaseMax, SIGNAL(clicked(bool)), this, SLOT(increaseMaxTemp()));
    connect(ui->decreaseMax, SIGNAL(clicked(bool)), this, SLOT(decreaseMaxTemp()));

    connect(ui->increaseMin, SIGNAL(clicked(bool)), this, SLOT(increaseMinTemp()));
    connect(ui->decreaseMin, SIGNAL(clicked(bool)), this, SLOT(decreaseMinTemp()));

    flagTimer = new QTimer();
    connect(flagTimer, SIGNAL(timeout()), this, SLOT(triggerCallback()));
}
MainWindow::~MainWindow(){
    delete ui;
}


/********************** SETUP **********************/
void MainWindow::startRviz(){
    QStringList env;
    env.clear();

    env << "/home/termalesubuntu/catkin_ws";
    rvizProc = new QProcess(this);
    rvizProc->setEnvironment(env);
    rvizProc->start("rosrun rviz rviz"); //rviz window size is set in default.rviz file
}

/******************CALLBACK FOR FLAG******************/
void MainWindow::callbackFlag(const optris_drivers::FlagConstPtr &flag){
    if(flag->flag_state == 3){
        ui->bagfileButton->setStyleSheet("background-color: red;");
    } else if(flag->flag_state == 0 && isRecordingBag){
        ui->bagfileButton->setStyleSheet("background-color: green;");
    } else if(flag->flag_state == 0){
        ui->bagfileButton->setStyleSheet("background-color: white;");
    }
}

void MainWindow::triggerCallback(){
    ros::spinOnce();
}
/*****************************************************/

/********************** CAMERAS **********************/
void MainWindow::startCameras(){
    if(!isRunningCams){
        QStringList env;
        env.clear();

        ui->bagfileButton->setStyleSheet("background-color: white;");
        flagSub = rosNodeHandle.subscribe("/optris/flag_state", 100, &MainWindow::callbackFlag, this);
        flagTimer->start();


        env << "/home/termalesubuntu/catkin_ws";
        camerasProc = new QProcess(this);
        camerasProc->setEnvironment(env);
        camerasProc->start("roslaunch /home/termalesubuntu/Dokumente/startCams.launch");

        isRunningCams = true;
        ui->camsButton->setText("cams Running");
    }
}


/********************** BAGFILE **********************/
void MainWindow::recordBag(){
    if(isRunningCams && !isRecordingBag){
        QStringList env;
        env.clear();

        QString recordTopics = "rosbag record /optris/camera_info /optris/image_rect_color /camera/imu /camera/depth/image_rect_raw /camera/depth/camera_info /optris/flag_state   __name:=bagfile";
        QString enableAutoFlag = "rosservice call /optris/auto_flag 1";
        QString forceFlag = "rosservice call /optris/force_flag";

        env << "/home/termalesubuntu/catkin_ws";
        QProcess triggerFlag;
        triggerFlag.setEnvironment(env);
        triggerFlag.execute(forceFlag);
        triggerFlag.waitForFinished();

        QProcess shutterFlag;
        shutterFlag.setEnvironment(env);
        //shutterFlag.execute(enableAutoFlag);
        //shutterFlag.waitForFinished();

        rosBagProc = new QProcess(this);
        rosBagProc->setEnvironment(env);
        rosBagProc->setWorkingDirectory("/home/termalesubuntu/Dokumente/bags");
        rosBagProc->start(recordTopics);

        isRecordingBag = true;
        ui->bagfileButton->setStyleSheet("background-color: green;");

    } else if(isRecordingBag){
        QStringList env;
        env.clear();
        env << "/home/termalesubuntu/catkin_ws";

        QString disableAutoFlag = "rosservice call /optris/auto_flag 0";

        QProcess disableFlag;
        disableFlag.setEnvironment(env);
        //disableFlag.execute(disableAutoFlag);
        //disableFlag.waitForFinished();

        rosBagProc->kill();
        delete rosBagProc;
        killRosbagRecord = new QProcess(this);
        killRosbagRecord->setEnvironment(env);
        killRosbagRecord->start("rosnode kill /bagfile");

        isRecordingBag = false;
        ui->bagfileButton->setStyleSheet("background-color: white;");
    }
}

/******************SET TEMPERATURE RANGES******************/
using namespace std;
void MainWindow::increaseMaxTemp(){
    if(isRunningCams){
        QStringList env;
        env.clear();

        maxT++;
        QString up = QString::number(maxT);
        QString down = QString::number(minT);

        QString increase = "rosservice call /optris/palette 6 1 " + down + " " + up;

        env << "/home/termalesubuntu/catkin_ws";
        QProcess upMax;
        upMax.setEnvironment(env);
        upMax.execute(increase);
        upMax.waitForFinished();

        ui->maxTemp->setText(" \nMaxTemp: " + QString::number(maxT));
    }
}
void MainWindow::decreaseMaxTemp(){
    if(isRunningCams){
        if(maxT-1 != minT){
            QStringList env;
            env.clear();

            maxT--;
            QString up = QString::number(maxT);
            QString down = QString::number(minT);

            QString decrease = "rosservice call optris/palette 6 1 " + down + " " + up;

            env << "/home/termalesubuntu/catkin_ws";
            QProcess downMax;
            downMax.setEnvironment(env);
            downMax.execute(decrease);
            downMax.waitForFinished();

            ui->maxTemp->setText(" \nMaxTemp: " + QString::number(maxT));
        }
    }
}

void MainWindow::increaseMinTemp(){
    if(isRunningCams){
        if(minT+1 != maxT){
            QStringList env;
            env.clear();

            minT++;
            QString up = QString::number(maxT);
            QString down = QString::number(minT);

            QString increase = "rosservice call optris/palette 6 1 " + down + " " + up;

            env << "/home/termalesubuntu/catkin_ws";
            QProcess upMin;
            upMin.setEnvironment(env);
            upMin.execute(increase);
            upMin.waitForFinished();

            ui->minTemp->setText(" \nMinTemp: " + QString::number(minT));
        }
    }
}
void MainWindow::decreaseMinTemp(){
    if(isRunningCams && minT > 0){
        QStringList env;
        env.clear();

        minT--;
        QString up = QString::number(maxT);
        QString down = QString::number(minT);

        QString decrease = "rosservice call optris/palette 6 1 " + up + " " + down;

        env << "/home/termalesubuntu/catkin_ws";
        QProcess downMin;
        downMin.setEnvironment(env);
        downMin.execute(decrease);
        downMin.waitForFinished();

        ui->minTemp->setText(" \nMinTemp: " + QString::number(minT));
    }
}
