#include "mainwindow.h"
#include <QApplication>
#include <QScreen>

/**
 * @author: Timo Burger
 */

QProcess* rosMasterProc;

void startROSMater(){
    QStringList env;
    env.clear();

    env << "/home/termalesubuntu/catkin_ws";
    rosMasterProc = new QProcess();
    rosMasterProc->setEnvironment(env);
    rosMasterProc->start("roscore");
}

int main(int argc, char** argv){
    QApplication a(argc, argv);
    ros::init(argc, argv, "ProjektAndreas");

    startROSMater();

    MainWindow w;
    QRect screenrect = a.primaryScreen()->geometry();
    w.move(screenrect.left(), screenrect.top());

    w.show();
    return a.exec();
}
