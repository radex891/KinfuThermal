/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *camsButton;
    QPushButton *bagfileButton;
    QPushButton *increaseMax;
    QPushButton *decreaseMax;
    QPushButton *increaseMin;
    QPushButton *decreaseMin;
    QTextEdit *maxTemp;
    QTextEdit *minTemp;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->setEnabled(true);
        MainWindow->resize(300, 235);
        MainWindow->setMinimumSize(QSize(300, 235));
        MainWindow->setMaximumSize(QSize(300, 235));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        camsButton = new QPushButton(centralWidget);
        camsButton->setObjectName(QStringLiteral("camsButton"));
        camsButton->setGeometry(QRect(10, 10, 131, 61));
        camsButton->setMinimumSize(QSize(131, 61));
        camsButton->setMaximumSize(QSize(131, 61));
        bagfileButton = new QPushButton(centralWidget);
        bagfileButton->setObjectName(QStringLiteral("bagfileButton"));
        bagfileButton->setGeometry(QRect(150, 10, 141, 61));
        bagfileButton->setMinimumSize(QSize(141, 61));
        bagfileButton->setMaximumSize(QSize(1410000, 10000));
        increaseMax = new QPushButton(centralWidget);
        increaseMax->setObjectName(QStringLiteral("increaseMax"));
        increaseMax->setGeometry(QRect(150, 90, 61, 51));
        increaseMax->setMinimumSize(QSize(61, 51));
        increaseMax->setMaximumSize(QSize(61, 51));
        decreaseMax = new QPushButton(centralWidget);
        decreaseMax->setObjectName(QStringLiteral("decreaseMax"));
        decreaseMax->setGeometry(QRect(230, 90, 61, 51));
        decreaseMax->setMinimumSize(QSize(61, 51));
        decreaseMax->setMaximumSize(QSize(61, 51));
        increaseMin = new QPushButton(centralWidget);
        increaseMin->setObjectName(QStringLiteral("increaseMin"));
        increaseMin->setGeometry(QRect(150, 170, 61, 51));
        increaseMin->setMinimumSize(QSize(61, 51));
        increaseMin->setMaximumSize(QSize(61, 51));
        decreaseMin = new QPushButton(centralWidget);
        decreaseMin->setObjectName(QStringLiteral("decreaseMin"));
        decreaseMin->setGeometry(QRect(230, 170, 61, 51));
        decreaseMin->setMinimumSize(QSize(61, 51));
        decreaseMin->setMaximumSize(QSize(61, 51));
        maxTemp = new QTextEdit(centralWidget);
        maxTemp->setObjectName(QStringLiteral("maxTemp"));
        maxTemp->setGeometry(QRect(10, 80, 131, 71));
        maxTemp->setMinimumSize(QSize(131, 71));
        maxTemp->setMaximumSize(QSize(131, 71));
        minTemp = new QTextEdit(centralWidget);
        minTemp->setObjectName(QStringLiteral("minTemp"));
        minTemp->setGeometry(QRect(10, 160, 131, 71));
        minTemp->setMinimumSize(QSize(131, 71));
        minTemp->setMaximumSize(QSize(131, 71));
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        camsButton->setText(QApplication::translate("MainWindow", "start Cameras", Q_NULLPTR));
        bagfileButton->setText(QApplication::translate("MainWindow", "start recording bag\n"
" bagtime: 0 sec", Q_NULLPTR));
        increaseMax->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        decreaseMax->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        increaseMin->setText(QApplication::translate("MainWindow", "+", Q_NULLPTR));
        decreaseMin->setText(QApplication::translate("MainWindow", "-", Q_NULLPTR));
        maxTemp->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">MaxTemp: 40</p></body></html>", Q_NULLPTR));
        minTemp->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">MinTemp: 10</p></body></html>", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
