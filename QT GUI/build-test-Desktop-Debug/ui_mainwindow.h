/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDial>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLabel *timeLabel;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLabel *searchingLabel;
    QLabel *label_11;
    QLabel *label_10;
    QLabel *foundFoodLabel;
    QLabel *StatusLabel;
    QLabel *StatusLabel_2;
    QPushButton *startButton;
    QLabel *StatusLabel_3;
    QPushButton *stopButton;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QDial *meal2HoursKnob;
    QDial *meal3HoursKnob;
    QLabel *label;
    QLabel *meal1AdjustLabel;
    QPushButton *setMeal3Btn;
    QLabel *label_3;
    QPushButton *setMeal1Btn;
    QPushButton *setMeal2Btn;
    QDial *meal3MinsKnob;
    QLabel *label_4;
    QLabel *label_2;
    QDial *meal1HoursKnob;
    QDial *meal1MinsKnob;
    QLabel *meal3AdjustLabel;
    QLabel *meal2AdjustLabel;
    QDial *meal2MinsKnob;
    QLabel *label_5;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QLabel *meal2TimeLabel;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *meal1TimeLabel;
    QLabel *meal3TimeLabel;
    QLabel *StatusLabel_4;
    QFrame *line;
    QFrame *line_2;
    QFrame *line_3;
    QFrame *line_4;
    QFrame *frame;
    QFrame *line_5;
    QFrame *line_6;
    QFrame *line_7;
    QFrame *line_8;
    QFrame *line_9;
    QFrame *line_10;
    QFrame *line_15;
    QFrame *line_16;
    QFrame *line_17;
    QFrame *line_18;
    QFrame *line_19;
    QFrame *line_20;
    QFrame *line_21;
    QFrame *line_11;
    QFrame *line_12;
    QFrame *line_13;
    QFrame *line_14;
    QFrame *line_22;
    QFrame *line_23;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1347, 761);
        QFont font;
        font.setFamilies({QString::fromUtf8("Dubai")});
        font.setPointSize(11);
        MainWindow->setFont(font);
        MainWindow->setAutoFillBackground(false);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        timeLabel = new QLabel(centralwidget);
        timeLabel->setObjectName("timeLabel");
        timeLabel->setGeometry(QRect(190, 10, 131, 31));
        QFont font1;
        font1.setFamilies({QString::fromUtf8("Courier")});
        font1.setPointSize(20);
        font1.setBold(true);
        timeLabel->setFont(font1);
        timeLabel->setAlignment(Qt::AlignCenter);
        gridLayoutWidget_3 = new QWidget(centralwidget);
        gridLayoutWidget_3->setObjectName("gridLayoutWidget_3");
        gridLayoutWidget_3->setGeometry(QRect(24, 80, 391, 301));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setObjectName("gridLayout_3");
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        searchingLabel = new QLabel(gridLayoutWidget_3);
        searchingLabel->setObjectName("searchingLabel");
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(searchingLabel->sizePolicy().hasHeightForWidth());
        searchingLabel->setSizePolicy(sizePolicy);
        searchingLabel->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(searchingLabel, 0, 1, 1, 1);

        label_11 = new QLabel(gridLayoutWidget_3);
        label_11->setObjectName("label_11");

        gridLayout_3->addWidget(label_11, 1, 0, 1, 1);

        label_10 = new QLabel(gridLayoutWidget_3);
        label_10->setObjectName("label_10");

        gridLayout_3->addWidget(label_10, 0, 0, 1, 1);

        foundFoodLabel = new QLabel(gridLayoutWidget_3);
        foundFoodLabel->setObjectName("foundFoodLabel");
        sizePolicy.setHeightForWidth(foundFoodLabel->sizePolicy().hasHeightForWidth());
        foundFoodLabel->setSizePolicy(sizePolicy);
        foundFoodLabel->setAlignment(Qt::AlignCenter);

        gridLayout_3->addWidget(foundFoodLabel, 1, 1, 1, 1);

        StatusLabel = new QLabel(centralwidget);
        StatusLabel->setObjectName("StatusLabel");
        StatusLabel->setGeometry(QRect(10, 40, 131, 31));
        StatusLabel->setFont(font1);
        StatusLabel->setAlignment(Qt::AlignCenter);
        StatusLabel_2 = new QLabel(centralwidget);
        StatusLabel_2->setObjectName("StatusLabel_2");
        StatusLabel_2->setGeometry(QRect(10, 10, 201, 31));
        StatusLabel_2->setFont(font1);
        StatusLabel_2->setAlignment(Qt::AlignCenter);
        startButton = new QPushButton(centralwidget);
        startButton->setObjectName("startButton");
        startButton->setGeometry(QRect(300, 570, 111, 41));
        StatusLabel_3 = new QLabel(centralwidget);
        StatusLabel_3->setObjectName("StatusLabel_3");
        StatusLabel_3->setGeometry(QRect(430, 40, 321, 31));
        StatusLabel_3->setFont(font1);
        StatusLabel_3->setAlignment(Qt::AlignCenter);
        stopButton = new QPushButton(centralwidget);
        stopButton->setObjectName("stopButton");
        stopButton->setGeometry(QRect(450, 570, 111, 41));
        gridLayoutWidget = new QWidget(centralwidget);
        gridLayoutWidget->setObjectName("gridLayoutWidget");
        gridLayoutWidget->setGeometry(QRect(440, 80, 851, 340));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(0, 0, 0, 0);
        meal2HoursKnob = new QDial(gridLayoutWidget);
        meal2HoursKnob->setObjectName("meal2HoursKnob");
        meal2HoursKnob->setMaximum(23);

        gridLayout->addWidget(meal2HoursKnob, 2, 0, 1, 1);

        meal3HoursKnob = new QDial(gridLayoutWidget);
        meal3HoursKnob->setObjectName("meal3HoursKnob");
        meal3HoursKnob->setMaximum(23);

        gridLayout->addWidget(meal3HoursKnob, 3, 0, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName("label");
        label->setFont(font);
        label->setTextFormat(Qt::AutoText);
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 1, 2, 1, 1);

        meal1AdjustLabel = new QLabel(gridLayoutWidget);
        meal1AdjustLabel->setObjectName("meal1AdjustLabel");

        gridLayout->addWidget(meal1AdjustLabel, 1, 3, 1, 1);

        setMeal3Btn = new QPushButton(gridLayoutWidget);
        setMeal3Btn->setObjectName("setMeal3Btn");

        gridLayout->addWidget(setMeal3Btn, 3, 4, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName("label_3");
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 3, 2, 1, 1);

        setMeal1Btn = new QPushButton(gridLayoutWidget);
        setMeal1Btn->setObjectName("setMeal1Btn");

        gridLayout->addWidget(setMeal1Btn, 1, 4, 1, 1);

        setMeal2Btn = new QPushButton(gridLayoutWidget);
        setMeal2Btn->setObjectName("setMeal2Btn");

        gridLayout->addWidget(setMeal2Btn, 2, 4, 1, 1);

        meal3MinsKnob = new QDial(gridLayoutWidget);
        meal3MinsKnob->setObjectName("meal3MinsKnob");
        meal3MinsKnob->setMaximum(59);

        gridLayout->addWidget(meal3MinsKnob, 3, 1, 1, 1);

        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName("label_4");
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_4, 0, 0, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName("label_2");
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 2, 2, 1, 1);

        meal1HoursKnob = new QDial(gridLayoutWidget);
        meal1HoursKnob->setObjectName("meal1HoursKnob");
        meal1HoursKnob->setMaximum(23);

        gridLayout->addWidget(meal1HoursKnob, 1, 0, 1, 1);

        meal1MinsKnob = new QDial(gridLayoutWidget);
        meal1MinsKnob->setObjectName("meal1MinsKnob");
        meal1MinsKnob->setMaximum(59);

        gridLayout->addWidget(meal1MinsKnob, 1, 1, 1, 1);

        meal3AdjustLabel = new QLabel(gridLayoutWidget);
        meal3AdjustLabel->setObjectName("meal3AdjustLabel");

        gridLayout->addWidget(meal3AdjustLabel, 3, 3, 1, 1);

        meal2AdjustLabel = new QLabel(gridLayoutWidget);
        meal2AdjustLabel->setObjectName("meal2AdjustLabel");

        gridLayout->addWidget(meal2AdjustLabel, 2, 3, 1, 1);

        meal2MinsKnob = new QDial(gridLayoutWidget);
        meal2MinsKnob->setObjectName("meal2MinsKnob");
        meal2MinsKnob->setMaximum(59);

        gridLayout->addWidget(meal2MinsKnob, 2, 1, 1, 1);

        label_5 = new QLabel(gridLayoutWidget);
        label_5->setObjectName("label_5");
        label_5->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_5, 0, 1, 1, 1);

        gridLayoutWidget_2 = new QWidget(centralwidget);
        gridLayoutWidget_2->setObjectName("gridLayoutWidget_2");
        gridLayoutWidget_2->setGeometry(QRect(1060, 480, 221, 211));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName("gridLayout_2");
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        meal2TimeLabel = new QLabel(gridLayoutWidget_2);
        meal2TimeLabel->setObjectName("meal2TimeLabel");

        gridLayout_2->addWidget(meal2TimeLabel, 1, 1, 1, 1);

        label_6 = new QLabel(gridLayoutWidget_2);
        label_6->setObjectName("label_6");
        label_6->setFont(font);
        label_6->setTextFormat(Qt::AutoText);
        label_6->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_6, 0, 0, 1, 1);

        label_7 = new QLabel(gridLayoutWidget_2);
        label_7->setObjectName("label_7");
        label_7->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_7, 1, 0, 1, 1);

        label_8 = new QLabel(gridLayoutWidget_2);
        label_8->setObjectName("label_8");
        label_8->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(label_8, 2, 0, 1, 1);

        meal1TimeLabel = new QLabel(gridLayoutWidget_2);
        meal1TimeLabel->setObjectName("meal1TimeLabel");

        gridLayout_2->addWidget(meal1TimeLabel, 0, 1, 1, 1);

        meal3TimeLabel = new QLabel(gridLayoutWidget_2);
        meal3TimeLabel->setObjectName("meal3TimeLabel");

        gridLayout_2->addWidget(meal3TimeLabel, 2, 1, 1, 1);

        StatusLabel_4 = new QLabel(centralwidget);
        StatusLabel_4->setObjectName("StatusLabel_4");
        StatusLabel_4->setGeometry(QRect(1010, 450, 321, 31));
        StatusLabel_4->setFont(font1);
        StatusLabel_4->setAlignment(Qt::AlignCenter);
        line = new QFrame(centralwidget);
        line->setObjectName("line");
        line->setGeometry(QRect(420, 30, 20, 581));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName("line_2");
        line_2->setGeometry(QRect(10, 60, 191, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(centralwidget);
        line_3->setObjectName("line_3");
        line_3->setGeometry(QRect(430, 60, 311, 20));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        line_4 = new QFrame(centralwidget);
        line_4->setObjectName("line_4");
        line_4->setGeometry(QRect(260, 440, 321, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        frame = new QFrame(centralwidget);
        frame->setObjectName("frame");
        frame->setGeometry(QRect(10, 10, 291, 31));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        line_5 = new QFrame(centralwidget);
        line_5->setObjectName("line_5");
        line_5->setGeometry(QRect(440, 410, 861, 16));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);
        line_6 = new QFrame(centralwidget);
        line_6->setObjectName("line_6");
        line_6->setGeometry(QRect(440, 70, 861, 16));
        line_6->setFrameShape(QFrame::HLine);
        line_6->setFrameShadow(QFrame::Sunken);
        line_7 = new QFrame(centralwidget);
        line_7->setObjectName("line_7");
        line_7->setGeometry(QRect(430, 200, 851, 16));
        line_7->setFrameShape(QFrame::HLine);
        line_7->setFrameShadow(QFrame::Sunken);
        line_8 = new QFrame(centralwidget);
        line_8->setObjectName("line_8");
        line_8->setGeometry(QRect(430, 310, 851, 16));
        line_8->setFrameShape(QFrame::HLine);
        line_8->setFrameShadow(QFrame::Sunken);
        line_9 = new QFrame(centralwidget);
        line_9->setObjectName("line_9");
        line_9->setGeometry(QRect(-190, 220, 621, 16));
        line_9->setFrameShape(QFrame::HLine);
        line_9->setFrameShadow(QFrame::Sunken);
        line_10 = new QFrame(centralwidget);
        line_10->setObjectName("line_10");
        line_10->setGeometry(QRect(210, 80, 20, 301));
        line_10->setFrameShape(QFrame::VLine);
        line_10->setFrameShadow(QFrame::Sunken);
        line_15 = new QFrame(centralwidget);
        line_15->setObjectName("line_15");
        line_15->setGeometry(QRect(600, 80, 20, 341));
        line_15->setFrameShape(QFrame::VLine);
        line_15->setFrameShadow(QFrame::Sunken);
        line_16 = new QFrame(centralwidget);
        line_16->setObjectName("line_16");
        line_16->setGeometry(QRect(770, 80, 20, 341));
        line_16->setFrameShape(QFrame::VLine);
        line_16->setFrameShadow(QFrame::Sunken);
        line_17 = new QFrame(centralwidget);
        line_17->setObjectName("line_17");
        line_17->setGeometry(QRect(940, 80, 20, 341));
        line_17->setFrameShape(QFrame::VLine);
        line_17->setFrameShadow(QFrame::Sunken);
        line_18 = new QFrame(centralwidget);
        line_18->setObjectName("line_18");
        line_18->setGeometry(QRect(1130, 540, 81, 20));
        line_18->setFrameShape(QFrame::HLine);
        line_18->setFrameShadow(QFrame::Sunken);
        line_19 = new QFrame(centralwidget);
        line_19->setObjectName("line_19");
        line_19->setGeometry(QRect(1130, 610, 81, 20));
        line_19->setFrameShape(QFrame::HLine);
        line_19->setFrameShadow(QFrame::Sunken);
        line_20 = new QFrame(centralwidget);
        line_20->setObjectName("line_20");
        line_20->setGeometry(QRect(1160, 510, 21, 161));
        line_20->setFrameShape(QFrame::VLine);
        line_20->setFrameShadow(QFrame::Sunken);
        line_21 = new QFrame(centralwidget);
        line_21->setObjectName("line_21");
        line_21->setGeometry(QRect(1060, 470, 221, 20));
        line_21->setFrameShape(QFrame::HLine);
        line_21->setFrameShadow(QFrame::Sunken);
        line_11 = new QFrame(centralwidget);
        line_11->setObjectName("line_11");
        line_11->setGeometry(QRect(1050, 440, 20, 251));
        line_11->setFrameShape(QFrame::VLine);
        line_11->setFrameShadow(QFrame::Sunken);
        line_12 = new QFrame(centralwidget);
        line_12->setObjectName("line_12");
        line_12->setGeometry(QRect(1270, 440, 20, 251));
        line_12->setFrameShape(QFrame::VLine);
        line_12->setFrameShadow(QFrame::Sunken);
        line_13 = new QFrame(centralwidget);
        line_13->setObjectName("line_13");
        line_13->setGeometry(QRect(1060, 680, 221, 20));
        line_13->setFrameShape(QFrame::HLine);
        line_13->setFrameShadow(QFrame::Sunken);
        line_14 = new QFrame(centralwidget);
        line_14->setObjectName("line_14");
        line_14->setGeometry(QRect(1060, 430, 221, 20));
        line_14->setFrameShape(QFrame::HLine);
        line_14->setFrameShadow(QFrame::Sunken);
        line_22 = new QFrame(centralwidget);
        line_22->setObjectName("line_22");
        line_22->setGeometry(QRect(1110, 80, 20, 341));
        line_22->setFrameShape(QFrame::VLine);
        line_22->setFrameShadow(QFrame::Sunken);
        line_23 = new QFrame(centralwidget);
        line_23->setObjectName("line_23");
        line_23->setGeometry(QRect(1290, 80, 20, 341));
        line_23->setFrameShape(QFrame::VLine);
        line_23->setFrameShadow(QFrame::Sunken);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1347, 30));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        timeLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        searchingLabel->setText(QCoreApplication::translate("MainWindow", "NULL", nullptr));
        label_11->setText(QCoreApplication::translate("MainWindow", "Found Food Place", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "Searching for Food Place", nullptr));
        foundFoodLabel->setText(QCoreApplication::translate("MainWindow", "NULL", nullptr));
        StatusLabel->setText(QCoreApplication::translate("MainWindow", "STATUS", nullptr));
        StatusLabel_2->setText(QCoreApplication::translate("MainWindow", "Current Time", nullptr));
        startButton->setText(QCoreApplication::translate("MainWindow", "Start", nullptr));
        StatusLabel_3->setText(QCoreApplication::translate("MainWindow", "Meal Times Setting", nullptr));
        stopButton->setText(QCoreApplication::translate("MainWindow", "Stop", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Meal 1 Time:", nullptr));
        meal1AdjustLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        setMeal3Btn->setText(QCoreApplication::translate("MainWindow", "Set Meal 3 Time", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "Meal 3 Time:", nullptr));
        setMeal1Btn->setText(QCoreApplication::translate("MainWindow", "Set Meal 1 Time", nullptr));
        setMeal2Btn->setText(QCoreApplication::translate("MainWindow", "Set Meal 2 Time", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "Hours Knob", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Meal 2 Time:", nullptr));
        meal3AdjustLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        meal2AdjustLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Minutes Knob", nullptr));
        meal2TimeLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Meal 1 Time:", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "Meal 2 Time:", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "Meal 3 Time:", nullptr));
        meal1TimeLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        meal3TimeLabel->setText(QCoreApplication::translate("MainWindow", "00:00", nullptr));
        StatusLabel_4->setText(QCoreApplication::translate("MainWindow", "Meal Times", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
