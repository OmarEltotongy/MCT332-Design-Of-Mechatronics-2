#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
#include <QDate>
#include <QTime>
#include <QTimer>
#include "label_display.h"
#include <QStringListModel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , localTimeTimer(new QTimer(this))
{
    ui->setupUi(this);

    displayCurrentLocalTime(ui->timeLabel);

    time1Adjust = "00:00";
    time2Adjust = "00:00";
    time3Adjust = "00:00";

    displayCurrentLocalTime(ui->timeLabel);

    connect(
        localTimeTimer,
        SIGNAL(timeout()),
        this,
        SLOT(updateTime())
        );

    connectAllBtns();
    connectAllKnobes();

    localTimeTimer->start(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete localTimeTimer;
    delete history;
}


void MainWindow::connectAllBtns()
{
    connect(
        ui->startButton,
        SIGNAL(clicked(bool)),
        this,
        SLOT(startBtnPressed())
        );

    connect(
        ui->stopButton,
        SIGNAL(clicked(bool)),
        this,
        SLOT(stopBtnPressed())
        );

    connect(
        ui->setMeal1Btn,
        SIGNAL(clicked(bool)),
        this,
        SLOT(meal1BtnPressed())
        );

    connect(
        ui->setMeal2Btn,
        SIGNAL(clicked(bool)),
        this,
        SLOT(meal2BtnPressed())
        );

    connect(
        ui->setMeal3Btn,
        SIGNAL(clicked(bool)),
        this,
        SLOT(meal3BtnPressed())
        );
}

///==================================================
void MainWindow::connectAllKnobes()
{
    connect(
        ui->meal1HoursKnob,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(meal1HoursKnobChanged(int))
        );

    connect(
        ui->meal1MinsKnob,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(meal1MinsKnobChanged(int))
        );

    connect(
        ui->meal2HoursKnob,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(meal2HoursKnobChanged(int))
        );

    connect(
        ui->meal2MinsKnob,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(meal2MinsKnobChanged(int))
        );

    connect(
        ui->meal3HoursKnob,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(meal3HoursKnobChanged(int))
        );

    connect(
        ui->meal3MinsKnob,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(meal3MinsKnobChanged(int))
        );
}




/*************************************************SLOTS*********************************************************/

void MainWindow::updateTime()
{
    displayCurrentLocalTime(ui->timeLabel);
    QNetworkAccessManager *man= new QNetworkAccessManager(this);
    connect(man, &QNetworkAccessManager::finished, this, &MainWindow::modifyStatus);
    const QUrl url = QUrl(myURL);
    QNetworkRequest request(url);
    man->get(request);

}

void MainWindow::modifyStatus(QNetworkReply *reply){

    QByteArray Message_From_Server = reply->readAll();

    QString Searching = "SP";
    QString Found = "FP";
    QString Message = QString::fromStdString(Message_From_Server.toStdString());

    if(!(QString::compare(Searching, Message, Qt::CaseInsensitive))){
        ui->searchingLabel->setText("Active");
        ui->foundFoodLabel->setText("NULL");
    }
    else if (!(QString::compare(Found, Message, Qt::CaseInsensitive))){
        ui->searchingLabel->setText("NULL");
        ui->foundFoodLabel->setText("Active");
    }
    else{
        ui->searchingLabel->setText("NULL");
        ui->foundFoodLabel->setText("NULL");
    }

}

void MainWindow::redundant(QNetworkReply *reply){

}

void MainWindow::startBtnPressed()
{
    QNetworkAccessManager *man= new QNetworkAccessManager(this);
    connect(man, &QNetworkAccessManager::finished, this, &MainWindow::redundant);
    const QUrl url = QUrl(myURL + "start");
    QNetworkRequest request(url);
    man->get(request);

}

void MainWindow::stopBtnPressed()
{
    QNetworkAccessManager *man= new QNetworkAccessManager(this);
    connect(man, &QNetworkAccessManager::finished, this, &MainWindow::redundant);
    const QUrl url = QUrl(myURL + "stop");
    QNetworkRequest request(url);
    man->get(request);

}

void MainWindow::meal1HoursKnobChanged(int value)
{
    time1Adjust = adjustHoursInLabel(value, ui->meal1AdjustLabel, time1Adjust);
}


void MainWindow::meal1MinsKnobChanged(int value)
{
    time1Adjust = adjustMinutesInLabel(value, ui->meal1AdjustLabel, time1Adjust);
}


void MainWindow::meal2HoursKnobChanged(int value)
{
    time2Adjust = adjustHoursInLabel(value, ui->meal2AdjustLabel, time2Adjust);
}


void MainWindow::meal2MinsKnobChanged(int value)
{
    time2Adjust = adjustMinutesInLabel(value, ui->meal2AdjustLabel, time2Adjust);
}


void MainWindow::meal3HoursKnobChanged(int value)
{
    time3Adjust = adjustHoursInLabel(value, ui->meal3AdjustLabel, time3Adjust);
}


void MainWindow::meal3MinsKnobChanged(int value)
{
    time3Adjust = adjustMinutesInLabel(value, ui->meal3AdjustLabel, time3Adjust);
}


void MainWindow::meal1BtnPressed()
{
    ui->meal1TimeLabel->setText(time1Adjust);
    QString Message_For_Client = time1Adjust;
    QNetworkAccessManager *man= new QNetworkAccessManager(this);
    connect(man, &QNetworkAccessManager::finished, this, &MainWindow::redundant);
    const QUrl url = QUrl(myURL + Message_For_Client);
    QNetworkRequest request(url);
    man->get(request);

}

void MainWindow::meal2BtnPressed()
{
    ui->meal2TimeLabel->setText(time2Adjust);
    QString Message_For_Client = time2Adjust;
    QNetworkAccessManager *man= new QNetworkAccessManager(this);
    connect(man, &QNetworkAccessManager::finished, this, &MainWindow::redundant);
    const QUrl url = QUrl(myURL + Message_For_Client);
    QNetworkRequest request(url);
    man->get(request);

}

void MainWindow::meal3BtnPressed()
{
    ui->meal3TimeLabel->setText(time3Adjust);
    QString Message_For_Client = time3Adjust;
    QNetworkAccessManager *man= new QNetworkAccessManager(this);
    connect(man, &QNetworkAccessManager::finished, this, &MainWindow::redundant);
    const QUrl url = QUrl(myURL + Message_For_Client);
    QNetworkRequest request(url);
    man->get(request);

}

//=================================================For Copying===================================================
//void MainWindow::Read_Data_From_Socket(){
//    QTcpSocket *socket = reinterpret_cast<QTcpSocket*>(sender());
//    QString Searching = "SP";
//    QString Found = "FP";

//    QByteArray Message_From_Server = socket->readAll();

//    QString Message = QString::fromStdString(Message_From_Server.toStdString());

//    if(!(QString::compare(Searching, Message, Qt::CaseInsensitive))){
//        ui->searchingLabel->setText("Active");
//        ui->foundFoodLabel->setText("NULL");
//        qDebug() << QString::compare(Searching, Message, Qt::CaseInsensitive);
//    }
//    else if (!(QString::compare(Found, Message, Qt::CaseInsensitive))){
//        ui->searchingLabel->setText("NULL");
//        ui->foundFoodLabel->setText("Active");
//        qDebug() << QString::compare(Searching, Message, Qt::CaseInsensitive);
//    }
//    else{
//        ui->searchingLabel->setText("NULL");
//        ui->foundFoodLabel->setText("NULL");
//        qDebug() << QString::compare(Searching, Message, Qt::CaseInsensitive);
//    }


//}
//==========================================================================================================

