#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include <QStringListModel>
#include "history.h"
#include <QDebug>
#include <QList>
#include <QMessageBox>
#include <QByteArray>
#include <QDataStream>
#include <QNetworkAccessManager>
#include <QNetworkReply>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateTime();

    void modifyStatus(QNetworkReply *reply);

    void redundant(QNetworkReply *reply);

    void startBtnPressed();

    void stopBtnPressed();

    void meal1HoursKnobChanged(int value);

    void meal1MinsKnobChanged(int value);

    void meal2HoursKnobChanged(int value);

    void meal2MinsKnobChanged(int value);

    void meal3HoursKnobChanged(int value);

    void meal3MinsKnobChanged(int value);

    void meal1BtnPressed();

    void meal2BtnPressed();

    void meal3BtnPressed();

//==========================================================================
private:
    Ui::MainWindow *ui;
    QTimer* localTimeTimer;
    QStringListModel* model;
    History* history;

    QString time1Adjust;
    QString time2Adjust;
    QString time3Adjust;
    QString myURL = "https://google.com/";

    void connectAllBtns();
    void connectAllKnobes();

};

#endif // MAINWINDOW_H
