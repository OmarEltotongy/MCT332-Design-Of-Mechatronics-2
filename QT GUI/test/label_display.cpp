#include "label_display.h"
#include <QTime>

QString adjustHoursInLabel(int newHour, QLabel* label, QString& prevTime)
{
    QString zero = "";

    if(newHour <= 9)
    {
        zero = "0";
    }
    QString newTime = zero + QString::number(newHour) + prevTime.last(3);
    label->setText(newTime);

    return newTime;
}

QString adjustMinutesInLabel(int newMin, QLabel* label, QString& prevTime)
{
    QString zero = "";

    if(newMin <= 9)
    {
        zero = "0";
    }
    QString newTime = prevTime.first(3) + zero + QString::number(newMin);
    label->setText(newTime);

    return newTime;
}

void displayCurrentLocalTime(QLabel* label)
{
    label->setText(QTime::currentTime().toString().first(5));
}
