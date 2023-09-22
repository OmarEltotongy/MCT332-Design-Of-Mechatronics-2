#ifndef LABEL_DISPLAY_H
#define LABEL_DISPLAY_H

#include <QLabel>
#include <QString>

QString adjustHoursInLabel(int newHour, QLabel* label, QString& prevTime);
QString adjustMinutesInLabel(int newMin, QLabel* label, QString& prevTime);
void displayCurrentLocalTime(QLabel* label);

#endif // LABEL_DISPLAY_H
