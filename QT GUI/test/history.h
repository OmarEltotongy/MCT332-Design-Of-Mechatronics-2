#ifndef HISTORY_H
#define HISTORY_H


#include <QStringListModel>
#include <QListView>
#include <QString>

class History
{
public:
    History(QListView* listView);
    ~History();

    void addToHistory(QString toBeAdded);
    void displayHistory() const;

private:
    QListView* listView;
    QStringListModel* model;
};

#endif // HISTORY_H
