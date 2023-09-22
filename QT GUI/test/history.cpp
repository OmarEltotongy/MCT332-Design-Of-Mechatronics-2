#include "history.h"
#include <QTime>

History::History(QListView* listView):
    listView(listView)
    , model(new QStringListModel())
{

}

History::~History()
{
    delete model;
}

void History::addToHistory(QString toBeAdded)
{
    QString curTime = QTime::currentTime().toString();
    QString toBeStored = curTime + ": " + toBeAdded;

    QModelIndex index;
    model->insertRow(model->rowCount());
    index = model->index(model->rowCount()-1);
    model->setData(index, toBeStored);
}

void History::displayHistory() const
{
    listView->setModel(model);
    listView->scrollToBottom();
}
