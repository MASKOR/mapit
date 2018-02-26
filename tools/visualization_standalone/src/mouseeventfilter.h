#ifndef MouseEventFilter_H
#define MouseEventFilter_H

#include <QObject>
#include <QEvent>
#include <QMouseEvent>
#include <QDebug>

class MouseEventFilter : public QObject
{
    Q_OBJECT
public:
    MouseEventFilter(QObject *parent = nullptr)
        :QObject(parent)
    {

    }
    bool eventFilter(QObject *watched, QEvent *event)
    {
        if (event->type() == QEvent::MouseButtonRelease)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            Q_EMIT mouseReleased(mouseEvent->localPos().x(), mouseEvent->localPos().y());
            return false;
        }
        return false;
    }
Q_SIGNALS:
    void mouseReleased(float mouseX, float mouseY);
};

#endif
