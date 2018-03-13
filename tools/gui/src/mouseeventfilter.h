/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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
