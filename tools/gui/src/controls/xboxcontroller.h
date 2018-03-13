/*******************************************************************************
 *
 * Copyright      2016 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#include <QObject>

#ifdef _WIN32
#include <windows.h>
#include <xinput.h>
#else
    //TODO: Add Linux Support
#endif

class XBoxController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int controllerId READ controllerId WRITE setControllerId NOTIFY controllerIdChanged)
    Q_PROPERTY(bool buttonA READ buttonA NOTIFY buttonAChanged)
    Q_PROPERTY(bool buttonB READ buttonB NOTIFY buttonBChanged)
    Q_PROPERTY(bool buttonX READ buttonX NOTIFY buttonXChanged)
    Q_PROPERTY(bool buttonY READ buttonY NOTIFY buttonYChanged)
    Q_PROPERTY(bool buttonStart READ buttonStart NOTIFY buttonStartChanged)
    Q_PROPERTY(bool buttonBack READ buttonBack NOTIFY buttonBackChanged)
    Q_PROPERTY(bool dpadLeft READ dpadLeft NOTIFY dpadLeftChanged)
    Q_PROPERTY(bool dpadUp READ dpadUp NOTIFY dpadUpChanged)
    Q_PROPERTY(bool dpadRight READ dpadRight NOTIFY dpadRightChanged)
    Q_PROPERTY(bool dpadDown READ dpadDown NOTIFY dpadDownChanged)
    Q_PROPERTY(bool leftShoulder READ leftShoulder NOTIFY leftShoulderChanged)
    Q_PROPERTY(bool rightShoulder READ rightShoulder NOTIFY rightShoulderChanged)
    Q_PROPERTY(bool leftThumb READ leftThumb NOTIFY leftThumbChanged)
    Q_PROPERTY(bool rightThumb READ rightThumb NOTIFY rightThumbChanged)
    Q_PROPERTY(float triggerRight READ triggerRight NOTIFY triggerRightChanged)
    Q_PROPERTY(float triggerLeft READ triggerLeft NOTIFY triggerLeftChanged)
    Q_PROPERTY(float stickLX READ stickLX NOTIFY stickLXChanged)
    Q_PROPERTY(float stickLY READ stickLY NOTIFY stickLYChanged)
    Q_PROPERTY(float stickRX READ stickRX NOTIFY stickRXChanged)
    Q_PROPERTY(float stickRY READ stickRY NOTIFY stickRYChanged)

public:
    XBoxController();

    int controllerId();

    bool buttonA() const;
    bool buttonB() const;
    bool buttonX() const;
    bool buttonY() const;
    bool buttonStart() const;
    bool buttonBack() const;
    bool dpadLeft() const;
    bool dpadUp() const;
    bool dpadRight() const;
    bool dpadDown() const;
    bool leftShoulder() const;
    bool rightShoulder() const;
    bool leftThumb() const;
    bool rightThumb() const;

    float triggerRight() const;
    float triggerLeft() const;
    float stickLX() const;
    float stickLY() const;
    float stickRX() const;
    float stickRY() const;


public Q_SLOTS:
    void update();
    void setControllerId(int controllerId);

Q_SIGNALS:
    void controllerIdChanged(int controllerId);
    void buttonAChanged(bool buttonA);
    void buttonBChanged(bool buttonB);
    void buttonXChanged(bool buttonX);
    void buttonYChanged(bool buttonY);
    void buttonStartChanged(bool buttonStart);
    void buttonBackChanged(bool buttonBack);
    void dpadLeftChanged(bool dpadLeft);
    void dpadUpChanged(bool dpadUp);
    void dpadRightChanged(bool dpadRight);
    void dpadDownChanged(bool dpadDown);
    void leftShoulderChanged(bool leftShoulder);
    void rightShoulderChanged(bool rightShoulder);
    void leftThumbChanged(bool leftThumb);
    void rightThumbChanged(bool rightThumb);
    void triggerRightChanged(float triggerRight);
    void triggerLeftChanged(float triggerLeft);
    void stickLXChanged(float stickLX);
    void stickLYChanged(float stickLY);
    void stickRXChanged(float stickRX);
    void stickRYChanged(float stickRY);

private:
    void setButtonA(bool buttonA);
    void setButtonB(bool buttonB);
    void setButtonX(bool buttonX);
    void setButtonY(bool buttonY);
    void setButtonStart(bool buttonStart);
    void setButtonBack(bool buttonBack);
    void setDpadLeft(bool dpadLeft);
    void setDpadUp(bool dpadUp);
    void setDpadRight(bool dpadRight);
    void setDpadDown(bool dpadDown);
    void setLeftShoulder(bool leftShoulder);
    void setRightShoulder(bool rightShoulder);
    void setLeftThumb(bool leftThumb);
    void setRightThumb(bool rightThumb);
    void setTriggerRight(float triggerRight);
    void setTriggerLeft(float triggerLeft);
    void setStickLX(float stickLX);
    void setStickLY(float stickLY);
    void setStickRX(float stickRX);
    void setStickRY(float stickRY);

    bool initialize();

    int m_controllerId;
    bool m_buttonA;
    bool m_buttonB;
    bool m_buttonX;
    bool m_buttonY;
    bool m_buttonStart;
    bool m_buttonBack;
    bool m_dpadLeft;
    bool m_dpadUp;
    bool m_dpadRight;
    bool m_dpadDown;
    bool m_leftShoulder;
    bool m_rightShoulder;
    bool m_leftThumb;
    bool m_rightThumb;
    float m_triggerRight;
    float m_triggerLeft;
    float m_stickLX;
    float m_stickLY;
    float m_stickRX;
    float m_stickRY;

#ifdef _WIN32
    XINPUT_STATE m_state;
#else
    //TODO: Add Linux Support
#endif
};

#endif
