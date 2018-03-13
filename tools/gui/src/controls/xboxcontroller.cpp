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

#include "xboxcontroller.h"
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#include <xinput.h>
#endif

XBoxController::XBoxController()
    :m_controllerId( -1 ),
     m_buttonA( false ),
     m_buttonB( false ),
     m_buttonX( false ),
     m_buttonY( false ),
     m_buttonStart( false ),
     m_buttonBack( false ),
     m_dpadLeft( false ),
     m_dpadUp( false ),
     m_dpadRight( false ),
     m_dpadDown( false ),
     m_leftShoulder( false ),
     m_rightShoulder( false ),
     m_leftThumb( false ),
     m_rightThumb( false ),
     m_triggerRight( 0.f ),
     m_triggerLeft( 0.f ),
     m_stickLX( 0.f ),
     m_stickLY( 0.f ),
     m_stickRX( 0.f ),
     m_stickRY( 0.f )
{
}

void XBoxController::update()
{
    if(m_controllerId == -1)
    {
        if(!initialize())
        {
            // Initialization failed
            return;
        }
    }
    else
    {
#ifdef _WIN32
        memset(&m_state, 0, sizeof(XINPUT_STATE));
#endif
#ifdef _WIN32
        if (XInputGetState(m_controllerId, &m_state) != ERROR_SUCCESS)
#else
        if(false) // TODO
#endif
        {
            std::cout << "Error. Could not read Controller Number: " << m_controllerId << std::endl;
            setControllerId(-1);
        }
    }
#ifdef _WIN32
    setButtonA((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_A) != 0);
    setButtonB((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0);
    setButtonX((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_X) != 0);
    setButtonY((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_Y) != 0);
    setDpadLeft((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) != 0);
    setDpadRight((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) != 0);
    setDpadUp((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP) != 0);
    setDpadDown((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN) != 0);
    setLeftShoulder((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0);
    setRightShoulder((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0);
    setLeftThumb((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB) != 0);
    setRightThumb((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) != 0);
    setButtonBack((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_BACK) != 0);
    setButtonStart((m_state.Gamepad.wButtons & XINPUT_GAMEPAD_START) != 0);
    setTriggerLeft(static_cast<float>(m_state.Gamepad.bLeftTrigger) / 255.f);
    setTriggerRight(static_cast<float>( m_state.Gamepad.bRightTrigger) / 255.f);

    float deadzoneX = 0.25f;
    float deadzoneY = 0.25f;
    float leftStickX = fmaxf(-1, static_cast<float>( m_state.Gamepad.sThumbLX) / 32767.f);
    float leftStickY = fmaxf(-1, static_cast<float>( m_state.Gamepad.sThumbLY) / 32767.f);

    leftStickX = (abs(leftStickX) < deadzoneX ? 0.f : (abs(leftStickX) - deadzoneX) * (leftStickX / abs(leftStickX)));
    leftStickY = (abs(leftStickY) < deadzoneY ? 0.f : (abs(leftStickY) - deadzoneY) * (leftStickY / abs(leftStickY)));

    if (deadzoneX > 0) leftStickX /= 1 - deadzoneX;
    if (deadzoneY > 0) leftStickY /= 1 - deadzoneY;
    setStickLX(leftStickX);
    setStickLY(leftStickY);

    float rightStickX = fmaxf(-1, static_cast<float>( m_state.Gamepad.sThumbRX) / 32767.f);
    float rightStickY = fmaxf(-1, static_cast<float>( m_state.Gamepad.sThumbRY) / 32767.f);

    rightStickX = (abs(rightStickX) < deadzoneX ? 0.f : (abs(rightStickX) - deadzoneX) * (rightStickX / abs(rightStickX)));
    rightStickY = (abs(rightStickY) < deadzoneY ? 0.f : (abs(rightStickY) - deadzoneY) * (rightStickY / abs(rightStickY)));

    if (deadzoneX > 0) rightStickX /= 1 - deadzoneX;
    if (deadzoneY > 0) rightStickY /= 1 - deadzoneY;
    setStickRX(rightStickX);
    setStickRY(rightStickY);
#else
    // TODO
    // Also: Make "deadzone" configurable.
#endif
}

bool XBoxController::initialize()
{
#ifdef _WIN32
    for (DWORD i = 0; i < XUSER_MAX_COUNT && m_controllerId == -1; i++)
    {
        ZeroMemory(&m_state, sizeof(XINPUT_STATE));

        if (XInputGetState(i, &m_state) == ERROR_SUCCESS)
        {
            setControllerId(i);
            std::cout << "Found Xbox 360." << std::endl;
            return true;
        }
    }
#else
    //TODO: Add Linux support
#endif
    return false;
}

int XBoxController::controllerId()
{
    if(m_controllerId == -1)
    {
        initialize();
    }
    return m_controllerId;
}

bool XBoxController::buttonA() const
{
    return m_buttonA;
}

bool XBoxController::buttonB() const
{
    return m_buttonB;
}

bool XBoxController::buttonX() const
{
    return m_buttonX;
}

bool XBoxController::buttonY() const
{
    return m_buttonY;
}

bool XBoxController::buttonStart() const
{
    return m_buttonStart;
}

bool XBoxController::buttonBack() const
{
    return m_buttonBack;
}

bool XBoxController::dpadLeft() const
{
    return m_dpadLeft;
}

bool XBoxController::dpadUp() const
{
    return m_dpadUp;
}

bool XBoxController::dpadRight() const
{
    return m_dpadRight;
}

bool XBoxController::dpadDown() const
{
    return m_dpadDown;
}

bool XBoxController::leftShoulder() const
{
    return m_leftShoulder;
}

bool XBoxController::rightShoulder() const
{
    return m_rightShoulder;
}

bool XBoxController::leftThumb() const
{
    return m_leftThumb;
}

bool XBoxController::rightThumb() const
{
    return m_rightThumb;
}

float XBoxController::triggerRight() const
{
    return m_triggerRight;
}

float XBoxController::triggerLeft() const
{
    return m_triggerLeft;
}

float XBoxController::stickLX() const
{
    return m_stickLX;
}

float XBoxController::stickLY() const
{
    return m_stickLY;
}

float XBoxController::stickRX() const
{
    return m_stickRX;
}

float XBoxController::stickRY() const
{
    return m_stickRY;
}

void XBoxController::setControllerId(int controllerId)
{
    if (m_controllerId == controllerId)
        return;

    m_controllerId = controllerId;
    Q_EMIT controllerIdChanged(controllerId);
}

void XBoxController::setButtonA(bool buttonA)
{
    if (m_buttonA == buttonA)
        return;

    m_buttonA = buttonA;
    Q_EMIT buttonAChanged(buttonA);
}

void XBoxController::setButtonB(bool buttonB)
{
    if (m_buttonB == buttonB)
        return;

    m_buttonB = buttonB;
    Q_EMIT buttonBChanged(buttonB);
}

void XBoxController::setButtonX(bool buttonX)
{
    if (m_buttonX == buttonX)
        return;

    m_buttonX = buttonX;
    Q_EMIT buttonXChanged(buttonX);
}

void XBoxController::setButtonY(bool buttonY)
{
    if (m_buttonY == buttonY)
        return;

    m_buttonX = buttonY;
    Q_EMIT buttonYChanged(buttonY);
}

void XBoxController::setButtonStart(bool buttonStart)
{
    if (m_buttonStart == buttonStart)
        return;

    m_buttonStart = buttonStart;
    Q_EMIT buttonStartChanged(buttonStart);
}

void XBoxController::setButtonBack(bool buttonBack)
{
    if (m_buttonBack == buttonBack)
        return;

    m_buttonBack = buttonBack;
    Q_EMIT buttonBackChanged(buttonBack);
}

void XBoxController::setDpadLeft(bool dpadLeft)
{
    if (m_dpadLeft == dpadLeft)
        return;

    m_dpadLeft = dpadLeft;
    Q_EMIT dpadLeftChanged(dpadLeft);
}

void XBoxController::setDpadUp(bool dpadUp)
{
    if (m_dpadUp == dpadUp)
        return;

    m_dpadUp = dpadUp;
    Q_EMIT dpadUpChanged(dpadUp);
}

void XBoxController::setDpadRight(bool dpadRight)
{
    if (m_dpadRight == dpadRight)
        return;

    m_dpadRight = dpadRight;
    Q_EMIT dpadRightChanged(dpadRight);
}

void XBoxController::setDpadDown(bool dpadDown)
{
    if (m_dpadDown == dpadDown)
        return;

    m_dpadDown = dpadDown;
    Q_EMIT dpadDownChanged(dpadDown);
}

void XBoxController::setLeftShoulder(bool leftShoulder)
{
    if (m_leftShoulder == leftShoulder)
        return;

    m_leftShoulder = leftShoulder;
    Q_EMIT leftShoulderChanged(leftShoulder);
}

void XBoxController::setRightShoulder(bool rightShoulder)
{
    if (m_rightShoulder == rightShoulder)
        return;

    m_rightShoulder = rightShoulder;
    Q_EMIT rightShoulderChanged(rightShoulder);
}

void XBoxController::setLeftThumb(bool leftThumb)
{
    if (m_leftThumb == leftThumb)
        return;

    m_leftThumb = leftThumb;
    Q_EMIT leftThumbChanged(leftThumb);
}

void XBoxController::setRightThumb(bool rightThumb)
{
    if (m_rightThumb == rightThumb)
        return;

    m_rightThumb = rightThumb;
    Q_EMIT rightThumbChanged(rightThumb);
}

void XBoxController::setTriggerRight(float triggerRight)
{
    if (m_triggerRight == triggerRight)
        return;

    m_triggerRight = triggerRight;
    Q_EMIT triggerRightChanged(triggerRight);
}

void XBoxController::setTriggerLeft(float triggerLeft)
{
    if (m_triggerLeft == triggerLeft)
        return;

    m_triggerLeft = triggerLeft;
    Q_EMIT triggerLeftChanged(triggerLeft);
}

void XBoxController::setStickLX(float stickLX)
{
    if (m_stickLX == stickLX)
        return;

    m_stickLX = stickLX;
    Q_EMIT stickLXChanged(stickLX);
}

void XBoxController::setStickLY(float stickLY)
{
    if (m_stickLY == stickLY)
        return;

    m_stickLY = stickLY;
    Q_EMIT stickLYChanged(stickLY);
}

void XBoxController::setStickRX(float stickRX)
{
    if (m_stickRX == stickRX)
        return;

    m_stickRX = stickRX;
    Q_EMIT stickRXChanged(stickRX);
}

void XBoxController::setStickRY(float stickRY)
{
    if (m_stickRY == stickRY)
        return;

    m_stickRY = stickRY;
    Q_EMIT stickRYChanged(stickRY);
}
