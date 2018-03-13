/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef QMLTRANSFORM
#define QMLTRANSFORM

#include <QObject>
#include <QMatrix4x4>
#include "qmlentitydata.h"
#include "qmlcheckout.h"
#include "qmlstamp.h"
#include <upns/layertypes/tflayer/tf2/buffer_core.h>

class QmlTransform : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QmlCheckout* checkout READ checkout WRITE setCheckout NOTIFY checkoutChanged)
    Q_PROPERTY(QString path READ path WRITE setPath NOTIFY pathChanged) // path of entity to determine map/top-level-tree
    Q_PROPERTY(QMatrix4x4 matrix READ matrix NOTIFY matrixChanged)
    Q_PROPERTY(bool mustExist READ mustExist WRITE setMustExist NOTIFY mustExistChanged)
    Q_PROPERTY(bool exists READ exists NOTIFY existsChanged)
    Q_PROPERTY(QString targetFrame READ targetFrame WRITE setTargetFrame NOTIFY targetFrameChanged)
    Q_PROPERTY(QString sourceFrame READ sourceFrame WRITE setSourceFrame NOTIFY sourceFrameChanged)
    Q_PROPERTY(QmlStamp* stamp READ stamp WRITE setStamp NOTIFY stampChanged)

public:
    QmlTransform();

    QMatrix4x4 matrix() const;
    bool mustExist() const;
    bool exists() const;

    QmlCheckout* checkout() const;
    QString path() const;
    QString targetFrame() const;
    QString sourceFrame() const;

    QmlStamp *stamp() const;

public Q_SLOTS:
    void setMustExist(bool mustExist);
    void setCheckout(QmlCheckout* checkout);
    void setPath(QString path);
    void setTargetFrame(QString targetFrame);
    void setSourceFrame(QString sourceFrame);

    void setStamp(QmlStamp *stamp);

Q_SIGNALS:
    void matrixChanged(QMatrix4x4 matrix);
    void mustExistChanged(bool mustExist);
    void existsChanged(bool exists);
    void checkoutChanged(QmlCheckout* checkout);
    void pathChanged(QString path);
    void targetFrameChanged(QString targetFrame);
    void sourceFrameChanged(QString sourceFrame);
    void updated();

    void stampChanged(QmlStamp *stamp);

private Q_SLOTS:
    void emitMatrixChanged();
    void emitExistsChanged();

private:
    bool m_mustExist;
    QmlCheckout* m_checkout;
    QString m_path;
    QString m_targetFrame;
    QString m_sourceFrame;

    ::tf::TransformStamped getTfs(bool *found = nullptr) const;
    QmlStamp *m_stamp;
};

#endif
