/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef QMLENTITYDATA
#define QMLENTITYDATA

#include <QObject>
#include <mapit/msgs/services.pb.h>
#include <mapit/abstractentitydata.h>
#include "qmlworkspace.h"

class QmlWorkspace;
class EntitydataLoader;
class QmlEntitydata : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QmlWorkspace* workspace READ workspace WRITE setWorkspace NOTIFY workspaceChanged)
    Q_PROPERTY(QString path READ path WRITE setPath NOTIFY pathChanged)
    Q_PROPERTY(QJsonObject info READ info NOTIFY infoChanged)
    Q_PROPERTY(bool isLoading READ isLoading NOTIFY isLoadingChanged)
public:
    QmlEntitydata(QObject *parent = nullptr);
    QmlEntitydata(std::shared_ptr<mapit::AbstractEntitydata> &entitydata, QmlWorkspace* workspace, QString path = "");
    ~QmlEntitydata();
    std::shared_ptr<mapit::AbstractEntitydata> getEntitydata();

    QString path() const;

    QmlWorkspace* workspace() const;

    void updateInfo();

    QJsonObject info() const;

    bool isLoading() const;

public Q_SLOTS:
    void setWorkspace(QmlWorkspace* workspace);
    void setPath(QString path);
    void setInfo(QJsonObject info);

Q_SIGNALS:
    void updated();
    void workspaceChanged(QmlWorkspace* workspace);
    void pathChanged(QString path);

    void internalEntitydataChanged(QmlEntitydata *ed);
    void infoChanged(QJsonObject info);

    void isLoadingChanged(bool isLoading);

protected:
    std::shared_ptr<mapit::AbstractEntitydata> m_entitydata;

private:
    QmlWorkspace* m_workspace;
    QString m_path;
    QJsonObject m_info;
    bool m_isLoading;
    EntitydataLoader *m_edLoader;
};

#endif
