/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *                2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
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

#include "mapit/ui/bindings/qmlentitydata.h"
#include "entitydataloader.h"

QmlEntitydata::QmlEntitydata(QObject *parent)
    :QObject(parent),
     m_entitydata( NULL ),
     m_workspace( NULL ),
     m_path( "" ),
     m_isLoading( false ),
     m_edLoader(nullptr)
{

}

QmlEntitydata::QmlEntitydata(std::shared_ptr<mapit::AbstractEntitydata> &entitydata, QmlWorkspace *workspace, QString path)
    :m_entitydata(entitydata),
     m_workspace( workspace ),
     m_path( path ),
     m_isLoading( false ),
     m_edLoader(nullptr)
{
    if(m_workspace)
    {
        connect(m_workspace, &QmlWorkspace::internalWorkspaceChanged, this, &QmlEntitydata::setWorkspace);
    }
}

QmlEntitydata::~QmlEntitydata()
{
    if(m_edLoader) delete m_edLoader;
}

std::shared_ptr<mapit::AbstractEntitydata> QmlEntitydata::getEntitydata() { return m_entitydata; }

QString QmlEntitydata::path() const
{
    return m_path;
}

QmlWorkspace *QmlEntitydata::workspace() const
{
    return m_workspace;
}

void QmlEntitydata::updateInfo()
{
    m_info = QJsonObject();
    Q_EMIT infoChanged(m_info);
    if(m_entitydata == nullptr) return;
    m_isLoading = true;
    Q_EMIT isLoadingChanged(m_isLoading);
    if(m_edLoader) delete m_edLoader;
    m_edLoader = new EntitydataLoader(this, m_workspace->getWorkspaceObj(), m_path);
    connect(m_edLoader, &EntitydataLoader::entityInfoLoaded, this, &QmlEntitydata::setInfo);
    m_edLoader->start();
}

QJsonObject QmlEntitydata::info() const
{
    return m_info;
}

bool QmlEntitydata::isLoading() const
{
    return m_isLoading;
}

void QmlEntitydata::setWorkspace(QmlWorkspace *workspace)
{
    bool changed = false;
    if (m_workspace != workspace)
    {
        if(m_workspace)
        {
            disconnect(m_workspace, &QmlWorkspace::internalWorkspaceChanged, this, &QmlEntitydata::setWorkspace);
        }
        m_workspace = workspace;
        if(m_workspace)
        {
            connect(m_workspace, &QmlWorkspace::internalWorkspaceChanged, this, &QmlEntitydata::setWorkspace);
        }
        Q_EMIT workspaceChanged(workspace);
        changed = true;
    }
    if( !m_path.isEmpty() && m_workspace->getWorkspaceObj() )
    {
        m_entitydata = m_workspace->getWorkspaceObj()->getEntitydataReadOnly(m_path.toStdString());
        Q_EMIT internalEntitydataChanged( this );

        changed = true;
    }
    if(changed)
    {
        //Note: this is wrong sequence. updateInfo should be called before internalEntitydataChanged and workspaceChanged (?).
        updateInfo();
        Q_EMIT updated();
    }
}

void QmlEntitydata::setPath(QString path)
{
    if (m_path == path)
        return;

    m_path = path;
    if( m_workspace && m_workspace->getWorkspaceObj() && !m_path.isEmpty() )
    {
        std::shared_ptr<mapit::msgs::Entity> e = m_workspace->getWorkspaceObj()->getEntity(m_path.toStdString());
        if(e)
        {
            m_entitydata = m_workspace->getWorkspaceObj()->getEntitydataReadOnly(m_path.toStdString());
            Q_EMIT internalEntitydataChanged( this );
        }
    }
    updateInfo();
    Q_EMIT pathChanged(path);
    Q_EMIT updated();
}

void QmlEntitydata::setInfo(QJsonObject info)
{
    m_isLoading = false;
    m_info = info;
    Q_EMIT isLoadingChanged(m_isLoading);
    Q_EMIT infoChanged(info);
}
