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

#include "mapit/ui/bindings/qmlrepositoryserver.h"
#include <mapit/versioning/repositorynetworkingfactory.h>
#include "../serverthread.h"
#include <zmq.hpp>
#include <mapit/logging.h>

QmlRepositoryServer::QmlRepositoryServer(QObject *parent)
    :QObject(parent)
    ,m_running(false)
    ,m_port(55512)
{

}

QmlRepositoryServer::~QmlRepositoryServer()
{

}

QmlRepository* QmlRepositoryServer::repository() const
{
    return m_repository;
}

bool QmlRepositoryServer::running() const
{
    return m_running;
}

void QmlRepositoryServer::setRepository(QmlRepository *repository)
{
    if (m_repository == repository)
        return;
    if(m_repository != NULL && m_connection != NULL)
    {
        disconnect(*m_connection);
        m_connection = NULL;
    }
    m_repository = repository;
    if(m_repository)
    {
        std::shared_ptr<QMetaObject::Connection> con( new QMetaObject::Connection(
                                                                  connect(m_repository,
                                                                          &QmlRepository::internalRepositoryChanged,
                                                                          this,
                                                                          &QmlRepositoryServer::reconnect)));
        m_connection = con;
    }
    reconnect();
    Q_EMIT repositoryChanged(m_repository);
}

int QmlRepositoryServer::port() const
{
    return m_port;
}

void QmlRepositoryServer::setPort(int port)
{
    if (m_port == port)
        return;

    m_port = port;
    reconnect();
    Q_EMIT portChanged(m_port);
}

void QmlRepositoryServer::setRunning(bool running)
{
    if (m_thread && (m_running == running))
        return;
    m_running = running;
    reconnect();
    //TODO: running change event should be deferred
    Q_EMIT runningChanged(m_running);
}

void QmlRepositoryServer::reconnect()
{
    if(m_running) {
        try
        {
            //TODO: allow ssh://?
            std::shared_ptr<mapit::RepositoryServer> server( mapit::RepositoryNetworkingFactory::openRepositoryAsServer(m_port, repository()->getRepository().get()) );
            m_thread.reset(new ServerThread(server));
            m_thread->start();
        }
        catch (zmq::error_t err)
        {
            log_error("Server: Could not start: " + err.what());
            setRunning(false);
        }
    }
    else
    {
        if(m_thread)
        {
            m_thread->stop();
            m_thread.reset();
        }
    }
}
