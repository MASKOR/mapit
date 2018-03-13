/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H
#include <QThread>
#include <QMutex>
#include <QMutexLocker>
#include <upns/typedefs.h>
#include <upns/repositoryserver.h>

// Client Calls send() followed by stopServer().
// The server waits MAX_NETWORK_LATENCY ms and tries to hanlde the last message in that time
#define MAX_NETWORK_LATENCY 20

class ServerThread : public QThread
{
    Q_OBJECT
    std::shared_ptr<upns::RepositoryServer> m_srv;
    bool m_isStopped;
    QMutex m_mutex;
public:
    ServerThread(std::shared_ptr<upns::RepositoryServer> server):m_srv(server), m_isStopped(false) {}

    void run() Q_DECL_OVERRIDE
    {
        QMutexLocker l(&m_mutex);
        Q_UNUSED(l);
        while(!m_isStopped)
        {
            m_srv->handleRequest(100);
        }
    }
public slots:
    void stop()
    {
        m_isStopped = true;
        QMutexLocker l(&m_mutex); // finish loop
        m_isStopped = false;
        m_srv->handleRequest(MAX_NETWORK_LATENCY); // there might be something not send/received yet.
    }
};
#endif
