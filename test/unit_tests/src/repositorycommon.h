/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef REPOSITORYCOMMON_H
#define REPOSITORYCOMMON_H

#include <QTest>
#include <QMutex>

#include <mapit/versioning/repository.h>

namespace mapit {
class RepositoryServer;
}
class ServerThread;

class RepositoryCommon: public QObject
{
    Q_OBJECT
protected:
    void createTestdata(bool withServer = false, bool withServerLocalyCalculated = false);
    void initTestdata();
    void cleanupTestdata();
    void startServer();
    void stopServer();
private:
    std::shared_ptr<mapit::Repository> m_repo[4];
    std::shared_ptr<mapit::Workspace> m_workspace[4];
    std::function<void()> m_serverCallback;
    std::shared_ptr<ServerThread> m_serverThread[2];
    std::shared_ptr<mapit::Repository> m_networkRepo[2];

    // Start only after stop has finished. Let the last recv() run into it's timeout.
    QMutex m_serverThreadMutex;
};

#endif
