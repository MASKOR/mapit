/*******************************************************************************
 *
 * Copyright      2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/breadthFirstSearch.h>
#include <mapit/errorcodes.h>
#include <map>
#include <list>

namespace mapit
{
typedef std::pair<CommitId, std::shared_ptr<msgs::Commit>> BreadthCommit;
typedef std::map<time::Stamp, std::list<BreadthCommit>> BreadthMap;
typedef std::pair<time::Stamp, std::list<BreadthCommit>> BreadthMapPair;

StatusCode breadthFirstSearchHistory(  std::shared_ptr<mapit::Repository> repo
                                     , BreadthMap& breadthQueue
                                     , std::function<bool(std::shared_ptr<Commit>, const CommitId&)> beforeCommit
                                     , std::function<bool(std::shared_ptr<Commit>, const CommitId&)> afterCommit
                                    )
{
    bool continueSearchBefore = true;
    while ( ! breadthQueue.empty() && continueSearchBefore ) {
        BreadthMap::iterator currentCommitIt = breadthQueue.begin();
        std::list<BreadthCommit> currentCommits = currentCommitIt->second;
        for (BreadthCommit commit : currentCommits) {

            continueSearchBefore = beforeCommit(commit.second, commit.first);

            for (mapit::CommitId parentCommitID : commit.second->parentcommitids()) {
                if ( ! parentCommitID.empty() ) {
                    // get parent commit
                    std::shared_ptr<msgs::Commit> parentCommit = repo->getCommit(parentCommitID);
                    assert(parentCommit);
                    time::Stamp stamp = time::from_msg( parentCommit->stamp() );
                    BreadthCommit ParentCommitPair(parentCommitID, parentCommit);

                    // if not in map create new entry
                    if ( breadthQueue.find( stamp ) == breadthQueue.end() ) {
                        breadthQueue.insert( BreadthMapPair(stamp, std::list<BreadthCommit>() ) );
                    }
                    // add reference to time
                    breadthQueue[stamp].push_back(ParentCommitPair);
                }
            }
            afterCommit(commit.second, commit.first);
        }
        breadthQueue.erase(currentCommitIt);
    }

    return MAPIT_STATUS_OK;
}

StatusCode breadthFirstSearchHistory(  std::shared_ptr<mapit::Repository> repo
                                     , const CommitId& commitID
                                     , std::shared_ptr<mapit::msgs::Commit> commit
                                     , std::function<bool(std::shared_ptr<Commit>, const CommitId&)> beforeCommit
                                     , std::function<bool(std::shared_ptr<Commit>, const CommitId&)> afterCommit
                                    )
{
    BreadthMap breadthQueue;
    time::Stamp stamp = time::from_msg( commit->stamp() );
    BreadthCommit commitPair(commitID, commit);
    breadthQueue.insert( BreadthMapPair(stamp, std::list<BreadthCommit>() ) );
    breadthQueue[stamp].push_back(commitPair);

    return breadthFirstSearchHistory(repo, breadthQueue, beforeCommit, afterCommit);
}

}
