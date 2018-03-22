/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <iostream>

#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/repository.h>
#include <mapit/versioning/repositoryfactorystandard.h>
#include <mapit/errorcodes.h>
#include <mapit/logging.h>
#include <boost/program_options.hpp>
#include <mapit/breadthFirstSearch.h>
#include <mapit/time/time.h>

namespace po = boost::program_options;

void
getHistory(std::shared_ptr<mapit::Repository> repo, const std::string& commitID, const mapit::msgs::Commit& commit)
{
    std::string ret;
    mapit::breadthFirstSearchHistory(repo
                                     , commitID
                                     , commit
                                     , [&](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID)
                                       {
                                           ret += "\033[38;5;208mcommit " + commitID + " (<TODO, list REFs here>)\033[0m\n";
                                           if (commit->parentcommitids_size() > 1) {
                                               // this is a merge commit
                                               ret += "Merge:";
                                               for (std::string parentCommitID : commit->parentcommitids()) {
                                                   ret += " " + parentCommitID.substr(0, 7);
                                               }
                                               ret += "\n";
                                           }
                                           ret += "Author: " + commit->author() + "\n";
                                           ret += "Date:   " + mapit::time::to_string_human( mapit::time::from_msg( commit->stamp() ) ) + "\n";
                                           std::string commitMsg = commit->commitmessage();
                                           while ( ! commitMsg.empty() ) {
                                               size_t endOfLine = commitMsg.find("\n") + 1;
                                               std::string line = commitMsg.substr(0, endOfLine);
                                               commitMsg = commitMsg.substr(endOfLine, commitMsg.size());

                                               ret += "\t" + line;
                                           }

                                           ret += "\n";
                                           return true;
                                       }
                                     , [&](std::shared_ptr<mapit::msgs::Commit> commit,  const mapit::CommitId& commitID){return true;}
    );

    std::cout << ret;
}

void
getHistory(std::shared_ptr<mapit::Repository> repo, const std::string& workspaceName, std::shared_ptr<mapit::Workspace> workspace)
{
    getHistory( repo, workspaceName, workspace->getRollingcommit() );
}

void
getProgramOptions(int argc, char *argv[], po::variables_map& vars)
{
    po::options_description programOptionsDesc(  std::string("Usage:")
                                                 + std::string("\n\t") + argv[0] + " <workspace name>"
                                                 + std::string("\n\t") + argv[0] + " <commit id>");

    programOptionsDesc.add_options()
            ("help,h", "print usage")
            ("ref,r", po::value<std::string>(), "Can be either a workspace name or a commit id where to search from")
            ("all,a", po::value<std::string>(), "starts the search at every workspace and ref")
            ;
    po::positional_options_description posOptions;
    posOptions.add("ref",  1);

    mapit::RepositoryFactoryStandard::addProgramOptions(programOptionsDesc);
    try {
        po::store(po::command_line_parser(argc, argv).options(programOptionsDesc).positional(posOptions).run(), vars);
    } catch(...) {
        std::cout << programOptionsDesc << std::endl;
        throw std::invalid_argument("Can't parse program options");
    }
    if(vars.count("help")) {
        std::cout << programOptionsDesc << std::endl;
        throw std::invalid_argument("Stop, and only show the help");
    }
    try {
        po::notify(vars);
    } catch(...) {
        std::cout << programOptionsDesc << std::endl;
        throw std::invalid_argument("Can't contain options");
    }

    if ( ! vars.count("all") && ! vars.count("ref") ) {
        std::cout << programOptionsDesc << std::endl;
        throw std::invalid_argument("Either all or ref need to be set");
    }

    // untill all is supported
    if ( ! vars.count("ref") ) {
        std::cout << programOptionsDesc << std::endl;
        throw std::invalid_argument("Currently ref has to be set");
    }
}

int main(int argc, char *argv[])
{
    mapit_init_logging();

    // get parameter
    po::variables_map vars;
    try {
        getProgramOptions(argc, argv, vars);
    } catch(std::invalid_argument e) {
        log_error("\n" << e.what() << "\n");
        return 1;
    }

    bool all = vars.count("all");
    bool ref = vars.count("ref");

    if (all) {
        log_warn("the parameter all is not yet implemented");
    }

    std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );

    std::shared_ptr<mapit::Workspace> workspace;
    std::shared_ptr<mapit::msgs::Commit> commit;
    workspace = repo->getWorkspace( vars["ref"].as<std::string>() );
    if ( workspace ) {
        getHistory(repo, vars["ref"].as<std::string>(), workspace);
    } else {
        commit = repo->getCommit( vars["ref"].as<std::string>() );
        if ( commit ) {
            getHistory(repo, vars["ref"].as<std::string>(), *commit.get());
        } else {
            std::cout << "given parameter ref: \"" << vars["ref"].as<std::string>() << "\" is neither a workspace nor a commit id" << std::endl;
            return 1;
        }
    }

    return 0;
}
