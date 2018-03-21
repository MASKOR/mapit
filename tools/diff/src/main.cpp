/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/msgs/services.pb.h>
#include <mapit/versioning/repository.h>
#include <mapit/versioning/repositoryfactorystandard.h>
#include <mapit/logging.h>
#include <boost/program_options.hpp>
#include <mapit/depthfirstsearch.h>
#include <stdexcept>
#include "diff-match-patch-cpp-stl/diff_match_patch.h"

namespace po = boost::program_options;

void
getProgramOptions(int argc, char *argv[], po::variables_map& vars)
{
    po::options_description programOptionsDesc(std::string("Usage:\n")
                                                 + argv[0] + " <commit>..<commit> ~~[<path>...]\n");
    programOptionsDesc.add_options()
            ("help,h", "print usage")
            ("commit,c", po::value<std::string>(), "the commit IDs to be compared, seperated by \"..\"")
//            ("path,p", po::value<std::string>(),   "the path to be compared")
            ;
    po::positional_options_description posOptions;
    posOptions.add("commit", 1);
//    posOptions.add("path",   2);

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
}

std::map<mapit::Path, std::string>
createTextMap(const std::shared_ptr<mapit::Repository>& repo, const mapit::CommitId& coID)
{
    std::map<mapit::Path, std::string> diffMap; // map that stores path -> text to be diffed
    mapit::depthFirstSearchHistory(  repo
                                   , coID
                                   , depthFirstSearchCommitContinue
                                   , depthFirstSearchCommitStop
                                   , [&](std::shared_ptr<mapit::msgs::Tree> tree, const ObjectReference& ref, const mapit::Path &path)
                                     {
                                         std::string diffText = "Tree:     " + path + "\n";
                                         if (! ref.id().empty())   diffText += "REF-ID:   " + ref.id()   + "\n";
                                         if (! ref.path().empty()) diffText += "REF-Path: " + ref.path() + "\n";
                                         for (auto ref : tree->refs()) {
                                             diffText += "  " + path;
                                             if (path.back() != '/') {
                                                 diffText += "/" + ref.first;
                                             } else {
                                                 diffText +=       ref.first;
                                             }
                                             if (! ref.second.id().empty())   diffText += "\n    -> RED-ID:   " + ref.second.id();
                                             if (! ref.second.path().empty()) diffText += "\n    -> REF-Path: " + ref.second.path();
                                             diffText += "\n";
                                         }
                                         diffMap.insert( std::pair<mapit::Path, std::string>(path, diffText) );

                                         return true;
                                     }
                                   , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                                   , [&](std::shared_ptr<mapit::msgs::Entity> entity, const ObjectReference& ref, const mapit::Path &path)
                                     {
                                         std::ostringstream stamp;
                                         stamp << mapit::time::to_sec( mapit::time::from_msg(entity->stamp()) );
                                                       std::string diffText  = "Entity:   " + path + "\n";
                                         if (! ref.id().empty())   diffText += "REF-ID:   " + ref.id()   + "\n";
                                         if (! ref.path().empty()) diffText += "REF-Path: " + ref.path() + "\n";
                                                                   diffText += "  frame_id: " + entity->frame_id() + "\n";
                                                                   diffText += "  stamp:    " + stamp.str() + "\n";
                                                                   diffText += "  dataID:   " + entity->dataid() + "\n";
                                                                   diffText += "  type:     " + entity->type() + "\n";
                                                                   diffText += "  LUType:   " + mapit::msgs::LayerUsageType_Name( entity->usagetype() ) + "\n";
                                                                   diffText += "\n";

                                         diffMap.insert( std::pair<mapit::Path, std::string>(path, diffText) );

                                         return true;
                                     }
                                   , depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
            );
    return diffMap;
}

std::string
generateDiff(const std::string& path, const std::string& strFrom, const std::string& strTo, const std::string& commitNameFrom, const std::string& commitNameTo)
{
    diff_match_patch<std::string> dmp;
    diff_match_patch<std::string>::Diffs diffs = dmp.diff_main(strFrom, strTo);

    dmp.diff_cleanupSemantic(diffs);

    std::stringstream ss;
    ss << "\e[1mindex " << commitNameFrom.substr(0, 7) << ".." << commitNameTo.substr(0, 7) << "\033[0m" << std::endl;
    ss << "\e[1mpath  " << path << "\033[0m" << std::endl;
    for (diff_match_patch<std::string>::Diff diff : diffs) {
        if (diff.operation == diff_match_patch<std::string>::DELETE) {
//            ss << "\033[1;31m-{" << diff.text << "}\033[0m";
            ss << "\033[1;31m" << diff.text << "\033[0m";
        }
        if (diff.operation == diff_match_patch<std::string>::INSERT) {
//            ss <<"\033[1;32m+{" << diff.text << "}\033[0m";
            ss <<"\033[1;32m" << diff.text << "\033[0m";
        }
        if (diff.operation == diff_match_patch<std::string>::EQUAL) {
            ss << diff.text;
        }
    }
    ss << std::endl;
    return ss.str();
}

int main(int argc, char *argv[])
{
    mapit_init_logging();

    // get parameter
    po::variables_map vars;
    try {
        getProgramOptions(argc, argv, vars);
    } catch(std::exception e) {
        log_error(e.what());
        return 1;
    }

    bool useCommit = vars.count("commit");
//    bool usePath = vars.count("path");
    if ( ! useCommit) {
        log_error("can't display diff without commit");
        return 1;
    }

    std::string commitName = vars["commit"].as<std::string>();
    size_t commitSeperatorPose = commitName.find("..");
    std::string commitNameFrom;
    std::string commitNameTo;
    bool use2Commits = false;
    if (commitSeperatorPose != std::string::npos) {
        commitNameFrom = commitName.substr(0, commitSeperatorPose);
        commitNameTo = commitName.substr(commitSeperatorPose+2, commitName.size());
        if ( ! commitNameFrom.empty() && ! commitNameTo.empty() ) {
            use2Commits = true;
        }
    }
    if ( ! use2Commits) {
        log_error("need 2 commit IDs");
        return 1;
    }
//    std::string pathName;
//    if ( ! usePath) {
//        pathName = vars["path"].as<std::string>();
//    }

    // connect to repo
    std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );

    std::shared_ptr<mapit::msgs::Commit> commitFrom = repo->getCommit(commitNameFrom);
    std::shared_ptr<mapit::msgs::Commit> commitTo = repo->getCommit(commitNameTo);
    if ( ! commitFrom) {
        log_error("Commit \"" << commitNameFrom << "\" dosn't exists");
        return 1;
    }
    if ( ! commitTo) {
        log_error("Commit \"" << commitNameTo << "\" dosn't exists");
        return 1;
    }

    // from search
    std::map<mapit::Path, std::string> textMapFrom = createTextMap(repo, commitNameFrom);
    std::map<mapit::Path, std::string> textMapTo = createTextMap(repo, commitNameTo);

    // generate diff for each path in from
    for (std::pair<mapit::Path, std::string> pathFrom : textMapFrom) {
        std::map<mapit::Path, std::string>::const_iterator pathToItr = textMapTo.find(pathFrom.first);

        std::string diff;
        if (pathToItr == textMapTo.end()) {
            // path is removed
            diff = generateDiff(pathFrom.first, pathFrom.second, "", commitNameFrom, commitNameTo);
        } else {
            diff = generateDiff(pathFrom.first, pathFrom.second, pathToItr->second, commitNameFrom, commitNameTo);
            textMapTo.erase(pathToItr);
        }
        std::cout << diff;
    }

    // generate diff for each left path in to
    for (std::pair<mapit::Path, std::string> pathTo : textMapTo) {
        std::string diff = generateDiff(pathTo.first, "", pathTo.second, commitNameFrom, commitNameTo);
        std::cout << diff;
    }

    return 0;
}
