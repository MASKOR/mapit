/*******************************************************************************
 *
 * Copyright      2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

namespace po = boost::program_options;

int
get_program_options(int argc, char *argv[], po::variables_map& vars)
{
    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name>");
    program_options_desc.add_options()
            ("help,h", "print usage")
            ("workspace,w", po::value<std::string>(), "the workspace (formerly checkout) to work with")
            ("recursive,r", po::bool_switch()->default_value(false), "the structure should be displayed recursivly")
            ("path,p", po::value<std::string>(), "the path to start the search with");
    po::positional_options_description pos_options;
    pos_options.add("workspace",  1);

    mapit::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    try {
        po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    } catch(...) {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    if(vars.count("help")) {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    try {
        po::notify(vars);
    } catch(...) {
        std::cout << program_options_desc << std::endl;
        return 1;
    }

    return 0;
}

void display_checkout(std::shared_ptr<mapit::Checkout> co, bool use_recursive = false, std::string search_prefix = "")
{
    int depth = 1;
    ObjectReference nullRef;
    mapit::StatusCode s = mapit::depthFirstSearch(
                co.get(),
                search_prefix.empty() ? co->getRoot() : co->getTree(search_prefix),
                nullRef,
                search_prefix,
                depthFirstSearchAll(Commit),
                depthFirstSearchAll(Commit),
                [&](std::shared_ptr<mapit::msgs::Tree> obj, const ObjectReference& ref, const mapit::Path &path)
                {
                    std::string prefix = "  ";
                    for (int t = 0; t < depth-1; ++t) {
                        prefix +="|  ";
                    }
                    for (int t = std::max(depth-1, 0); t < depth; ++t) {
                        prefix +="|- ";
                    }

                    if ( ! path.empty() ) {
                        log_info(prefix + path.substr(path.find_last_of("/")+1));
                        ++depth;
                    }

                    if (path.empty() || use_recursive) {
                        return true;
                    } else {
                        return false;
                    }
                },
                [&](std::shared_ptr<mapit::msgs::Tree> obj, const ObjectReference& ref, const mapit::Path &path)
                {
                    if ( ! path.empty() ) {
                        --depth;
                    }

                    return true;
                },
                [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                {
                    std::string prefix = "  ";
                    for (int t = 0; t < depth-1; ++t) {
                        prefix +="|  ";
                    }
                    for (int t = depth-1; t < depth; ++t) {
                        prefix +="|- ";
                    }
                    std::string output = prefix + path.substr(path.find_last_of("/")+1);
                    std::string spacing = "";
                    if (output.size() < 60) {
                        for (int i = output.size(); i < 60; ++i) {
                            spacing += " ";
                        }
                    } else {
                        spacing = "\t";
                    }
                    log_info(output + spacing + "(" + obj->type() + ")");

                    return true;
                },
                depthFirstSearchAll(mapit::msgs::Entity)
            );
}

int main(int argc, char *argv[])
{
    upns_init_logging();

    int ret = 0;
    // get parameter
    po::variables_map vars;
    ret = get_program_options(argc, argv, vars);
    if (ret != 0) {
      return ret;
    }
    bool use_recursive = vars["recursive"].as<bool>();
    bool use_workspace = vars.count("workspace");
    bool use_path = vars.count("path");
    if (use_path && ! use_workspace) {
        log_error("can't display path without workspace");
    }

    // connect to repo and workspace
    std::shared_ptr<mapit::Repository> repo( mapit::RepositoryFactoryStandard::openRepository( vars ) );
    if (use_workspace) {
      std::shared_ptr<mapit::Checkout> co = repo->getCheckout( vars["workspace"].as<std::string>() );
      if(co == nullptr)
      {
          log_error("Checkout \"" + vars["workspace"].as<std::string>() + "\" not found");
          std::vector<std::string> possibleCheckouts = repo->listCheckoutNames();
          if (possibleCheckouts.size() == 0) {
              log_info("No possible checkout");
          }
          for ( std::string checkout : possibleCheckouts ) {
              log_info("Possible checkout: " + checkout);
          }
          return 1;
      }

      if (use_path) {
          display_checkout(co, use_recursive, vars["path"].as<std::string>());
      } else {
          display_checkout(co, use_recursive);
      }

    } else {
        std::vector<std::string> co_names = repo->listCheckoutNames();
        for (auto co_name : co_names) {
          log_info("- " + co_name);
          if (use_recursive) {
            std::shared_ptr<mapit::Checkout> co = repo->getCheckout( co_name );
            display_checkout(co, use_recursive);
          }
        }
    }

    return 0;
}
