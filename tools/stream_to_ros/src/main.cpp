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
#include <mapit/depthfirstsearch.h>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include "publishclock.h"
#include <rosgraph_msgs/Clock.h>

#include <mapit/layertypes/pointcloudlayer.h>
#include "publishpointclouds.h"
#include <mapit/layertypes/tflayer.h>
#include "publishtfs.h"

namespace po = boost::program_options;

enum PublishType { pointcloud };

std::vector<std::string>
get_substrings(std::string row)
{
  std::vector<std::string> data;

  std::size_t begin = 0;
  std::size_t end = 0;
  while (true) {
    end = row.find(" ", begin);
    if (end == std::string::npos) {
      data.push_back( row.substr(begin, row.size()) );
      break;
    } else {
      data.push_back( row.substr(begin, end - begin) );
    }
    begin = end + 1;
  }

  return data;
}

std::string
get_publisher_name(const std::shared_ptr<mapit::Workspace>& workspace, const ::Entity& entity)
{
    if ( 0 == entity.entity->type().compare( TfEntitydata::TYPENAME() ) ) {
        std::shared_ptr<mapit::AbstractEntitydata> ed_a = workspace->getEntitydataReadOnly(entity.entity_name);
        std::shared_ptr<TfEntitydata> ed = std::static_pointer_cast<TfEntitydata>(ed_a);
        if ( ed->getData()->get_is_static() ) {
            return "/tf_static";
        } else {
            return "/tf";
        }
    } else {
        return entity.user_prefix;
    }
}

int
get_program_options(int argc, char *argv[], po::variables_map& vars)
{
  po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name>");
  program_options_desc.add_options()
      ("help,h", "print usage")
      ("workspace,w", po::value<std::string>()->required(), "the workspace (formerly workspace) to work with")
      ("use_sim_time,s", po::value<bool>()->default_value(false), "whenever the clock should be published or not.\n"
                                                                  "When entities are shown, this param will be ignored.\n"
                                                                  "(Only usefull in the \"playback mode\" which is only availible when layesr are displayed)")
      ("data,d", po::value<std::vector<std::string>>()->multitoken(), "This can be one ore more trees or entities.\n"
                                                                      "E.g. \"tree_1\" \"entity_2\" \"tree_2\" \"entity_1\" ...")
      ("use_playback,p", po::value<bool>()->default_value(false), "whenever the data should be displayed in a \"playback mode\"\n"
                                                                  "or \"all at once\", when \"use_sim_time\" is set, this is expected to be true");

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

int
get_mapit_data(  po::variables_map& vars
               , std::shared_ptr<mapit::Repository>& repo
               , std::shared_ptr<mapit::Workspace>& workspace)
{
  repo = std::shared_ptr<mapit::Repository>( mapit::RepositoryFactoryStandard::openRepository( vars ) );

  workspace = repo->getWorkspace( vars["workspace"].as<std::string>() );
  if(workspace == nullptr)
  {
      log_error("Workspace \"" + vars["workspace"].as<std::string>() + "\" not found");
      std::vector<std::string> possibleWorkspaces = repo->listWorkspaceNames();
      if (possibleWorkspaces.size() == 0) {
          log_info("No possible workspace");
      }
      for ( std::string workspace : possibleWorkspaces ) {
          log_info("Possible workspace: " + workspace);
      }
      return 1;
  }

  if ( ! vars.count("data")) {
      log_error("the param \"data\" needs to be specified");
//      std::cout << program_options_desc << std::endl;
      return 1;
  }

  return 0;
}

int
load_entities( const po::variables_map& vars
             , const std::shared_ptr<mapit::Workspace> workspace
             , std::list<::Entity>& entities)
{
    std::vector<std::string> data_names = vars["data"].as<std::vector<std::string>>();
    for (std::string data_name : data_names) {
        std::shared_ptr<mapit::msgs::Entity> entity = workspace->getEntity(data_name);
        // is entity => add
        if (entity != nullptr) {
            ::Entity entity_struct;
            entity_struct.entity = entity;
            entity_struct.entity_name = data_name;
            entity_struct.user_prefix = data_name;
            entities.push_back( entity_struct );
        } else {
            std::shared_ptr<mapit::msgs::Tree> tree = workspace->getTree(data_name);
            // is tree => depth first search for all entities
            if (tree != nullptr) {
                ObjectReference nullRef;
                mapit::depthFirstSearchWorkspace(
                            workspace.get(),
                            tree,
                            nullRef,
                            data_name,
                            depthFirstSearchWorkspaceAll(mapit::msgs::Tree),
                            depthFirstSearchWorkspaceAll(mapit::msgs::Tree),
                            [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                            {
                                ::Entity entity_struct;
                                entity_struct.entity = obj;
                                entity_struct.entity_name = path;
                                entity_struct.user_prefix = data_name;
                                entities.push_back( entity_struct );
                                return true;
                            },
                            depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
                        );
            } else {
                log_warn("can't open \"" + data_name + "\", its neither a entity nor a tree");
            }
        }
    }

  return 0;
}

int
start_publishing_in_playback_mode_and_clock(  const po::variables_map& vars
                                  , const std::shared_ptr<mapit::Workspace> workspace
                                  , const std::shared_ptr<ros::NodeHandle>& node_handle
                                  , const std::string& pub_name
                                  , const std::list<::Entity>& entities
                                  , std::list<std::shared_ptr<PublishToROS>>& publish_managers
                                  , std::shared_ptr<PublishClock>& publish_clock)
{
    std::map<std::string, std::shared_ptr<PublishToROS>> entities_for_one_publisher;
    // for all entities
    for (const ::Entity& entity : entities) {
        std::string publisher_name = get_publisher_name(workspace, entity);
        // get the class that manages the publishing to ROS (search by the user input)
        std::map<std::string, std::shared_ptr<PublishToROS>>::const_iterator publisher_manager_it = entities_for_one_publisher.find( publisher_name );
        // if this does not exists, create it
        if (publisher_manager_it == entities_for_one_publisher.end()) {
            std::shared_ptr<PublishToROS> publisher_manager = nullptr;
            if ( 0 == entity.entity->type().compare( PointcloudEntitydata::TYPENAME() ) ) {
                std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<sensor_msgs::PointCloud2>(pub_name + publisher_name, 10, true));
                publisher_manager = std::make_shared<PublishPointClouds>(
                            workspace, node_handle, std::move(pub)
                            );
            } else if ( 0 == entity.entity->type().compare( TfEntitydata::TYPENAME() ) ) {
                std::string publisher_name_tf = publisher_name[0] == '/' ? publisher_name : "/" + publisher_name;
                std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<tf2_msgs::TFMessage>(publisher_name_tf, 10, true));
                publisher_manager = std::make_shared<PublishTFs>(
                            workspace, node_handle, std::move(pub)
                            );
            } else {
              log_error("layertype \"" + entity.entity->type() + "\" not implemented in this tool");
              return 1;
            }
            entities_for_one_publisher.insert(
                        std::pair<std::string, std::shared_ptr<PublishToROS>>(publisher_name, publisher_manager));
            publisher_manager_it = entities_for_one_publisher.find( publisher_name );
        }

        // add the entity to the manager
        publisher_manager_it->second->add_entity( entity );
    }
    // get to publish_managers
    for (std::pair<std::string, std::shared_ptr<PublishToROS>> publisher_manager : entities_for_one_publisher) {
        publish_managers.push_back( publisher_manager.second );
    }

  // get min offset of all
  double time_stamp_first = (*publish_managers.begin())->get_stamp_of_first_data();
  for (auto publish_manager : publish_managers) {
    double time_stamp_first_current = publish_manager->get_stamp_of_first_data();
    if (time_stamp_first > time_stamp_first_current) {
      time_stamp_first = time_stamp_first_current;
    }
  }
  // calculate the offset
  double offset = ros::Time::now().toSec() - time_stamp_first;
  if ( vars["use_sim_time"].as<bool>() ) {
    offset = 0;
  }
  // start the clock
  if ( vars["use_sim_time"].as<bool>() ) {
    bool ros_sim_time;
    node_handle->param("/use_sim_time", ros_sim_time, false);
    if ( ! ros_sim_time ) {
      log_error("publish data with simulated time, but for ROS \"/use_sim_time\" is not set or false.\n"
                "run \"rosparam set /use_sim_time true\" before executing this node again\n"
                "or start this tool with \"-s false\" to disable the sim time");
      return 1;
    }

    std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<rosgraph_msgs::Clock>("/clock", 1));
    publish_clock = std::make_shared<PublishClock>( std::move(pub), time_stamp_first );

    publish_clock->start_publishing();
  }
  // share the time offset and start publishing
  for (auto publish_manager : publish_managers) {
    publish_manager->set_offset(offset);
    publish_manager->start_publishing();
  }

  return 0;
}

int
start_publishing_all_entities_at_once(  const std::shared_ptr<mapit::Workspace> workspace
                          , const std::shared_ptr<ros::NodeHandle>& node_handle
                          , const std::string& pub_name
                          , const std::list<::Entity>& entities
                          , std::list<std::shared_ptr<PublishToROS>>& publish_managers)
{
    for (const ::Entity& entity : entities) {
        std::string publisher_name = get_publisher_name(workspace, entity);
        std::string current_pub_name = pub_name + "/" + entity.user_prefix;
        if ( 0 == entity.entity->type().compare( PointcloudEntitydata::TYPENAME() ) ) {
            std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<sensor_msgs::PointCloud2>(current_pub_name, 10, true));
            publish_managers.push_back( std::make_shared<PublishPointClouds>(
                                        workspace, node_handle, std::move(pub)
                                        )
                                      );
            publish_managers.back()->publish_entity(entity.entity_name, std::make_shared<::Entity>(entity));
        } else if ( 0 == entity.entity->type().compare( TfEntitydata::TYPENAME() ) ) {
            current_pub_name = publisher_name; // special case, don't use the /mapit/ prefix for the topic for TFs
            std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<tf2_msgs::TFMessage>(current_pub_name, 10, true));
            publish_managers.push_back( std::make_shared<PublishTFs>(
                                        workspace, node_handle, std::move(pub)
                                        )
                                      );
            publish_managers.back()->publish_entity(entity.entity_name, std::make_shared<::Entity>(entity));
        } else {
            log_error("layertype \"" + entity.entity->type() + "\" not implemented in this tool");
            return 1;
        }
    }

  return 0;
}

int main(int argc, char *argv[])
{
    mapit_init_logging();

    int ret = 0;
    // get parameter
    po::variables_map vars;
    ret = get_program_options(argc, argv, vars);
    if (ret != 0) {
      return ret;
    }

    // get mapit data
    std::shared_ptr<mapit::Repository> repo( nullptr );
    std::shared_ptr<mapit::Workspace> co( nullptr );
    ret = get_mapit_data(vars, repo, co);
    if (ret != 0) {
      return ret;
    }
    bool use_playback = vars["use_playback"].as<bool>() || vars["use_sim_time"].as<bool>();

    std::list<::Entity> entities;
    ret = load_entities(vars, co, entities);
    if (ret != 0) {
      return ret;
    }

    // connect to ROS
    ros::init (argc, argv, "mapit_stream_to_ros");
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>("~");

    // create pubblisher
    std::shared_ptr<PublishClock> publish_clock = nullptr;
    std::list<std::shared_ptr<PublishToROS>> publish_managers;

    const std::string pub_name = "/mapit/";
    if (use_playback) {
      ret = start_publishing_in_playback_mode_and_clock(vars, co, node_handle, pub_name, entities, publish_managers, publish_clock);
      if (ret != 0) {
        return ret;
      }
    } else {
      ret = start_publishing_all_entities_at_once(co, node_handle, pub_name, entities, publish_managers);
      if (ret != 0) {
        return ret;
      }
    }

    ros::spin();

    return 0;
}
