#include <mapit/msgs/services.pb.h>
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/logging.h>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include "publishclock.h"
#include <rosgraph_msgs/Clock.h>

#include <upns/layertypes/pointcloudlayer.h>
#include "publishpointclouds.h"
#include <upns/layertypes/tflayer.h>
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

int
get_program_options(int argc, char *argv[], po::variables_map& vars)
{
  po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout name>");
  program_options_desc.add_options()
      ("help,h", "print usage")
      ("workspace,w", po::value<std::string>()->required(), "the workspace (formerly checkout) to work with")
      ("use_sim_time,s", po::value<bool>()->default_value(false), "whenever the clock should be published or not.\n"
                                                                  "When entities are shown, this param will be ignored.\n"
                                                                  "(Only usefull in the \"playback mode\" which is only availible when layesr are displayed)")
      ("map,m", po::value<std::string>()->required(), "the map to work with")
      ("layers,l", po::value<std::vector<std::string>>()->multitoken(), "When specified, this will be used and the entities options will be ignored."
                                                                        "this can be (if set) one ore more layer.\n"
                                                                        "E.g. \"layer_1\" \"layer_2\" ...\n\n"
                                                                        "Data will be displayed in a \"playback mode\" (published to there relativ timestamps)")
      ("entities,e", po::value<std::vector<std::string>>()->multitoken(), "When layers is used, this will be ignored."
                                                                          "this can be (if set) specific layer/entity pairs.\n"
                                                                          "E.g. \"layer_1 entity_1 entity_2 ...\"\n"
                                                                          "     \"layer_2 entity_8 entity_5 ...\"\n"
                                                                          "      ...\n\n"
                                                                          "Data will be displayed \"all at once\" (timestamps will be ignored)");
  po::positional_options_description pos_options;
  pos_options.add("checkout",  1);

  upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
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
  if ( ! vars.count("layers") && ! vars.count("entities") ) {
    log_error("one of layers or entities need to be set");
    std::cout << program_options_desc << std::endl;
    return 1;
  }

  return 0;
}

int
get_mapit_data(  po::variables_map& vars
               , std::shared_ptr<upns::Repository>& repo
               , std::shared_ptr<upns::Checkout>& co
               , std::shared_ptr<mapit::Map>& map)
{
  repo = std::shared_ptr<upns::Repository>( upns::RepositoryFactoryStandard::openRepository( vars ) );

  co = repo->getCheckout( vars["workspace"].as<std::string>() );
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

  map = co->getMap(vars["map"].as<std::string>());
  if (map == nullptr) {
    log_error("can't open map \"" + vars["map"].as<std::string>() + "\"");
    return 1;
  }

  return 0;
}

int
load_layers(  const po::variables_map& vars
            , const std::shared_ptr<upns::Checkout> co
            , const std::shared_ptr<mapit::Map> map
            , std::list<std::shared_ptr<mapit::Layer>>& layers)
{
  // load all given layers into the layers variable
  std::vector<std::string> layer_names = vars["layers"].as<std::vector<std::string>>();
  for (std::string layer_name : layer_names) {
    std::shared_ptr<mapit::Layer> layer = co->getLayer(map, layer_name);
    if (layer == nullptr) {
      log_error("can't open layer \"" + layer_name + "\"");
      return 1;
    }
    layers.push_back(layer);
  }

  return 0;
}

int
start_publishing_layers_and_clock(  const po::variables_map& vars
                                  , const std::shared_ptr<upns::Checkout> co
                                  , const std::shared_ptr<ros::NodeHandle>& node_handle
                                  , const std::string& pub_name
                                  , const std::list<std::shared_ptr<mapit::Layer>>& layers
                                  , std::list<std::shared_ptr<PublishToROS>>& publish_managers
                                  , std::shared_ptr<PublishClock>& publish_clock)
{
  for (auto layer : layers) {
    // get type of layer
    std::string current_pub_name = pub_name + "/" + layer->getDataPath();
    if ( 0 == layer->getTypeString().compare( PointcloudEntitydata::TYPENAME() ) ) {
      std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<sensor_msgs::PointCloud2>(current_pub_name, 10, true));
      publish_managers.push_back( std::make_shared<PublishPointClouds>(
                                    co, node_handle, std::move(pub), layer
                                    )
                                  );
    } else if ( 0 == layer->getTypeString().compare( TfEntitydata::TYPENAME() ) ) {
      // when the default layername is used, use here the default topics for ROS
      if ( 0 == layer->getName().compare( upns::tf::_DEFAULT_LAYER_NAME_STATIC_ ) ) {
        current_pub_name = "/tf_static";
      } else if ( 0 == layer->getName().compare( upns::tf::_DEFAULT_LAYER_NAME_DYNAMIC_ ) ) {
        current_pub_name = "/tf";
      }
      std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<tf2_msgs::TFMessage>(current_pub_name, 10, true));
      publish_managers.push_back( std::make_shared<PublishTFs>(
                                    co, node_handle, std::move(pub), layer
                                    )
                                  );
    } else {
      log_error("layertype \"" + layer->getTypeString() + "\" not implemented in this tool");
      return 1;
    }
  }
  if (publish_managers.empty()) {
    log_error("checked for layers but didn't get any layer. This input error should have been caught beforehand...");
    return 1;
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
load_entities(  const po::variables_map& vars
              , const std::shared_ptr<upns::Checkout> co
              , const std::shared_ptr<mapit::Map> map
              , std::list<std::shared_ptr<mapit::Entity>>& entities)
{
  // load all given entities into the entities variable
  std::vector<std::string> layer_names = vars["entities"].as<std::vector<std::string>>();
  for (std::string layer_set_str : layer_names) {
    std::vector<std::string> layer_set = get_substrings(layer_set_str);
    if (layer_set.size() < 2) {
      log_warn("Does not contain layer with at least one entity \"" + layer_set_str + "\"");
      return 1;
    }

    std::string layer_name = layer_set.at(0);
    std::shared_ptr<mapit::Layer> layer = co->getLayer(map, layer_name);
    if (layer == nullptr) {
      log_error("can't open layer \"" + layer_name + "\"");
      return 1;
    }

    for (int i = 1; i < layer_set.size(); ++i) {
      std::shared_ptr<mapit::Entity> entity = co->getEntity(layer, layer_set.at(i));
      if (entity == nullptr) {
        log_error("can't open entity \"" + layer_set.at(i) + "\"");
        return 1;
      }
      entities.push_back(entity);
    }
  }

  return 0;
}

int
start_publishing_entities(  const std::shared_ptr<upns::Checkout> co
                          , const std::shared_ptr<ros::NodeHandle>& node_handle
                          , const std::string& pub_name
                          , const std::list<std::shared_ptr<mapit::Entity>>& entities
                          , std::list<std::shared_ptr<PublishToROS>>& publish_managers)
{
  for (auto entity : entities) {
    std::string current_pub_name = pub_name + "/" + entity->getDataPath();
    if ( 0 == entity->getTypeString().compare( PointcloudEntitydata::TYPENAME() ) ) {
      std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<sensor_msgs::PointCloud2>(current_pub_name, 10, true));
      publish_managers.push_back( std::make_shared<PublishPointClouds>(
                                    co, node_handle, std::move(pub)
                                    )
                                  );
      publish_managers.back()->publish_entity(entity);
    } else if ( 0 == entity->getTypeString().compare( TfEntitydata::TYPENAME() ) ) {
      std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<tf2_msgs::TFMessage>(current_pub_name, 10, true));
      publish_managers.push_back( std::make_shared<PublishTFs>(
                                    co, node_handle, std::move(pub)
                                    )
                                  );
      publish_managers.back()->publish_entity(entity);
    } else {
      log_error("layertype \"" + entity->getTypeString() + "\" not implemented in this tool");
      return 1;
    }
  }

  return 0;
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
    bool use_layers = vars.count("layers");

    // get mapit data
    std::shared_ptr<upns::Repository> repo( nullptr );
    std::shared_ptr<upns::Checkout> co( nullptr );
    std::shared_ptr<mapit::Map> map( nullptr );
    ret = get_mapit_data(vars, repo, co, map);
    if (ret != 0) {
      return ret;
    }

    std::list<std::shared_ptr<mapit::Layer>> layers;
    std::list<std::shared_ptr<mapit::Entity>> entities;
    if (use_layers) {
      ret = load_layers(vars, co, map, layers);
      if (ret != 0) {
        return ret;
      }
    } else {
      ret = load_entities(vars, co, map, entities);
      if (ret != 0) {
        return ret;
      }
    }

    // connect to ROS
    ros::init (argc, argv, "mapit_stream_to_ros__" + vars["map"].as<std::string>());
    std::shared_ptr<ros::NodeHandle> node_handle = std::shared_ptr<ros::NodeHandle>( new ros::NodeHandle("~") );

    // create pubblisher
    std::shared_ptr<PublishClock> publish_clock = nullptr;
    std::list<std::shared_ptr<PublishToROS>> publish_managers;

    const std::string pub_name = "/mapit/";
    if (use_layers) {
      ret = start_publishing_layers_and_clock(vars, co, node_handle, pub_name, layers, publish_managers, publish_clock);
      if (ret != 0) {
        return ret;
      }
    } else {
      ret = start_publishing_entities(co, node_handle, pub_name, entities, publish_managers);
      if (ret != 0) {
        return ret;
      }
    }

    ros::spin();

    return 0;
}
