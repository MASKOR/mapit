#include <mapit/msgs/services.pb.h>
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/logging.h>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "publishclock.h"
#include "publishpointclouds.h"

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

int main(int argc, char *argv[])
{
    upns_init_logging();

    // get parameter
    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout name> <destination>");
    program_options_desc.add_options()
        ("help,h", "print usage")
        ("checkout,c", po::value<std::string>()->required(), "checkout to work with")
        ("use_sim_time,s", po::value<bool>()->default_value(false), "whenever the clock should be published or not")
        ("map,m", po::value<std::string>()->required(), "the map to work with")
        ("layers,l", po::value<std::vector<std::string>>()->multitoken(), "When specified, this will be used and the entities options will be ignored."
                                                                          "this can be (if set) one ore more layer.\n"
                                                                          "E.g. \"layer_1\" \"layer_2\" ...")
        ("entities,e", po::value<std::vector<std::string>>()->multitoken(), "When layers is used, this will be ignored."
                                                                            "this can be (if set) specific layer/entity pairs.\n"
                                                                            "E.g. \"layer_1 entity_1 entity_2 ...\"\n"
                                                                            "     \"layer_2 entity_8 entity_5 ...\"\n"
                                                                            "      ...");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1);

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
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
    bool use_layers = vars.count("layers");

    std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

//    // evaluate parameter
//    PublishType publish_type;
//    if ( 0 == vars["type"].as<std::string>().compare("pointcloud") ) {
//      publish_type = PublishType::pointcloud;
//    } else {
//      log_error("unknown type to publish is given");
//      return 1;
//    }

    // get mapit data
    std::shared_ptr<upns::Checkout> co = repo->getCheckout( vars["checkout"].as<std::string>() );
    if(co == nullptr)
    {
        log_error("Checkout \"" + vars["checkout"].as<std::string>() + "\" not found");
        std::vector<std::string> possibleCheckouts = repo->listCheckoutNames();
        if (possibleCheckouts.size() == 0) {
            log_info("No possible checkout");
        }
        for ( std::string checkout : possibleCheckouts ) {
            log_info("Possible checkout: " + checkout);
        }
        return 1;
    }

    std::shared_ptr<mapit::Map> map = co->getMap(vars["map"].as<std::string>());
    if (map == nullptr) {
      log_error("can't open map \"" + vars["map"].as<std::string>() + "\"");
      return 1;
    }

    std::list<std::shared_ptr<mapit::Layer>> layers;
    std::list<std::shared_ptr<mapit::Entity>> entities;
    if (use_layers) {
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
    } else {
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
    }

    // connect to ROS
    ros::init (argc, argv, "mapit_stream_to_ros__" + vars["map"].as<std::string>());

    std::shared_ptr<ros::NodeHandle> node_handle = std::shared_ptr<ros::NodeHandle>( new ros::NodeHandle("~") );

    // create clock when parameter is given
    // TODO where to start/end (map time, layer time????)
    std::shared_ptr<PublishClock> publish_clock = nullptr;
    if ( vars["use_sim_time"].as<bool>() ) {
      // set simulated time
      node_handle->setParam("/use_sim_time", true);

      std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<rosgraph_msgs::Clock>("/clock", 1));
      publish_clock = std::make_shared<PublishClock>( std::move(pub) );
    }

    // create pubblisher
    std::list<std::shared_ptr<PublishToROS>> publish_managers;
    const std::string pub_name = "/mapit/";
    if (use_layers) {
      for (auto layer : layers) {
        // get type of layer
        // switch over type
        std::string current_pub_name = pub_name + "/" + layer->getDataPath();
        std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<sensor_msgs::PointCloud2>(current_pub_name, 10, true));
        publish_managers.push_back( std::make_shared<PublishPointClouds>(
                                     co, node_handle, std::move(pub), layer
                                     )
                                   );
      }
    } else {
      for (auto entity : entities) {
        std::string current_pub_name = pub_name + "/" + entity->getDataPath();
        std::unique_ptr<ros::Publisher> pub = std::make_unique<ros::Publisher>(node_handle->advertise<sensor_msgs::PointCloud2>(current_pub_name, 10, true));
        publish_managers.push_back( std::make_shared<PublishPointClouds>(
                                      co, node_handle, std::move(pub)
                                      )
                                    );
        publish_managers.back()->publish_entity(entity);
      }
    }

    ros::spin();

    return 0;
}
