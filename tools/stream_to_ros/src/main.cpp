#include <mapit/msgs/services.pb.h>
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/logging.h>
#include <boost/filesystem.hpp>
//#include <boost/iostreams/device/mapped_file.hpp>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <sensor_msgs/PointCloud2.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl_conversions/pcl_conversions.h>

namespace fs = boost::filesystem;
//namespace iostr = boost::iostreams;

namespace po = boost::program_options;

enum PublishType { pointcloud };

int main(int argc, char *argv[])
{
    upns_init_logging();

    po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <checkout name> <destination>");
    program_options_desc.add_options()
        ("help,h", "print usage")
        ("checkout,c", po::value<std::string>()->required(), "checkout to work with")
        ("use_sim_time,s", po::value<bool>()->default_value(false), "whenever the clock should be published or not")
        ("map,m", po::value<std::string>()->required(), "")
        ("layer,l", po::value<std::string>()->required(), "")
        ("entity,e", po::value<std::string>()->default_value(""), "")
        ("type,t", po::value<std::string>()->required(), "the type of what to be published,\nsupported options are:\npointcloud,\n...");
    po::positional_options_description pos_options;
    pos_options.add("checkout",  1);

    upns::RepositoryFactoryStandard::addProgramOptions(program_options_desc);
    po::variables_map vars;
    po::store(po::command_line_parser(argc, argv).options(program_options_desc).positional(pos_options).run(), vars);
    if(vars.count("help"))
    {
        std::cout << program_options_desc << std::endl;
        return 1;
    }
    po::notify(vars);

    std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );

    PublishType publish_type;
    if ( 0 == vars["type"].as<std::string>().compare("pointcloud") ) {
      publish_type = PublishType::pointcloud;
    } else {
      log_error("unknown type to publish is given");
      return 1;
    }

    bool single_entity = true;
    if ( 0 == vars["entity"].as<std::string>().compare("") ) {
      single_entity = false;
    }

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
      // TODO error
      return 1;
    }
    std::shared_ptr<mapit::Layer> layer = co->getLayer(map, vars["layer"].as<std::string>());
    if (layer == nullptr) {
      // TODO error
      return 1;
    }
    std::shared_ptr<mapit::Entity> entity;
    if ( single_entity ) {
      entity = co->getEntity(layer, vars["entity"].as<std::string>());
      if (entity == nullptr) {
        // TODO error
        return 1;
      }
    }

    // publish data
    // loop till end

    // connect to ROS
    ros::init (argc, argv, "mapit_stream_to_ros");

    // disable sim time for startup (this is needed otherwise this node itself waits for clock
    if ( vars["use_sim_time"].as<bool>() ) {
      ros::NodeHandle n("~");
      n.setParam("/use_sim_time", false);
    }

    ros::NodeHandle n("~");

    rosgraph_msgs::Clock clock;
    ros::Publisher pub_clock;
    if ( vars["use_sim_time"].as<bool>() ) {
      // set simulated time
      n.setParam("/use_sim_time", true);

      pub_clock = n.advertise<rosgraph_msgs::Clock>("/clock", 1);
      rosgraph_msgs::Clock clock;
      clock.clock.fromSec(1);
    }

    // create pubblisher
    std::string pub_name = "/mapit/" + vars["map"].as<std::string>() + "/" + vars["layer"].as<std::string>();
    if ( single_entity ) {
      pub_name += "/" + vars["entity"].as<std::string>();
    }
    ros::Publisher pub;
    switch (publish_type) {
      case PublishType::pointcloud:
        pub = n.advertise<sensor_msgs::PointCloud2>(pub_name, 10, true);
        break;
      default:
        log_error("Type is not implemented compleatly???");
    }
    if ( single_entity ) {
      // get Data
      std::shared_ptr<upns::AbstractEntitydata> entity_data_abstract = co->getEntityDataReadOnly(entity);
      std::shared_ptr<pcl::PCLPointCloud2> entity_data =
          std::dynamic_pointer_cast<PointcloudEntitydata>(
            entity_data_abstract
            )->getData();
      // publish data
      sensor_msgs::PointCloud2 entity_data_publishable;
      pcl_conversions::fromPCL(*entity_data, entity_data_publishable);
      std::string ef = entity->frame_id();
      if (0 == ef.compare("")) {
        ef = "world";
      }
      entity_data_publishable.header.frame_id = ef;
      ros::Time et;
      et.fromSec( mapit::time::to_sec( entity->stamp() ) );
      entity_data_publishable.header.stamp = et;
      pub.publish( entity_data_publishable );
    }

    ros::Rate r( 100 ); // publish the clock with 100 Hz
    while (n.ok()) {
      if ( vars["use_sim_time"].as<bool>() ) {
        // publish clock
        clock.clock.fromSec( clock.clock.toSec() + 0.01 );
        pub_clock.publish( clock );
      }

      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
