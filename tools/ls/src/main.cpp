#include <mapit/msgs/services.pb.h>
#include <upns/versioning/repository.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/logging.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int
get_program_options(int argc, char *argv[], po::variables_map& vars)
{
  po::options_description program_options_desc(std::string("Usage: ") + argv[0] + " <workspace name>");
  program_options_desc.add_options()
      ("help,h", "print usage")
      ("workspace,w", po::value<std::string>()->required(), "the workspace (formerly checkout) to work with")
      ("recursive,r", po::bool_switch()->default_value(false), "the structure should be displayed recursivly")
      ("map,m", po::value<std::string>(), "the content within the map")
      ("layer,l", po::value<std::string>(), "the content within the layer, needs map to be specified")
      ("entity,e", po::value<std::string>(), "the content within the entity, needs layer to be specified");
  po::positional_options_description pos_options;
  pos_options.add("workspace",  1);

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

  return 0;
}

void
display_map(std::string name)
{
  log_info("- " + name);
}

void
display_layer(std::string name, std::string type)
{
  log_info("  |- " + name + "\t\t[" + type + "]");
}

void
display_entity(std::string name)
{
  log_info("  |  |- " + name);
}

void
display_entities(std::shared_ptr<upns::Checkout> co, std::shared_ptr<mapit::Layer> layer, bool use_recursive)
{
  std::list<std::shared_ptr<mapit::Entity>> entities = co->getListOfEntities(layer);
  for (auto entity : entities) {
    display_entity(entity->getName());
  }
}

void
display_layers(std::shared_ptr<upns::Checkout> co, std::shared_ptr<mapit::Map> map, bool use_recursive)
{
  std::list<std::shared_ptr<mapit::Layer>> layers = co->getListOfLayers(map);
  for (auto layer : layers) {
    display_layer(layer->getName(), layer->getTypeString());
    if (use_recursive) {
      display_entities(co, layer, use_recursive);
    }
  }
}

void
display_maps(std::shared_ptr<upns::Checkout> co, bool use_recursive)
{
  std::list<std::shared_ptr<mapit::Map>> maps = co->getListOfMaps();
  for (auto map : maps) {
    display_map(map->getName());
    if (use_recursive) {
      display_layers(co, map, use_recursive);
    }
  }
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
    bool use_map = vars.count("map");
    bool use_layer = vars.count("layer");
    if (use_layer && ! use_map) {
      log_error("when a layer is given, a map needs to be given as well");
      return 1;
    }
    bool use_entity = vars.count("entity");
    if (use_entity && ! use_layer) {
      log_error("when an entity is given, a layer needs to be given as well");
      return 1;
    }

    // connect to repo and workspace
    std::shared_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepository( vars ) );
    std::shared_ptr<upns::Checkout> co = repo->getCheckout( vars["workspace"].as<std::string>() );
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

    if (use_entity) {
      std::shared_ptr<mapit::Map> map = co->getMap( vars["map"].as<std::string>() );
      display_map(map->getName());
      std::shared_ptr<mapit::Layer> layer = co->getLayer( map, vars["layer"].as<std::string>() );
      display_layer(layer->getName(), layer->getTypeString());
      std::shared_ptr<mapit::Entity> entity = co->getEntity( layer, vars["entity"].as<std::string>() );
      display_entity(entity->getName());
    } else if (use_layer) {
      std::shared_ptr<mapit::Map> map = co->getMap( vars["map"].as<std::string>() );
      display_map(map->getName());
      std::shared_ptr<mapit::Layer> layer = co->getLayer( map, vars["layer"].as<std::string>() );
      display_layer(layer->getName(), layer->getTypeString());
      display_entities(co, layer, use_recursive);
    } else if (use_map) {
      std::shared_ptr<mapit::Map> map = co->getMap( vars["map"].as<std::string>() );
      display_map(map->getName());
      display_layers(co, map, use_recursive);
    } else if ( ! use_map ) {
      display_maps(co, use_recursive);
    }

    return 0;
}
