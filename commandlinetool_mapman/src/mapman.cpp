#include <boost/program_options.hpp>
#include <vector>

namespace po = boost::program_options;

/**
 * mapman map|layer|help list|store|create
 *
 * --source=<mapsource>
 *
 * map -l|list
 * map <mapId>
 * map create --name=<name> --<todo>...
 * map remove <mapId>
 * map newlayer <mapId> --name=<name> --type=<type> --usagetype=<usagetype>
 * map removelayer <layerId>
 * map addlayer <mapId> <layerId>
 *
 *
 * layer -l <mapId>
 * layer <layerId>
 *
 * layerdata read <mapId> <layerId> <minx> <miny> <minz> <maxx> <maxy> <maxz> <clip> <lod>
 * layerdata read <clip> <lod>
 *
 * layerdata store <mapId> <layerId> <minx> <miny> <minz> <maxx> <maxy> <maxz> <lod> [datastream cin]|--filename
 * layerdata store <clip> <lod> [datastream cin]|--filename
 *
 * operator <opname> <params...>
 *
 * dump
 *
 * help
 *
 */
int main(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("compression", po::value<int>(), "set compression level")
        ("input-file", po::value< std::vector<std::string> >(), "input file")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    if (vm.count("compression")) {
        std::cout << "Compression level was set to "
     << vm["compression"].as<int>() << ".\n";
    } else {
        std::cout << "Compression level was not set.\n";
    }

    return 0;
}
