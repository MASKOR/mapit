#include <boost/program_options.hpp>
#include <vector>

namespace po = boost::program_options;

/**
 * mapman map|layer|help list|store|create
 *
 * --source=<mapsource>
 *
 * map -l|list
 * map read <mapId>
 * map update <mapId> --name=<name> --<todo>...
 * map remove <mapId>
 *
 * layer -l <mapId>
 * layer read <layerId>
 * layer update <mapId> --name=<name> --usagetype=<usagetype>
 * layer remove <mapId> <layerId>
 * layer readdata <mapId> <layerId> <minx> <miny> <minz> <maxx> <maxy> <maxz> <clip> <lod>
 * layer readdata <mapId> <layerId> <clip> <lod>
 * layer persist <add|remove> <mapId> <layerId>
 *
 * operator <opname> <params...> (<params> includes mostly <mapId...> <layerId...>)
 *
 * dump
 *
 * help
 *
 * Description:
 * - layer update has no (--type=<type>)
 * - operator generates mapId(s) and layerId(s) + type + usagetype on demand
 * - maps and layers are created implicitly by operators.
 *   Thus, "map create" should be expressed by something like "operator copy --name=mymap"
 *   To read in rawdata: "operator load /home/.../myPcd.pcd"
 *   - layerdata store <mapId> <layerId> <minx> <miny> <minz> <maxx> <maxy> <maxz> <lod> [datastream cin]|--filename
 *   - layerdata store <clip> <lod> [datastream cin]|--filename
 *
 * TODO: discuss
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
