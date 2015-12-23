#include <boost/program_options.hpp>
#include <vector>

namespace po = boost::program_options;

/**
 * mapman map|layer|help list|store|create
 *
 * --source=<mapsource>
 *
 * map -l|list
 *  - list all mapIds
 * map read <mapId>
 *  - show info about map (name, author, ...)
 * map update <mapId> --name=<name> --<todo>...
 *  - update info about map (not layerdata update, that would be an <operation>)
 * map remove <mapId>
 *  - to be discussed
 *
 * layer -l <mapId>
 *  - shows all layers of a map
 * layer read <layerId>
 *  - shows info about layer
 * layer update <mapId> --name=<name> --usagetype=<usagetype>
 *  - update info about layer (not layerdata update, that would be an <operation>)
 * layer remove <mapId> <layerId>
 *  - to be discussed
 * layer readdata <mapId> <layerId> <minx> <miny> <minz> <maxx> <maxy> <maxz> <clip> <lod>
 *  - show layerdata (binary chunk of layer) in specific format (e.g. octomap, pcd, ...)
 * layer readdata <mapId> <layerId> <clip> <lod>
 *  - see above
 *  - note: transform+timestamp history gets lost/is not retrievable?
 *  - note: layer readobject <mapId> <layerId> <objectId>
 * layer persist <add|remove> <mapId> <layerId>
 *  - mark layer as physically stored (for fast retrieval/manipulation)
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
 *   Thus, "map create" should be expressed by something like "operator copy --name=mymap" (copy makes no sense)
 *   To read rawdata: "operator load /home/.../myPcd.pcd"
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
