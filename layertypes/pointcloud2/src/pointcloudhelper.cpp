#include "pointcloudhelper.h"
#include "pcl/io/pcd_io.h"
#include <vector>

namespace upns
{

//// Reading Pointcloud2 from stream seems not implemented in any library. pcl uses filenames to load pcds.
//// copy pasted and use streams.
//int _readHeader (std::istream &fs, pcl::PCLPointCloud2 &cloud)
//{
//  // Default values
//  unsigned int data_idx = 0;
//  int data_type = 0;
//  int pcd_version = 6;
//  Eigen::Vector4f origin      = Eigen::Vector4f::Zero ();
//  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity ();
//  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
//  cloud.data.clear ();

//  // By default, assume that there are _no_ invalid (e.g., NaN) points
//  //cloud.is_dense = true;

//  int nr_points = 0;
//  //std::ifstream fs;
//  std::string line;

//  int specified_channel_count = 0;


//  // Open file in binary mode to avoid problem of
//  // std::getline() corrupting the result of ifstream::tellg()
//  //fs.open (file_name.c_str (), std::ios::binary);
////  if (!fs. () || fs.fail ())
////  {
////    PCL_ERROR ("[pcl::PCDReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno));
////    fs.close ();
////    return (-1);
////  }

//  // Seek at the given offset
////  fs.seekg (offset, std::ios::beg);

//  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
//  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
//  std::vector<int> field_sizes, field_counts;
//  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
//  std::vector<char> field_types;
//  std::vector<std::string> st;

//  // Read the header and fill it in with wonderful values
//  try
//  {
//    while (!fs.eof ())
//    {
//      std::getline (fs, line);
//      // Ignore empty lines
//      if (line == "")
//        continue;

//      // Tokenize the line
//      boost::trim (line);
//      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

//      std::stringstream sstream (line);
//      sstream.imbue (std::locale::classic ());

//      std::string line_type;
//      sstream >> line_type;

//      // Ignore comments
//      if (line_type.substr (0, 1) == "#")
//        continue;

//      // Version numbers are not needed for now, but we are checking to see if they're there
//      if (line_type.substr (0, 7) == "VERSION")
//        continue;

//      // Get the field indices (check for COLUMNS too for backwards compatibility)
//      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
//      {
//        specified_channel_count = static_cast<int> (st.size () - 1);

//        // Allocate enough memory to accommodate all fields
//        cloud.fields.resize (specified_channel_count);
//        for (int i = 0; i < specified_channel_count; ++i)
//        {
//          std::string col_type = st.at (i + 1);
//          cloud.fields[i].name = col_type;
//        }

//        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
//        int offset = 0;
//        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
//        {
//          cloud.fields[i].offset   = offset;
//          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
//          cloud.fields[i].count    = 1;
//        }
//        cloud.point_step = offset;
//        continue;
//      }

//      // Get the field sizes
//      if (line_type.substr (0, 4) == "SIZE")
//      {
//        specified_channel_count = static_cast<int> (st.size () - 1);

//        // Allocate enough memory to accommodate all fields
//        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
//          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

//        // Resize to accommodate the number of values
//        field_sizes.resize (specified_channel_count);

//        int offset = 0;
//        for (int i = 0; i < specified_channel_count; ++i)
//        {
//          int col_type ;
//          sstream >> col_type;
//          cloud.fields[i].offset = offset;                // estimate and save the data offsets
//          offset += col_type;
//          field_sizes[i] = col_type;                      // save a temporary copy
//        }
//        cloud.point_step = offset;
//        //if (cloud.width != 0)
//          //cloud.row_step   = cloud.point_step * cloud.width;
//        continue;
//      }

//      // Get the field types
//      if (line_type.substr (0, 4) == "TYPE")
//      {
//        if (field_sizes.empty ())
//          throw "TYPE of FIELDS specified before SIZE in header!";

//        specified_channel_count = static_cast<int> (st.size () - 1);

//        // Allocate enough memory to accommodate all fields
//        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
//          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

//        // Resize to accommodate the number of values
//        field_types.resize (specified_channel_count);

//        for (int i = 0; i < specified_channel_count; ++i)
//        {
//          field_types[i] = st.at (i + 1).c_str ()[0];
//          cloud.fields[i].datatype = static_cast<uint8_t> (pcl::getFieldType (field_sizes[i], field_types[i]));
//        }
//        continue;
//      }

//      // Get the field counts
//      if (line_type.substr (0, 5) == "COUNT")
//      {
//        if (field_sizes.empty () || field_types.empty ())
//          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

//        specified_channel_count = static_cast<int> (st.size () - 1);

//        // Allocate enough memory to accommodate all fields
//        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
//          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

//        field_counts.resize (specified_channel_count);

//        int offset = 0;
//        for (int i = 0; i < specified_channel_count; ++i)
//        {
//          cloud.fields[i].offset = offset;
//          int col_count;
//          sstream >> col_count;
//          cloud.fields[i].count = col_count;
//          offset += col_count * field_sizes[i];
//        }
//        // Adjust the offset for count (number of elements)
//        cloud.point_step = offset;
//        continue;
//      }

//      // Get the width of the data (organized point cloud dataset)
//      if (line_type.substr (0, 5) == "WIDTH")
//      {
//        sstream >> cloud.width;
//        if (cloud.point_step != 0)
//          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
//        continue;
//      }

//      // Get the height of the data (organized point cloud dataset)
//      if (line_type.substr (0, 6) == "HEIGHT")
//      {
//        sstream >> cloud.height;
//        continue;
//      }

//      // Get the acquisition viewpoint
//      if (line_type.substr (0, 9) == "VIEWPOINT")
//      {
//        pcd_version = 7;
//        if (st.size () < 8)
//          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";

//        float x, y, z, w;
//        sstream >> x >> y >> z ;
//        origin      = Eigen::Vector4f (x, y, z, 0.0f);
//        sstream >> w >> x >> y >> z;
//        orientation = Eigen::Quaternionf (w, x, y, z);
//        continue;
//      }

//      // Get the number of points
//      if (line_type.substr (0, 6) == "POINTS")
//      {
//        sstream >> nr_points;
//        // Need to allocate: N * point_step
//        cloud.data.resize (nr_points * cloud.point_step);
//        continue;
//      }

//      // Read the header + comments line by line until we get to <DATA>
//      if (line_type.substr (0, 4) == "DATA")
//      {
//        data_idx = static_cast<int> (fs.tellg ());
//        if (st.at (1).substr (0, 17) == "binary_compressed")
//         data_type = 2;
//        else
//          if (st.at (1).substr (0, 6) == "binary")
//            data_type = 1;
//        continue;
//      }
//      break;
//    }
//  }
//  catch (const char *exception)
//  {
//    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
//    //fs.close ();
//    return (-1);
//  }

//  // Exit early: if no points have been given, there's no sense to read or check anything anymore
//  if (nr_points == 0)
//  {
//    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
//    //fs.close ();
//    return (-1);
//  }

//  // Compatibility with older PCD file versions
//  if (cloud.width == 0 && cloud.height == 0)
//  {
//    cloud.width  = nr_points;
//    cloud.height = 1;
//    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
//  }
//  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

//  // if both height/width are not given, assume an unorganized dataset
//  if (cloud.height == 0)
//  {
//    cloud.height = 1;
//    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
//    if (cloud.width == 0)
//      cloud.width  = nr_points;
//  }
//  else
//  {
//    if (cloud.width == 0 && nr_points != 0)
//    {
//      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
//      //fs.close ();
//      return (-1);
//    }
//  }

//  if (int (cloud.width * cloud.height) != nr_points)
//  {
//    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
//    //fs.close ();
//    return (-1);
//  }

//  // Close file
//  //fs.close ();

//  return (0);
//}

//int readPointcloud2(std::istream &fs, pcl::PCLPointCloud2 &cloud)
//{
//  //pcl::console::TicToc tt;
//  //tt.tic ();

//  int data_type;
//  unsigned int data_idx;

//  int res = _readHeader (fs, cloud);

//  if (res < 0)
//    return (res);

//  unsigned int idx = 0;

//  // Get the number of points the cloud should have
//  unsigned int nr_points = cloud.width * cloud.height;

//  // Setting the is_dense property to true by default
//  cloud.is_dense = true;

//  // if ascii
//  if (data_type == 0)
//  {
//    // Re-open the file (readHeader closes it)
//    //std::ifstream fs;
//    //fs.open (file_name.c_str ());
////    if (!fs.is_open () || fs.fail ())
////    {
////      PCL_ERROR ("[pcl::PCDReader::read] Could not open file %s.\n", file_name.c_str ());
////      return (-1);
////    }

//    fs.seekg (data_idx);

//    std::string line;
//    std::vector<std::string> st;

//    // Read the rest of the file
//    try
//    {
//      while (idx < nr_points && !fs.eof ())
//      {
//        getline (fs, line);
//        // Ignore empty lines
//        if (line == "")
//          continue;

//        // Tokenize the line
//        boost::trim (line);
//        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

//        if (idx >= nr_points)
//        {
//          PCL_WARN ("[pcl::PCDReader::read] input file %s has more points (%d) than advertised (%d)!\n", "file_name.c_str ()", idx, nr_points);
//          break;
//        }

//        size_t total = 0;
//        // Copy data
//        for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
//        {
//          // Ignore invalid padded dimensions that are inherited from binary data
//          if (cloud.fields[d].name == "_")
//          {
//            total += cloud.fields[d].count; // jump over this many elements in the string token
//            continue;
//          }
//          for (unsigned int c = 0; c < cloud.fields[d].count; ++c)
//          {
//            switch (cloud.fields[d].datatype)
//            {
//              case pcl::PCLPointField::INT8:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::UINT8:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::INT16:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::UINT16:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::INT32:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::UINT32:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::FLOAT32:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              case pcl::PCLPointField::FLOAT64:
//              {
//                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type> (
//                    st.at (total + c), cloud, idx, d, c);
//                break;
//              }
//              default:
//                PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
//                break;
//            }
//          }
//          total += cloud.fields[d].count; // jump over this many elements in the string token
//        }
//        idx++;
//      }
//    }
//    catch (const char *exception)
//    {
//      PCL_ERROR ("[pcl::PCDReader::read] %s\n", exception);
//      //fs.close ();
//      return (-1);
//    }

//    // Close file
//    //fs.close ();
//  }
//  else
//  /// ---[ Binary mode only
//  /// We must re-open the file and read with mmap () for binary
//  {
//    // Open for reading
////    int fd = pcl_open (file_name.c_str (), O_RDONLY);
////    if (fd == -1)
////    {
////      PCL_ERROR ("[pcl::PCDReader::read] Failure to open file %s\n", file_name.c_str () );
////      return (-1);
////    }

////    // Seek at the given offset
////    off_t result = pcl_lseek (fd, offset, SEEK_SET);
////    if (result < 0)
////    {
////      pcl_close (fd);
////      PCL_ERROR ("[pcl::PCDReader::read] lseek errno: %d strerror: %s\n", errno, strerror (errno));
////      PCL_ERROR ("[pcl::PCDReader::read] Error during lseek ()!\n");
////      return (-1);
////    }

//    size_t data_size = data_idx + cloud.data.size ();
//    // Prepare the map
//    std::vector<char> map(0);
//    while(fs)
//    {
//        map.resize(map.size()+32);
//        fs.read(&map[map.size()-32], 32);
//    }

//    /// ---[ Binary compressed mode only
//    if (data_type == 2)
//    {
//      // Uncompress the data first
//      unsigned int compressed_size, uncompressed_size;
//      memcpy (&compressed_size, &map[data_idx + 0], sizeof (unsigned int));
//      memcpy (&uncompressed_size, &map[data_idx + 4], sizeof (unsigned int));
//      PCL_DEBUG ("[pcl::PCDReader::read] Read a binary compressed file with %u bytes compressed and %u original.\n", compressed_size, uncompressed_size);
//      // For all those weird situations where the compressed data is actually LARGER than the uncompressed one
//      // (we really ought to check this in the compressor and copy the original data in those cases)
//      if (data_size < compressed_size || uncompressed_size < compressed_size)
//      {
//        PCL_DEBUG ("[pcl::PCDReader::read] Allocated data size (%lu) or uncompressed size (%lu) smaller than compressed size (%u). Need to remap.\n", data_size, uncompressed_size, compressed_size);

////        munmap (map, data_size);
//        data_size = compressed_size + data_idx + 8;
////        map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
//      }

//      if (uncompressed_size != cloud.data.size ())
//      {
//        PCL_WARN ("[pcl::PCDReader::read] The estimated cloud.data size (%u) is different than the saved uncompressed value (%u)! Data corruption?\n",
//                  cloud.data.size (), uncompressed_size);
//        cloud.data.resize (uncompressed_size);
//      }

//      char *buf = static_cast<char*> (malloc (data_size));
//      // The size of the uncompressed data better be the same as what we stored in the header
//      unsigned int tmp_size = pcl::lzfDecompress (&map[data_idx + 8], compressed_size, buf, static_cast<unsigned int> (data_size));
//      if (tmp_size != uncompressed_size)
//      {
//        free (buf);
//        //pcl_close (fd);
//        PCL_ERROR ("[pcl::PCDReader::read] Size of decompressed lzf data (%u) does not match value stored in PCD header (%u). Errno: %d\n", tmp_size, uncompressed_size, errno);
//        return (-1);
//      }

//      // Get the fields sizes
//      std::vector<pcl::PCLPointField> fields (cloud.fields.size ());
//      std::vector<int> fields_sizes (cloud.fields.size ());
//      int nri = 0, fsize = 0;
//      for (size_t i = 0; i < cloud.fields.size (); ++i)
//      {
//        if (cloud.fields[i].name == "_")
//          continue;
//        fields_sizes[nri] = cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);
//        fsize += fields_sizes[nri];
//        fields[nri] = cloud.fields[i];
//        ++nri;
//      }
//      fields.resize (nri);
//      fields_sizes.resize (nri);

//      // Unpack the xxyyzz to xyz
//      std::vector<char*> pters (fields.size ());
//      int toff = 0;
//      for (size_t i = 0; i < pters.size (); ++i)
//      {
//        pters[i] = &buf[toff];
//        toff += fields_sizes[i] * cloud.width * cloud.height;
//      }
//      // Copy it to the cloud
//      for (size_t i = 0; i < cloud.width * cloud.height; ++i)
//      {
//        for (size_t j = 0; j < pters.size (); ++j)
//        {
//          memcpy (&cloud.data[i * fsize + fields[j].offset], pters[j], fields_sizes[j]);
//          // Increment the pointer
//          pters[j] += fields_sizes[j];
//        }
//      }
//      //memcpy (&cloud.data[0], &buf[0], uncompressed_size);

//      free (buf);
//    }
//    else
//      // Copy the data
//      memcpy (&cloud.data[0], &map[0] + data_idx, cloud.data.size ());

//    // Unmap the pages of memory
////#ifdef _WIN32
////    UnmapViewOfFile (map);
////    CloseHandle (fm);
////#else
////    if (munmap (map, data_size) == -1)
////    {
////      pcl_close (fd);
////      PCL_ERROR ("[pcl::PCDReader::read] Munmap failure\n");
////      return (-1);
////    }
////#endif
////    pcl_close (fd);
//  }

//  if ((idx != nr_points) && (data_type == 0))
//  {
//    PCL_ERROR ("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
//    return (-1);
//  }

//  // No need to do any extra checks if the data type is ASCII
//  if (data_type != 0)
//  {
//    int point_size = static_cast<int> (cloud.data.size () / (cloud.height * cloud.width));
//    // Once copied, we need to go over each field and check if it has NaN/Inf values and assign cloud.is_dense to true or false
//    for (uint32_t i = 0; i < cloud.width * cloud.height; ++i)
//    {
//      for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
//      {
//        for (uint32_t c = 0; c < cloud.fields[d].count; ++c)
//        {
//          switch (cloud.fields[d].datatype)
//          {
//            case pcl::PCLPointField::INT8:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::UINT8:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::INT16:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::UINT16:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::INT32:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::UINT32:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::FLOAT32:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//            case pcl::PCLPointField::FLOAT64:
//            {
//              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(cloud, i, point_size, d, c))
//                cloud.is_dense = false;
//              break;
//            }
//          }
//        }
//      }
//    }
//  }
//  //double total_time = tt.toc ();
//  //PCL_DEBUG ("[pcl::PCDReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n",
//  //           file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time,
//  //           cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
//  return (0);
//}

//int readPointcloud2New(std::istream& s, const  ::pcl::PCLPointCloud2 &v)
//{
//    v.header.seq << s;
//    v.header.stamp << s;
//    v.header.frame_id << s;
//    v.height << s;
//    v.width << s;
//    size_t fieldssize;
//    fieldssize << s;
//    for (size_t i = 0; i < fieldssize; ++i)
//    {
//        ::pcl::PCLPointField f;
//        s >> f;
//        v.fields.push_back(f);
//    }
//    s >> v.is_bigendian;
//    s >> v.point_step;
//    s >> v.row_step;
//    size_t datasize;
//    s >> datasize;
//    for (size_t i = 0; i < datasize; ++i)
//    {
//        ::pcl::uint8_t d;
//        s >> d;
//        v.data.push_back(d);
//    }
//    s >> v.is_dense;
//    return 0;
//}

//int writePointcloud2New(std::ostream &s, const pcl::PCLPointCloud2 &v)
//{
//    s << v.header;
//    s << v.height << std::endl;
//    s << v.width << std::endl;
//    s << v.fields.size() << std::endl;
//    for (size_t i = 0; i < v.fields.size(); ++i)
//    {
//        s << v.fields[i] << std::endl;
//    }
//    s << v.is_bigendian << std::endl;
//    s << v.point_step << std::endl;
//    s << v.row_step << std::endl;
//    s << v.data.size() << std::endl;
//    for (size_t i = 0; i < v.data.size(); ++i)
//    {
//        s << v.data[i] << std::endl;
//    }
//    s << v.is_dense << std::endl;
//    return 0;
//}

}
