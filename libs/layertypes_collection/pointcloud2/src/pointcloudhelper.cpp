#include "pointcloudhelper.h"
#include <pcl/console/time.h>

int
writeBinaryCompressed (std::ostream &oss, const pcl::PCLPointCloud2 &cloud,
                                       const Eigen::Vector4f &origin,
                                       const Eigen::Quaternionf &orientation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PCDWriter::writeBinaryCompressed] Input point cloud has no data!\n");
    return (-1);
  }
  std::streamoff data_idx = 0;
  oss.imbue (std::locale::classic ());
  pcl::PCDWriter w;
  oss << w.generateHeaderBinaryCompressed (cloud, origin, orientation) << "DATA binary_compressed\n";
  oss.flush ();
  data_idx = oss.tellp ();

  size_t fsize = 0;
  size_t data_size = 0;
  size_t nri = 0;
  std::vector<pcl::PCLPointField> fields (cloud.fields.size ());
  std::vector<int> fields_sizes (cloud.fields.size ());
  // Compute the total size of the fields
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    if (cloud.fields[i].name == "_")
      continue;

    int fdt;
    switch (cloud.fields[i].datatype)
    {
      case pcl::PCLPointField::INT8:
      case pcl::PCLPointField::UINT8:
        fdt = (1);
        break;
      case pcl::PCLPointField::INT16:
      case pcl::PCLPointField::UINT16:
        fdt = (2);
        break;
      case pcl::PCLPointField::INT32:
      case pcl::PCLPointField::UINT32:
      case pcl::PCLPointField::FLOAT32:
        fdt = (4);
        break;
      case pcl::PCLPointField::FLOAT64:
        fdt = (8);
        break;
      default:
        fdt = (0);
    }

    fields_sizes[nri] = cloud.fields[i].count * fdt;
    fsize += fields_sizes[nri];
    fields[nri] = cloud.fields[i];
    ++nri;
  }
  fields_sizes.resize (nri);
  fields.resize (nri);

  // Compute the size of data
  data_size = cloud.width * cloud.height * fsize;

  //////////////////////////////////////////////////////////////////////
  // Empty array holding only the valid data
  // data_size = nr_points * point_size
  //           = nr_points * (sizeof_field_1 + sizeof_field_2 + ... sizeof_field_n)
  //           = sizeof_field_1 * nr_points + sizeof_field_2 * nr_points + ... sizeof_field_n * nr_points
  char *only_valid_data = static_cast<char*> (malloc (data_size));

  // Convert the XYZRGBXYZRGB structure to XXYYZZRGBRGB to aid compression. For
  // this, we need a vector of fields.size () (4 in this case), which points to
  // each individual plane:
  //   pters[0] = &only_valid_data[offset_of_plane_x];
  //   pters[1] = &only_valid_data[offset_of_plane_y];
  //   pters[2] = &only_valid_data[offset_of_plane_z];
  //   pters[3] = &only_valid_data[offset_of_plane_RGB];
  //
  std::vector<char*> pters (fields.size ());
  int toff = 0;
  for (size_t i = 0; i < pters.size (); ++i)
  {
    pters[i] = &only_valid_data[toff];
    toff += fields_sizes[i] * cloud.width * cloud.height;
  }

  // Go over all the points, and copy the data in the appropriate places
  for (size_t i = 0; i < cloud.width * cloud.height; ++i)
  {
    for (size_t j = 0; j < pters.size (); ++j)
    {
      memcpy (pters[j], &cloud.data[i * cloud.point_step + fields[j].offset], fields_sizes[j]);
      // Increment the pointer
      pters[j] += fields_sizes[j];
    }
  }

  char* temp_buf = static_cast<char*> (malloc (static_cast<size_t> (static_cast<float> (data_size) * 1.5f + 8.0f)));
  // Compress the valid data
  unsigned int compressed_size = pcl::lzfCompress (only_valid_data,
                                                   static_cast<unsigned int> (data_size),
                                                   &temp_buf[8],
                                                   static_cast<unsigned int> (static_cast<float> (data_size) * 1.5f));
  unsigned int compressed_final_size = 0;
  // Was the compression successful?
  if (compressed_size)
  {
    char *header = &temp_buf[0];
    memcpy (&header[0], &compressed_size, sizeof (unsigned int));
    memcpy (&header[4], &data_size, sizeof (unsigned int));
    data_size = compressed_size + 8;
    compressed_final_size = static_cast<unsigned int> (data_size + data_idx);
  }
  else
  {
    throw pcl::IOException ("[pcl::PCDWriter::writeBinaryCompressed] Error during compression!");
    return (-1);
  }

  // Copy the compressed data
  oss.write(temp_buf, data_size);

  free (only_valid_data);
  free (temp_buf);
  return (0);
}

int
readHeader (std::istream &is, pcl::PCLPointCloud2 &cloud, const int offset)
{
  // Default values
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  std::string line;

  int specified_channel_count = 0;


  // Seek at the given offset
  is.seekg (offset, std::ios::beg);

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!is.eof ())
    {
      getline (is, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (pcl::getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Check the format of the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    return (-1);
  }

  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    return (-1);
  }

  return (0);
}

int
readHeader (std::istream &fs, pcl::PCLPointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &pcd_version, int &data_type, unsigned int &data_idx, const int offset)
{
  // Default values
  data_idx = 0;
  data_type = 0;
  pcd_version = 6;//pcl::io::PCDReader::PCD_V6;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;

  int nr_points = 0;
  std::string line;

  int specified_channel_count = 0;

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  // field_sizes represents the size of one element in a field (e.g., float = 4, char = 1)
  // field_counts represents the number of elements in a field (e.g., x = 1, normal_x = 1, fpfh = 33)
  std::vector<int> field_sizes, field_counts;
  // field_types represents the type of data in a field (e.g., F = float, U = unsigned)
  std::vector<char> field_types;
  std::vector<std::string> st;

  // Read the header and fill it in with wonderful values
  try
  {
    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

      std::stringstream sstream (line);
      sstream.imbue (std::locale::classic ());

      std::string line_type;
      sstream >> line_type;

      // Ignore comments
      if (line_type.substr (0, 1) == "#")
        continue;

      // Version numbers are not needed for now, but we are checking to see if they're there
      if (line_type.substr (0, 7) == "VERSION")
        continue;

      // Get the field indices (check for COLUMNS too for backwards compatibility)
      if ( (line_type.substr (0, 6) == "FIELDS") || (line_type.substr (0, 7) == "COLUMNS") )
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        cloud.fields.resize (specified_channel_count);
        for (int i = 0; i < specified_channel_count; ++i)
        {
          std::string col_type = st.at (i + 1);
          cloud.fields[i].name = col_type;
        }

        // Default the sizes and the types of each field to float32 to avoid crashes while using older PCD files
        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i, offset += 4)
        {
          cloud.fields[i].offset   = offset;
          cloud.fields[i].datatype = pcl::PCLPointField::FLOAT32;
          cloud.fields[i].count    = 1;
        }
        cloud.point_step = offset;
        continue;
      }

      // Get the field sizes
      if (line_type.substr (0, 4) == "SIZE")
      {
        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <SIZE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_sizes.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          int col_type ;
          sstream >> col_type;
          cloud.fields[i].offset = offset;                // estimate and save the data offsets
          offset += col_type;
          field_sizes[i] = col_type;                      // save a temporary copy
        }
        cloud.point_step = offset;
        //if (cloud.width != 0)
          //cloud.row_step   = cloud.point_step * cloud.width;
        continue;
      }

      // Get the field types
      if (line_type.substr (0, 4) == "TYPE")
      {
        if (field_sizes.empty ())
          throw "TYPE of FIELDS specified before SIZE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <TYPE> differs than the number of elements in <FIELDS>!";

        // Resize to accommodate the number of values
        field_types.resize (specified_channel_count);

        for (int i = 0; i < specified_channel_count; ++i)
        {
          field_types[i] = st.at (i + 1).c_str ()[0];
          cloud.fields[i].datatype = static_cast<uint8_t> (pcl::getFieldType (field_sizes[i], field_types[i]));
        }
        continue;
      }

      // Get the field counts
      if (line_type.substr (0, 5) == "COUNT")
      {
        if (field_sizes.empty () || field_types.empty ())
          throw "COUNT of FIELDS specified before SIZE or TYPE in header!";

        specified_channel_count = static_cast<int> (st.size () - 1);

        // Allocate enough memory to accommodate all fields
        if (specified_channel_count != static_cast<int> (cloud.fields.size ()))
          throw "The number of elements in <COUNT> differs than the number of elements in <FIELDS>!";

        field_counts.resize (specified_channel_count);

        int offset = 0;
        for (int i = 0; i < specified_channel_count; ++i)
        {
          cloud.fields[i].offset = offset;
          int col_count;
          sstream >> col_count;
          cloud.fields[i].count = col_count;
          offset += col_count * field_sizes[i];
        }
        // Adjust the offset for count (number of elements)
        cloud.point_step = offset;
        continue;
      }

      // Get the width of the data (organized point cloud dataset)
      if (line_type.substr (0, 5) == "WIDTH")
      {
        sstream >> cloud.width;
        if (cloud.point_step != 0)
          cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
        continue;
      }

      // Get the height of the data (organized point cloud dataset)
      if (line_type.substr (0, 6) == "HEIGHT")
      {
        sstream >> cloud.height;
        continue;
      }

      // Get the acquisition viewpoint
      if (line_type.substr (0, 9) == "VIEWPOINT")
      {
        pcd_version = 7;//PCD_V7;
        if (st.size () < 8)
          throw "Not enough number of elements in <VIEWPOINT>! Need 7 values (tx ty tz qw qx qy qz).";

        float x, y, z, w;
        sstream >> x >> y >> z ;
        origin      = Eigen::Vector4f (x, y, z, 0.0f);
        sstream >> w >> x >> y >> z;
        orientation = Eigen::Quaternionf (w, x, y, z);
        continue;
      }

      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        sstream >> nr_points;
        // Need to allocate: N * point_step
        cloud.data.resize (nr_points * cloud.point_step);
        continue;
      }

      // Read the header + comments line by line until we get to <DATA>
      if (line_type.substr (0, 4) == "DATA")
      {
        data_idx = static_cast<int> (fs.tellg ());
        if (st.at (1).substr (0, 17) == "binary_compressed")
         data_type = 2;
        else
          if (st.at (1).substr (0, 6) == "binary")
            data_type = 1;
        continue;
      }
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] %s\n", exception);
    return (-1);
  }

  // Exit early: if no points have been given, there's no sense to read or check anything anymore
  if (nr_points == 0)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] No points to read\n");
    return (-1);
  }

  // Compatibility with older PCD file versions
  if (cloud.width == 0 && cloud.height == 0)
  {
    cloud.width  = nr_points;
    cloud.height = 1;
    cloud.row_step = cloud.point_step * cloud.width;      // row_step only makes sense for organized datasets
  }
  //assert (cloud.row_step != 0);       // If row_step = 0, either point_step was not set or width is 0

  // if both height/width are not given, assume an unorganized dataset
  if (cloud.height == 0)
  {
    cloud.height = 1;
    PCL_WARN ("[pcl::PCDReader::readHeader] no HEIGHT given, setting to 1 (unorganized).\n");
    if (cloud.width == 0)
      cloud.width  = nr_points;
  }
  else
  {
    if (cloud.width == 0 && nr_points != 0)
    {
      PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT given (%d) but no WIDTH!\n", cloud.height);
      return (-1);
    }
  }

  if (int (cloud.width * cloud.height) != nr_points)
  {
    PCL_ERROR ("[pcl::PCDReader::readHeader] HEIGHT (%d) x WIDTH (%d) != number of points (%d)\n", cloud.height, cloud.width, nr_points);
    return (-1);
  }


  return (0);
}

int
readPointcloudFromStream (std::istream &fs, pcl::PCLPointCloud2 &cloud,
                      Eigen::Vector4f origin, Eigen::Quaternionf orientation, int pcd_version,
                      const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;

  int res = readHeader (fs, cloud, origin, orientation, pcd_version, data_type, data_idx, offset);

  if (res < 0)
    return (res);

  unsigned int idx = 0;

  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  // if ascii
  if (data_type == 0)
  {

    fs.seekg (data_idx);

    std::string line;
    std::vector<std::string> st;

    // Read the rest of the file
    try
    {
      while (idx < nr_points && !fs.eof ())
      {
        getline (fs, line);
        // Ignore empty lines
        if (line == "")
          continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);

        if (idx >= nr_points)
        {
          PCL_WARN ("[pcl::PCDReader::read] input file %s has more points (%d) than advertised (%d)!\n", "<STREAM>", idx, nr_points);
          break;
        }

        size_t total = 0;
        // Copy data
        for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
        {
          // Ignore invalid padded dimensions that are inherited from binary data
          if (cloud.fields[d].name == "_")
          {
            total += cloud.fields[d].count; // jump over this many elements in the string token
            continue;
          }
          for (unsigned int c = 0; c < cloud.fields[d].count; ++c)
          {
            switch (cloud.fields[d].datatype)
            {
              case pcl::PCLPointField::INT8:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT8:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT8>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT16:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT16:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT16>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::INT32:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::INT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::UINT32:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::UINT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT32:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              case pcl::PCLPointField::FLOAT64:
              {
                pcl::copyStringValue<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type> (
                    st.at (total + c), cloud, idx, d, c);
                break;
              }
              default:
                PCL_WARN ("[pcl::PCDReader::read] Incorrect field data type specified (%d)!\n",cloud.fields[d].datatype);
                break;
            }
          }
          total += cloud.fields[d].count; // jump over this many elements in the string token
        }
        idx++;
      }
    }
    catch (const char *exception)
    {
      PCL_ERROR ("[pcl::PCDReader::read] %s\n", exception);
      return (-1);
    }
  }
  else
  /// ---[ Binary mode only
  /// We must re-open the file and read with mmap () for binary
  {
    // Open for reading
//    int fd = pcl_open (file_name.c_str (), O_RDONLY);
//    if (fd == -1)
//    {
//      PCL_ERROR ("[pcl::PCDReader::read] Failure to open file %s\n", file_name.c_str () );
//      return (-1);
//    }

//    // Seek at the given offset
//    off_t result = pcl_lseek (fd, offset, SEEK_SET);
//    if (result < 0)
//    {
//      pcl_close (fd);
//      PCL_ERROR ("[pcl::PCDReader::read] lseek errno: %d strerror: %s\n", errno, strerror (errno));
//      PCL_ERROR ("[pcl::PCDReader::read] Error during lseek ()!\n");
//      return (-1);
//    }

    size_t data_size = data_idx + cloud.data.size ();
//    // Prepare the map
//#ifdef _WIN32
//    // As we don't know the real size of data (compressed or not),
//    // we set dwMaximumSizeHigh = dwMaximumSizeLow = 0 so as to map the whole file
//    HANDLE fm = CreateFileMapping ((HANDLE) _get_osfhandle (fd), NULL, PAGE_READONLY, 0, 0, NULL);
//    // As we don't know the real size of data (compressed or not),
//    // we set dwNumberOfBytesToMap = 0 so as to map the whole file
//    char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, 0));
//    if (map == NULL)
//    {
//      CloseHandle (fm);
//      pcl_close (fd);
//      PCL_ERROR ("[pcl::PCDReader::read] Error mapping view of file, %s\n", file_name.c_str ());
//      return (-1);
//    }
//#else
//    char *map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
//    if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
//    {
//      pcl_close (fd);
//      PCL_ERROR ("[pcl::PCDReader::read] Error preparing mmap for binary PCD file.\n");
//      return (-1);
//    }
//#endif
    int read_pos = fs.tellg();
    /// ---[ Binary compressed mode only
    if (data_type == 2)
    {
      // Uncompress the data first
      unsigned int compressed_size, uncompressed_size;
      fs.seekg(data_idx + 0, fs.beg);
      fs.read(reinterpret_cast<std::istream::char_type*>(&compressed_size), sizeof (unsigned int));
      fs.seekg(data_idx + 4, fs.beg);
      fs.read(reinterpret_cast<std::istream::char_type*>(&uncompressed_size), sizeof (unsigned int));
      //memcpy (&compressed_size, &map[data_idx + 0], sizeof (unsigned int));
      //memcpy (&uncompressed_size, &map[data_idx + 4], sizeof (unsigned int));
      PCL_DEBUG ("[pcl::PCDReader::read] Read a binary compressed file with %u bytes compressed and %u original.\n", compressed_size, uncompressed_size);
      // For all those weird situations where the compressed data is actually LARGER than the uncompressed one
      // (we really ought to check this in the compressor and copy the original data in those cases)
      if (data_size < compressed_size || uncompressed_size < compressed_size)
      {
        PCL_DEBUG ("[pcl::PCDReader::read] Allocated data size (%lu) or uncompressed size (%lu) smaller than compressed size (%u). Need to remap.\n", data_size, uncompressed_size, compressed_size);
        data_size = compressed_size + data_idx + 8;
//#ifdef _WIN32
//        UnmapViewOfFile (map);
//        data_size = compressed_size + data_idx + 8;
//        map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, data_size));
//#else
//        munmap (map, data_size);
//        data_size = compressed_size + data_idx + 8;
//        map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
//#endif
      }

      if (uncompressed_size != cloud.data.size ())
      {
        PCL_WARN ("[pcl::PCDReader::read] The estimated cloud.data size (%u) is different than the saved uncompressed value (%u)! Data corruption?\n",
                  cloud.data.size (), uncompressed_size);
        cloud.data.resize (uncompressed_size);
      }

      char *buf = static_cast<char*> (malloc (data_size));
      char *compressed_tmp = static_cast<char*> (malloc (compressed_size));
      fs.seekg(data_idx + 8, fs.beg);
      fs.read(compressed_tmp, compressed_size);
      // The size of the uncompressed data better be the same as what we stored in the header
      unsigned int tmp_size = pcl::lzfDecompress (compressed_tmp, compressed_size, buf, static_cast<unsigned int> (data_size));
      free (compressed_tmp);
      if (tmp_size != uncompressed_size)
      {
        free (buf);
        PCL_ERROR ("[pcl::PCDReader::read] Size of decompressed lzf data (%u) does not match value stored in PCD header (%u). Errno: %d\n", tmp_size, uncompressed_size, errno);
        return (-1);
      }

      // Get the fields sizes
      std::vector<pcl::PCLPointField> fields (cloud.fields.size ());
      std::vector<int> fields_sizes (cloud.fields.size ());
      int nri = 0, fsize = 0;
      for (size_t i = 0; i < cloud.fields.size (); ++i)
      {
        if (cloud.fields[i].name == "_")
          continue;
        fields_sizes[nri] = cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);
        fsize += fields_sizes[nri];
        fields[nri] = cloud.fields[i];
        ++nri;
      }
      fields.resize (nri);
      fields_sizes.resize (nri);

      // Unpack the xxyyzz to xyz
      std::vector<char*> pters (fields.size ());
      int toff = 0;
      for (size_t i = 0; i < pters.size (); ++i)
      {
        pters[i] = &buf[toff];
        toff += fields_sizes[i] * cloud.width * cloud.height;
      }
      // Copy it to the cloud
      for (size_t i = 0; i < cloud.width * cloud.height; ++i)
      {
        for (size_t j = 0; j < pters.size (); ++j)
        {
          memcpy (&cloud.data[i * fsize + fields[j].offset], pters[j], fields_sizes[j]);
          // Increment the pointer
          pters[j] += fields_sizes[j];
        }
      }
      //memcpy (&cloud.data[0], &buf[0], uncompressed_size);

      free (buf);
    }
    else
    {
      // Copy the data
      fs.seekg(data_idx, fs.beg);
      fs.read(reinterpret_cast<std::istream::char_type*>(&cloud.data[0]), cloud.data.size ());
      //memcpy (&cloud.data[0], &map[0] + data_idx, cloud.data.size ());
    }
    fs.seekg(read_pos, fs.beg);
  }
  if ((idx != nr_points) && (data_type == 0))
  {
    PCL_ERROR ("[pcl::PCDReader::read] Number of points read (%d) is different than expected (%d)\n", idx, nr_points);
    return (-1);
  }

  // No need to do any extra checks if the data type is ASCII
  if (data_type != 0)
  {
    int point_size = static_cast<int> (cloud.data.size () / (cloud.height * cloud.width));
    // Once copied, we need to go over each field and check if it has NaN/Inf values and assign cloud.is_dense to true or false
    for (uint32_t i = 0; i < cloud.width * cloud.height; ++i)
    {
      for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
      {
        for (uint32_t c = 0; c < cloud.fields[d].count; ++c)
        {
          switch (cloud.fields[d].datatype)
          {
            case pcl::PCLPointField::INT8:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT8>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT8:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT8>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::INT16:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT16>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT16:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT16>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::INT32:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::INT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::UINT32:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::UINT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::FLOAT32:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT32>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
            case pcl::PCLPointField::FLOAT64:
            {
              if (!pcl::isValueFinite<pcl::traits::asType<pcl::PCLPointField::FLOAT64>::type>(cloud, i, point_size, d, c))
                cloud.is_dense = false;
              break;
            }
          }
        }
      }
    }
  }
  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::PCDReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n",
             "<STREAM>", cloud.is_dense ? "dense" : "non-dense", total_time,
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}



//template <typename PointT> int
//writeBinaryToStream (std::ostringstream &oss,
//                             const pcl::PointCloud<PointT> &cloud,
//                             const std::vector<int> &indices)
//{
//  if (cloud.points.empty () || indices.empty ())
//  {
//    throw pcl::IOException ("[pcl::PCDWriter::writeBinary] Input point cloud has no data or empty indices given!");
//    return (-1);
//  }
//  int data_idx = 0;
//  oss << pcl::PCDWriter::generateHeader<PointT> (cloud, static_cast<int> (indices.size ())) << "DATA binary\n";
//  oss.flush ();
//  data_idx = static_cast<int> (oss.tellp ());

//  std::vector<pcl::PCLPointField> fields;
//  std::vector<int> fields_sizes;
//  size_t fsize = 0;
//  size_t data_size = 0;
//  size_t nri = 0;
//  pcl::getFields (cloud, fields);
//  // Compute the total size of the fields
//  for (size_t i = 0; i < fields.size (); ++i)
//  {
//    if (fields[i].name == "_")
//      continue;
//    int fdt;
//    switch (fields[i].datatype)
//    {
//      case pcl::PCLPointField::INT8:
//      case pcl::PCLPointField::UINT8:
//        fdt = (1);
//        break;
//      case pcl::PCLPointField::INT16:
//      case pcl::PCLPointField::UINT16:
//        fdt = (2);
//        break;
//      case pcl::PCLPointField::INT32:
//      case pcl::PCLPointField::UINT32:
//      case pcl::PCLPointField::FLOAT32:
//        fdt = (4);
//        break;
//      case pcl::PCLPointField::FLOAT64:
//        fdt = (8);
//        break;
//      default:
//        fdt = (0);
//    }
//    int fs = fields[i].count * fdt;
//    fsize += fs;
//    fields_sizes.push_back (fs);
//    fields[nri++] = fields[i];
//  }
//  fields.resize (nri);

//  data_size = indices.size () * fsize;

//  // Copy the data
//  for (size_t i = 0; i < indices.size (); ++i)
//  {
//    int nrj = 0;
//    for (size_t j = 0; j < fields.size (); ++j)
//    {
//      oss.write(reinterpret_cast<const char*> (&cloud.points[indices[i]]) + fields[j].offset, fields_sizes[nrj]);
//    }
//  }

//  return (0);
//}
