/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <fstream>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <pcl/io/boost.h>
#include <pcl/common/io.h>
#include <pcl/io/leica_io.h>
#include <openjpeg.h>
#include <pcl/console/time.h>

#include <boost/version.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <pcl/io/lzf.h>
#include <cstring>
#include <cerrno>

#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
#endif
#include <boost/version.hpp>

#ifndef J2K_CFMT
#define J2K_CFMT 0
#endif

int
leica::PTXReader::tokenizeNextLine (std::ifstream &fs, std::string &line, std::vector<std::string> &st, std::size_t length_control) const
{
  getline (fs, line);
  // Exit on empty lines
  if (line == "")
    return (-1);
  // Tokenize the line
  boost::trim (line);
  boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
  // Control string length if asked
  if (length_control && (length_control != st.size ()))
    return (-2);
  
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
int
leica::PTXReader::readHeader (const std::string &file_name, sensor_msgs::PTXCloudData &cloud, 
                              Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
                              Eigen::Affine3f& transformation,
                              int &data_type, std::size_t &data_idx, 
                              std::size_t &image_idx, int &image_type,
                              std::size_t &image_size)
{
  // Default values
  data_idx = 0;
  data_type = 0;
//  ptx_version = PTX_V1;
  origin      = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  transformation = Eigen::Affine3f::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;
  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  int nr_points = 0;
  std::ifstream fs;
  std::string line;

  // Open file in binary mode to avoid problem of 
  // std::getline () corrupting the result of ifstream::tellg ()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror (errno)); 
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
//  fs.seekg (offset, std::ios::beg);

  std::vector<std::string> st;  
  // Cloud width
  if (tokenizeNextLine (fs, line, st))
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] error reading cloud width!\n");
    fs.close ();
    return (-1);
  }
  try
  {  
    cloud.width = boost::lexical_cast<int> (st.at (0));
  }
  catch (const boost::bad_lexical_cast& e)
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] unable to read cloud width: %s\n", e.what ());
    fs.close ();
    return (-2);
  }
  // Cloud height
  if (tokenizeNextLine (fs, line, st))
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] error reading cloud height!\n");
    fs.close ();
    return (-1);
  }
  try
  {  
    cloud.height = boost::lexical_cast<int> (st.at (0));
  }
  catch (const boost::bad_lexical_cast& e)
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] unable to read cloud height: %s\n", e.what ());
    fs.close ();
    return (-2);
  }
  // If both height/width are not given, exit
  if (cloud.height == 0 && cloud.height == 0)
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] cloud height and width are null!\n");
    fs.close ();
    return (-1);
  }
  // Sensor position
  if (tokenizeNextLine (fs, line, st, 3))
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] error reading sensor position!\n");
    fs.close ();
    return (-1);
  }
  for (std::size_t i = 0; i < st.size (); ++i)
  {
    try
    {
      origin[i] =static_cast<float> (boost::lexical_cast<double> (st.at (i)));
    }
    catch (const boost::bad_lexical_cast& e)
    {
      PCL_ERROR ("[leica::PTXReader::readHeader] unable to read position at (%zu): %s\n", i, e.what ());
      fs.close ();
      return (-2);
    }
  }
  std::cout <<  "origin " << origin.transpose () << std::endl;
  // Sensor orientation
  for (int l = 0; l < 3; ++l)
  {
    if (tokenizeNextLine (fs, line, st, 3))
    {
      PCL_ERROR ("[leica::PTXReader::readHeader] error reading sensor orientation line %d!\n", l);
      fs.close ();
      return (-1);
    }
    for (std::size_t i = 0; i < st.size (); ++i)
    {
      try
      {
        orientation.matrix ().coeffRef (i,l) = static_cast<float> (boost::lexical_cast<double> (st.at (i)));
      }
      catch (const boost::bad_lexical_cast& e)
      {
        PCL_ERROR ("[leica::PTXReader::readHeader] unable to read orientation at (%d,%zu) %s\n", l, i, e.what ());
        fs.close ();
        return (-2);
      }
    }
  }
  std::cout <<  "orientation " << orientation.matrix () << std::endl;
  // Transformation matrix
  for (int l = 0; l < 4; ++l)
  {
    if (tokenizeNextLine (fs, line, st, 4))
    {
      PCL_ERROR ("[leica::PTXReader::readHeader] error reading transformation matrix line %d!\n", l);
      fs.close ();
      return (-1);
    }
    for (std::size_t i = 0; i < st.size (); ++i)
    {
      try
      {
        transformation.matrix ().coeffRef (l,i) = static_cast<float> (boost::lexical_cast<double> (st.at (i)));
      }
      catch (const boost::bad_lexical_cast& e)
      {
        PCL_ERROR ("[leica::PTXReader::readHeader] unable to read transformation matrix at (%d,%zu): %s\n", l, i, e.what ());
        fs.close ();
        return (-2);
      }
    }
  }  
  std::cout <<  "transformation " << transformation.matrix () << std::endl;
  // Data type
  if (tokenizeNextLine (fs, line, st, 1))
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] error reading data_type!\n");
    fs.close ();
    return (-1);
  }
  if (st[0] == "ascii")
    data_type = 0;
  else if (st[0] == "binary")
    data_type = 1;
  else if (st[0] == "binary_compressed")
    data_type = 2;
  else
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] unknown data type: %s\n", st[0].c_str ());
    fs.close ();
    return (-2);
  }
  std::cout << "data_type " << st[0] << " : " << data_type << std::endl;
  // X Y Z I
  cloud.fields.resize (4);
  cloud.fields[0].name = "x"; 
  cloud.fields[0].offset = static_cast<pcl::uint32_t> (sizeof (float));
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32; 
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = static_cast<pcl::uint32_t> (sizeof (float));
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32; 
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = static_cast<pcl::uint32_t> (sizeof (float));
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32; 
  cloud.fields[2].count = 1;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = static_cast<pcl::uint32_t> (sizeof (float));
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32; 
  cloud.fields[3].count = 1;
  cloud.point_step = 4 * 1 * static_cast<pcl::uint32_t> (sizeof (float));

  // Image offset
  if (tokenizeNextLine (fs, line, st, 1))
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] error reading image_offset!\n");
    fs.close ();
    return (-1);
  }
  try
  {  
    cloud.image_offset = boost::lexical_cast<std::size_t> (st[0]);
  }
  catch (const boost::bad_lexical_cast& e)
  {
    PCL_ERROR ("[leica::PTXReader::readHeader] unable to read image_offset: %s\n", e.what ());
    fs.close ();
    return (-2);
  }
  if (cloud.image_offset != -1)
  {
    // Image encoding
    if (tokenizeNextLine (fs, line, st, 1))
    {
      PCL_ERROR ("[leica::PTXReader::readHeader] error reading image_encoding!\n");
      fs.close ();
      return (-1);
    }
    if (st[0] == "bgr8")
      image_type = 0;
    else if (st[0] == "jp2k")
    {
      image_type = 2;
      try
      {
        image_size = boost::lexical_cast<std::size_t> (st[1]);
      }
      catch (const boost::bad_lexical_cast& e)
      {
        PCL_ERROR ("[leica::PTXReader::readHeader] unable to read image size: %s\n", e.what ());
        fs.close ();
        return (-2);
      }
    }
    else
    {
      PCL_ERROR ("[leica::PTXReader::readHeader] unknown image encoding: %s\n", st[0].c_str ());
      fs.close ();
      return (-2);
    }
    std::cout << "image_encoding " << st[0] << " : " << image_type << std::endl;
    // add RGB
    if (image_type == 0)
    {
      cloud.image_encoding = st[0];
      cloud.fields.resize (5);
      cloud.fields[4].name = "rgb";
      cloud.fields[4].datatype = sensor_msgs::PointField::FLOAT32; 
      cloud.fields[4].count = 1;
      cloud.fields[4].offset = cloud.point_step;
      cloud.point_step+= static_cast<pcl::uint32_t> (sizeof (float));
    }
    else
    {
      if (tokenizeNextLine (fs, line, st, 1))
      {
        PCL_ERROR ("[leica::PTXReader::readHeader] error JP2K file name!\n");
        fs.close ();
        return (-1);
      }
      cloud.image_encoding = st[0]; 
    }
  }
  
  data_idx = static_cast<std::size_t> (fs.tellg ());
  // Close file
  fs.close ();

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
leica::PTXReader::read (const std::string &file_name, sensor_msgs::PTXCloudData &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, Eigen::Affine3f& transformation,
                      int &ptx_version, const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  std::size_t data_idx, image_offset;
  int image_encoding;
  std::size_t image_size = -1;
  int res = readHeader (file_name, cloud, origin, orientation, transformation, data_type, data_idx, image_offset, image_encoding, image_size);

  if (res < 0)
    return (res);

  unsigned int idx = 0;

  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[leica::PTXReader::read] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }
  
  // if ascii
  if (data_type == 0)
  {
    // Re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open (file_name.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
      PCL_ERROR ("[leica::PTXReader::read] Could not open file %s.\n", file_name.c_str ());
      return (-1);
    }

    fs.seekg (data_idx);
    cloud.data.resize (nr_points * cloud.point_step);

    std::string line;
    std::vector<std::string> st;
    std::cout << "cloud data size " <<  cloud.data.size () << std::endl;
    std::cout << "cloud.fields.size () " << cloud.fields.size () << std::endl;
    std::size_t total = 0;
    // Copy line data
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
          PCL_WARN ("[leica::PTXReader::read] input file %s has more points (%d) than advertised (%d)!\n", file_name.c_str (), idx, nr_points);
          break;
        }

        total = 0;
        // Copy data
        for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
        {
          float value = 0;
          if (cloud.fields[d].name == "rgb")
          {
            int r = boost::lexical_cast<int> (st.at (d));
            int g = boost::lexical_cast<int> (st.at (d+1));
            int b = boost::lexical_cast<int> (st.at (d+2));
            value = r << 16 | g << 8 | b;
          }
          else
            value = boost::lexical_cast<float> (st.at (d));
          
          memcpy (&cloud.data[idx * cloud.point_step + cloud.fields[d].offset], 
                  reinterpret_cast<char*> (&value), sizeof (float));
          total += cloud.fields[d].count; // jump over this many elements in the string token
        }
        idx++;
      }
    }
    catch (const char *exception)
    {
      PCL_ERROR ("[leica::PTXReader::read] %s\n", exception);
      fs.close ();
      return (-1);
    }

    // Close file
    fs.close ();
  }
  else
    readBinary (file_name, cloud, data_type, data_idx, image_encoding, image_size, nr_points);
  
  double total_time = tt.toc ();
  PCL_DEBUG ("[leica::PTXReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n", 
             file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time, 
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}

void 
leica::error_callback (const char *msg, void *client_data) 
{
	FILE *stream = (FILE*)client_data;
  pcl::console::print_error (stream, "[leica::PTXReader] %s", msg);
}

/**
 * warning callback expecting a FILE* client object
 */
void 
leica::warning_callback (const char *msg, void *client_data) 
{
	FILE *stream = (FILE*)client_data;
  pcl::console::print_warn (stream, "[leica::PTXReader] %s", msg);
}

/**
 *  debug callback expecting no client object
 */
void 
leica::info_callback (const char *msg, void *client_data) 
{
	(void)client_data;
  pcl::console::print_info ("[leica::PTXReader] %s", msg);
}

/*
 * Divide an integer by a power of 2 and round upwards.
 *
 * a divided by 2^b
 */
static int int_ceildivpow2(int a, int b) {
	return (a + (1 << b) - 1) >> b;
}

/*
 * Divide an integer and round upwards.
 *
 * a divided by b
 */
static int int_ceildiv(int a, int b) {
	return (a + b - 1) / b;
}

int
leica::PTXReader::readBinary (const std::string& file_name, sensor_msgs::PTXCloudData& cloud,
                              int data_type, std::size_t data_offset, 
                              int image_type, std::size_t image_size, unsigned int nr_points)
{
  // Open for reading
  int fd = pcl_open (file_name.c_str (), O_RDONLY);
  if (fd == -1)
  {
    PCL_ERROR ("[leica::PTXReader::read] Failure to open file %s\n", file_name.c_str () );
    return (-1);
  }
  std::size_t file_size = boost::filesystem::file_size (file_name);

  // Seek at the given offset
  off_t result = pcl_lseek (fd, data_offset, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    PCL_ERROR ("[leica::PTXReader::read] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[leica::PTXReader::read] Error during lseek ()!\n");
    return (-1);
  }

  cloud.data.resize (nr_points * cloud.point_step);

  std::size_t data_size = data_offset + cloud.data.size ();

  // Prepare the map
#ifdef _WIN32
  // As we don't know the real size of data (compressed or not), 
  // we set dwMaximumSizeHigh = dwMaximumSizeLow = 0 so as to map the whole file
  HANDLE fm = CreateFileMapping ((HANDLE) _get_osfhandle (fd), NULL, PAGE_READONLY, 0, 0, NULL);
  // As we don't know the real size of data (compressed or not), 
  // we set dwNumberOfBytesToMap = 0 so as to map the whole file
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, 0));
  if (map == NULL)
  {
    CloseHandle (fm);
    pcl_close (fd);
    PCL_ERROR ("[pcl::PCDReader::read] Error mapping view of file, %s\n", file_name.c_str ());
    return (-1);
  }
#else
  char *map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::PCDReader::read] Error preparing mmap for binary PCD file.\n");
    return (-1);
  }
#endif

  /// ---[ Binary compressed mode only
  if (data_type == 2)
  {
    // Uncompress the data first
    unsigned int compressed_size, uncompressed_size;
    memcpy (&compressed_size, &map[data_offset + 0], sizeof (unsigned int));
    memcpy (&uncompressed_size, &map[data_offset + 4], sizeof (unsigned int));
    PCL_DEBUG ("[leica::PTXReader::read] Read a binary compressed file with %u bytes compressed and %u original.\n", compressed_size, uncompressed_size);
    // For all those weird situations where the compressed data is actually LARGER than the uncompressed one
    // (we really ought to check this in the compressor and copy the original data in those cases)
    if (data_size < compressed_size || uncompressed_size < compressed_size)
    {
      PCL_DEBUG ("[leica::PTXReader::read] Allocated data size (%zu) or uncompressed size (%zu) smaller than compressed size (%u). Need to remap.\n", data_size, uncompressed_size, compressed_size);
      #ifdef _WIN32
        UnmapViewOfFile (map);
        data_size = compressed_size + data_offset + 8;
        map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, data_size));
#else
        munmap (map, data_size);
        data_size = compressed_size + data_offset + 8;
        map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
#endif
    }

    if (uncompressed_size != cloud.data.size ())
    {
      PCL_WARN ("[leica::PTXReader::read] The estimated cloud.data size (%u) is different than the saved uncompressed value (%u)! Data corruption?\n", 
                cloud.data.size (), uncompressed_size);
      cloud.data.resize (uncompressed_size);
    }

    char *buf = new char [data_size];
    // The size of the uncompressed data better be the same as what we stored in the header
    unsigned int tmp_size = pcl::lzfDecompress (&map[data_offset + 8], compressed_size, buf, static_cast<unsigned int> (data_size));
    if (tmp_size != uncompressed_size)
    {
      delete[] buf;
      pcl_close (fd);
      PCL_ERROR ("[leica::PTXReader::read] Size of decompressed lzf data (%u) does not match value stored in PTX header (%u). Errno: %d\n", tmp_size, uncompressed_size, errno);
      return (-1);
    }

    // Get the fields sizes
    std::vector<sensor_msgs::PointField> fields (cloud.fields.size ());
    std::vector<int> fields_sizes (cloud.fields.size ());
    int nri = 0, fsize = 0;
    for (size_t i = 0; i < cloud.fields.size (); ++i)
    {
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

    delete[] buf;
  }
  else
    // Copy the data
    memcpy (&cloud.data[0], &map[0] + data_offset, cloud.data.size ());

  // If JPEG 2000 compression is used
  if (image_type == 2)
  {
    assert (cloud.image_offset > data_offset);
    // Add the rgb field
    cloud.fields.resize (5);
    sensor_msgs::PointField rgb_field;
    rgb_field.name = "rgb";
    rgb_field.datatype = sensor_msgs::PointField::FLOAT32;
    rgb_field.count = 1;
    rgb_field.offset = data_size + cloud.point_step;
    cloud.fields.push_back (rgb_field);
    cloud.point_step+= static_cast<pcl::uint32_t> (sizeof (float));

    // decompression parameters
    opj_event_mgr_t event_mgr;		/* event manager */
    opj_dparameters_t parameters;
    opj_image_t* image = NULL;
    int file_length;
    opj_dinfo_t* dinfo = NULL;	/* handle to a decompressor */
    opj_cio_t *cio = NULL;
    opj_codestream_info_t cstr_info;  /* Codestream information structure */

    // Configure the event callbacks
    memset(&event_mgr, 0, sizeof (opj_event_mgr_t));
    event_mgr.error_handler = error_callback;
    event_mgr.warning_handler = warning_callback;
    event_mgr.info_handler = info_callback;

    // Set decoding parameters to default values 
    opj_set_default_decoder_parameters (&parameters);

		// Decode the code-stream 
    parameters.decod_format = J2K_CFMT;

    /* get a decoder handle */
    dinfo = opj_create_decompress (CODEC_J2K);
    
    /* catch events using our callbacks and give a local context */
    opj_set_event_mgr ((opj_common_ptr)dinfo, &event_mgr, stderr);
    
    /* setup the decoder decoding parameters using user parameters */
    opj_setup_decoder (dinfo, &parameters);

    /* open a byte stream */
    unsigned char* umap = (unsigned char*) (&map[0]);
    
    cio = opj_cio_open ((opj_common_ptr)dinfo, umap + data_offset + cloud.image_offset, image_size);

    /* decode the stream and fill the image structure */
    image = opj_decode (dinfo, cio);

    if(!image) 
    {
      PCL_ERROR ("[leica::PTXReader::read]: failed to decode image!\n");
      opj_destroy_decompress(dinfo);
      opj_cio_close(cio);
      return (-1);
    }
    
    // Resize cloud data
    std::size_t data_size = cloud.data.size ();
    cloud.data.resize (cloud.data.size () + cloud.width * cloud.height * sizeof (float));

    if (image->numcomps >= 3 && image->comps[0].dx == image->comps[1].dx
        && image->comps[1].dx == image->comps[2].dx
        && image->comps[0].dy == image->comps[1].dy
        && image->comps[1].dy == image->comps[2].dy
        && image->comps[0].prec == image->comps[1].prec
        && image->comps[1].prec == image->comps[2].prec) 
    {
      int w = int_ceildiv(image->x1 - image->x0, image->comps[0].dx);
      int wr = image->comps[0].w;
      
      int h = int_ceildiv(image->y1 - image->y0, image->comps[0].dy);
      int hr = image->comps[0].h;
	    
      int max = image->comps[0].prec > 8 ? 255 : (1 << image->comps[0].prec) - 1;
	    
      image->comps[0].x0 = int_ceildivpow2(image->comps[0].x0 - int_ceildiv(image->x0, image->comps[0].dx), image->comps[0].factor);
      image->comps[0].y0 = int_ceildivpow2(image->comps[0].y0 -	int_ceildiv(image->y0, image->comps[0].dy), image->comps[0].factor);
      
      int adjustR = 0, adjustG = 0, adjustB = 0, adjustA = 0;

      if (image->comps[0].prec > 8) 
      {
        adjustR = image->comps[0].prec - 8;
        printf("PNM CONVERSION: Truncating component 0 from %d bits to 8 bits\n", image->comps[0].prec);
      }

      if (image->comps[1].prec > 8) 
      {
        adjustG = image->comps[1].prec - 8;
        printf("PNM CONVERSION: Truncating component 1 from %d bits to 8 bits\n", image->comps[1].prec);
      }

      if (image->comps[2].prec > 8)
      {
        adjustB = image->comps[2].prec - 8;
        printf("PNM CONVERSION: Truncating component 2 from %d bits to 8 bits\n", image->comps[2].prec);
      }

      if (image->comps[3].prec > 8)
      {
        adjustA = image->comps[3].prec - 8;
        printf("PNM CONVERSION: Truncating component 3 from %d bits to 8 bits\n", image->comps[3].prec);
      }
      
      for (int i = 0; i < wr * hr; i++)
      {
        int r, g, b, a;
        pcl::RGB rgba;
        r = image->comps[0].data[i];
        r += (image->comps[0].sgnd ? 1 << (image->comps[0].prec - 1) : 0);
        rgba.r = (unsigned char) ((r >> adjustR)+((r >> (adjustR-1))%2));
        
        g = image->comps[1].data[i];
        g += (image->comps[1].sgnd ? 1 << (image->comps[1].prec - 1) : 0);
        rgba.g = (unsigned char) ((g >> adjustG)+((g >> (adjustG-1))%2));
        
        b = image->comps[2].data[i];
        b += (image->comps[2].sgnd ? 1 << (image->comps[2].prec - 1) : 0);
        rgba.b = (unsigned char) ((b >> adjustB)+((b >> (adjustB-1))%2));
        
        a = image->comps[3].data[i];
        a += (image->comps[3].sgnd ? 1 << (image->comps[3].prec - 1) : 0);
        rgba.a = (unsigned char) ((a >> adjustA)+((a >> (adjustA-1))%2));
        memcpy (&cloud.data[data_size + i * sizeof (pcl::RGB)], &rgba, sizeof (pcl::RGB));
      }
    }
    
		/* free remaining structures */
		if(dinfo) 
			opj_destroy_decompress(dinfo);

		/* free image data structure */
		opj_image_destroy(image);
  }
  
  // Unmap the pages of memory
#ifdef _WIN32
  UnmapViewOfFile (map);
  CloseHandle (fm);
#else
  if (munmap (map, data_size) == -1)
  {
    pcl_close (fd);
    PCL_ERROR ("[leica::PTXReader::read] Munmap failure\n");
    return (-1);
  }
#endif
  pcl_close (fd);

  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
leica::PTXWriter::setLockingPermissions (const std::string &file_name,
                                       boost::interprocess::file_lock &lock)
{
  (void)file_name;
  (void)lock;
#ifndef WIN32
  // Boost version 1.49 introduced permissions
#if BOOST_VERSION >= 104900
  // Attempt to lock the file. 
  // For mandatory locking, the filesystem must be mounted with the "mand" option in Linux (see http://www.hackinglinuxexposed.com/articles/20030623.html)
  lock = boost::interprocess::file_lock (file_name.c_str ());
  if (lock.try_lock ())
    PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] File %s locked succesfully.\n", file_name.c_str ());
  else
    PCL_DEBUG ("[pcl::PCDWriter::setLockingPermissions] File %s could not be locked!\n", file_name.c_str ());

  namespace fs = boost::filesystem;
  fs::permissions (fs::path (file_name), fs::add_perms | fs::set_gid_on_exe);
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
void
leica::PTXWriter::resetLockingPermissions (const std::string &file_name,
                                         boost::interprocess::file_lock &lock)
{
  (void)file_name;
  (void)lock;
#ifndef WIN32
  // Boost version 1.49 introduced permissions
#if BOOST_VERSION >= 104900
  (void)file_name;
  namespace fs = boost::filesystem;
  fs::permissions (fs::path (file_name), fs::remove_perms | fs::set_gid_on_exe);
  lock.unlock ();
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
std::string
leica::PTXWriter::writeHeader (const sensor_msgs::PTXCloudData &cloud, 
                               const Eigen::Vector4d &origin, 
                               const Eigen::Quaterniond &orientation, 
                               const Eigen::Affine3d& transformation,
                               int data_type,
                               bool& with_rgb)
{
  with_rgb = false;
  
  if (cloud.fields[0].name != "x")
  {
    PCL_ERROR ("[leica::PTXWriter::writeHeader] No \'x\' field!\n");
    return ("");
  }

  if (cloud.fields[1].name != "y")
  {
    PCL_ERROR ("[leica::PTXWriter::writeHeader] No \'y\' field!\n");
    return ("");
  }
  
  if (cloud.fields[2].name != "z")
  {
    PCL_ERROR ("[leica::PTXWriter::writeHeader] No \'z\' field!\n");
    return ("");
  }
  
  if (cloud.fields[3].name != "i")
  {
    PCL_ERROR ("[leica::PTXWriter::writeHeader] No \'i\' field!\n");
    return ("");
  }
  
  if (cloud.fields[3].name == "rgb" || cloud.fields[3].name == "rgba")
    with_rgb = true;
  
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());
  
  oss << cloud.width << "\n";
  oss << cloud.height << "\n";
  oss << origin[0] << origin[1] << origin[2] << "\n";
  Eigen::Matrix3d ort_mtx = orientation.matrix ();
  oss << ort_mtx (0,0) << " " << ort_mtx (0,1) << " " << ort_mtx (0,2) << "\n";
  oss << ort_mtx (1,0) << " " << ort_mtx (1,1) << " " << ort_mtx (1,2) << "\n";
  oss << ort_mtx (2,0) << " " << ort_mtx (2,1) << " " << ort_mtx (2,2) << "\n";
  Eigen::Matrix4d tft_mtx = transformation.matrix ();
  oss << tft_mtx (0,0) << " " << tft_mtx (0,1) << " " << tft_mtx (0,2) << " " << 0 << "\n";
  oss << tft_mtx (1,0) << " " << tft_mtx (1,1) << " " << tft_mtx (1,2) << " " << 0 << "\n";
  oss << tft_mtx (2,0) << " " << tft_mtx (2,1) << " " << tft_mtx (2,2) << " " << 0 << "\n";
  oss << tft_mtx (0,3) << " " << tft_mtx (1,3) << " " << tft_mtx (2,3) << " " << 1 << "\n";
  switch (data_type)
  {
    case 0 : oss << "ascii\n"; break;
    case 1 : oss << "binary\n"; break;
    case 2 : oss << "binary_compressed\n"; break;
    default : 
    {
      PCL_ERROR ("[leica::PTXWriter::writeHeader] Unknown data type %d\n", data_type);
      return ("");
    } 
  }

  return oss.str ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
leica::PTXWriter::writeASCII (const std::string &file_name, 
                              const sensor_msgs::PTXCloudData &cloud, 
                              const Eigen::Vector4d &origin, 
                              const Eigen::Quaterniond &orientation, 
                              const Eigen::Affine3d& transformation,
                              int precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[leica::PTXWriter::writeASCII] Input point cloud has no data!\n");
    return (-1);
  }
  if (cloud.fields.size () > 5)
  {
    PCL_WARN ("[leica::PTXWriter::writeASCII] Input point cloud has more fields than expected!\n");
  }
    
  std::ofstream fs;
  fs.precision (precision);
  fs.imbue (std::locale::classic ());
  fs.open (file_name.c_str ());      // Open file
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[leica::PTXWriter::writeASCII] Could not open file '%s' for writing! Error : %s\n", file_name.c_str (), strerror (errno)); 
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  int nr_points  = cloud.width * cloud.height;
  int point_size = static_cast<int> (cloud.data.size () / nr_points);
  bool with_rgb;
  // Write the header information
  fs << writeHeader (cloud, origin, orientation, transformation, 0, with_rgb);
  if (with_rgb)
    fs << "0\n" << "rgb8\n";
  else
    fs << "-1\n";
  
  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());

  int min = static_cast<int> (cloud.fields.size ());
  if (min > 5)
    min = 5;

  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (unsigned int d = 0; d < min; ++d)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (cloud.fields[d].name == "_")
        continue;

      int count = cloud.fields[d].count;
      if (count == 0) 
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c)
      {
        switch (cloud.fields[d].datatype)
        {
          case sensor_msgs::PointField::INT8:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::INT8>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::UINT8:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::UINT8>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::INT16:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::INT16>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::UINT16:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::UINT16>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::INT32:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::INT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::UINT32:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::UINT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::FLOAT32:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::FLOAT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::FLOAT64:
          {
            pcl::copyValueString<pcl::traits::asType<sensor_msgs::PointField::FLOAT64>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          default:
            PCL_WARN ("[leica::PTXWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }

        if (d < cloud.fields.size () - 1 || c < static_cast<int> (cloud.fields[d].count) - 1)
          stream << " ";
      }
    }
    // Copy the stream, trim it, and write it to disk
    std::string result = stream.str ();
    boost::trim (result);
    stream.str ("");
    fs << result << "\n";
  }
  fs.close ();              // Close file
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
leica::PTXWriter::writeBinary (const std::string &file_name, 
                               const sensor_msgs::PTXCloudData &cloud, 
                               const Eigen::Vector4d &origin, 
                               const Eigen::Quaterniond &orientation, 
                               const Eigen::Affine3d& transformation,
                               const int precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Input point cloud has no data!\n");
    return (-1);
  }
  
  std::streamoff data_idx = 0;
  bool with_rgb = false;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << writeHeader (cloud, origin, orientation, transformation, 1, with_rgb);

  if (with_rgb)
    oss << "0\n" << "rgb8\n";
  else
    oss << "-1\n";
 
  oss.flush ();
  data_idx = static_cast<unsigned int> (oss.tellp ());

#ifdef _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Error during CreateFile (%s)!\n", file_name.c_str ());
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
  {
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Error during open (%s)!\n", file_name.c_str ());
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + cloud.data.size () - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinary] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Error during write ()!\n");
    return (-1);
  }
#endif
  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, (DWORD) (data_idx + cloud.data.size ()), NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + cloud.data.size ()));
  CloseHandle (fm);

#else
  char *map = static_cast<char*> (mmap (0, static_cast<size_t> (data_idx + cloud.data.size ()), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Error during mmap ()!\n");
    return (-1);
  }
#endif

  // copy header
  memcpy (&map[0], oss.str ().c_str (), static_cast<size_t> (data_idx));

  // Copy the data
  memcpy (&map[0] + data_idx, &cloud.data[0], cloud.data.size ());

#ifndef _WIN32
  // If the user set the synchronization flag on, call msync
  if (map_synchronization_)
    msync (map, static_cast<size_t> (data_idx + cloud.data.size ()), MS_SYNC);
#endif

  // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, static_cast<size_t> (data_idx + cloud.data.size ())) == -1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinary] Error during munmap ()!\n");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle (h_native_file);
#else
  pcl_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
leica::PTXWriter::writeBinaryCompressed (const std::string &file_name, 
                                         const sensor_msgs::PTXCloudData &cloud,
                                         const Eigen::Vector4d &origin, 
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Affine3d& transformation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Input point cloud has no data!\n");
    return (-1);
  }

  bool with_rgb = false;
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << writeHeader (cloud, origin, orientation, transformation, 2, with_rgb);
  if (!with_rgb)
    oss << "0\n";

#ifdef _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Error during CreateFile (%s)!\n", file_name.c_str ());
    return (-1);
  }
#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
  {
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Error during open (%s)!\n", file_name.c_str ());
    return (-1);
  }
#endif
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  size_t fsize = 0;
  size_t data_size = 0;
  size_t nri = 0;
  std::vector<sensor_msgs::PointField> fields (cloud.fields.size ());
  std::vector<int> fields_sizes (cloud.fields.size ());
  int rgb_index = -1;
  // Compute the total size of the fields except RGB
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    if (cloud.fields[i].name == "_")
      continue;
    
    if ((cloud.fields[i].name == "rgb") || (cloud.fields[i].name == "rgba"))
    {
      rgb_index = nri;
    }
      
    fields_sizes[nri] = cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);
    fsize += fields_sizes[nri];
    fields[nri] = cloud.fields[i];
    ++nri;
  }
  fields_sizes.resize (nri);
  fields.resize (nri);
 
  // Compute the size of data
  if (rgb_index == -1)
    data_size = cloud.width * cloud.height * fsize;
  else
    data_size = cloud.width * cloud.height * (fsize - fields_sizes[rgb_index]);
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
    if (i == rgb_index)
      continue;
    pters[i] = &only_valid_data[toff];
    toff += fields_sizes[i] * cloud.width * cloud.height;
  }
  
  opj_cparameters_t parameters;	/* compression parameters */
  opj_event_mgr_t event_mgr;		/* event manager */
  /*
    configure the event callbacks (not required)
    setting of each callback is optionnal
  */
  memset(&event_mgr, 0, sizeof(opj_event_mgr_t));
  event_mgr.error_handler = error_callback;
  event_mgr.warning_handler = warning_callback;
  event_mgr.info_handler = info_callback;
  
  /* set encoding parameters to default values */
  opj_set_default_encoder_parameters (&parameters);
  
  /* Create comment for codestream */
  if(parameters.cp_comment == NULL) 
  {
    const char comment[] = "Created by OpenJPEG version ";
    const size_t clen = strlen(comment);
    const char *version = opj_version();
/* UniPG>> */
#ifdef USE_JPWL
    parameters.cp_comment = (char*)malloc(clen+strlen(version)+11);
    sprintf(parameters.cp_comment,"%s%s with JPWL", comment, version);
#else
    parameters.cp_comment = (char*)malloc(clen+strlen(version)+1);
    sprintf(parameters.cp_comment,"%s%s", comment, version);
#endif
/* <<UniPG */
  }
  
  // RGB data prep
  OPJ_COLOR_SPACE color_space = CLRSPC_SRGB;/* RGB, RGBA */
  int i, compno;
  opj_image_cmptparm_t cmptparm[4];	/* maximum of 4 components */
  opj_image_t *image = NULL;
  char value;
  int w = cloud.width;
  int h = cloud.height;
  int numcomps = (fields[rgb_index].name == "rgba") ? 4 : 3;
  int subsampling_dx = parameters.subsampling_dx;
  int subsampling_dy = parameters.subsampling_dy;
  bool is_image_compressed = false;
  int codestream_length = 0;
  opj_cio_t *cio = NULL;
  opj_cinfo_t* cinfo = NULL;
  
  if (with_rgb)
  {
    for (int i = 0; i < numcomps; i++) 
    {
      cmptparm[i].prec = 8;
      cmptparm[i].bpp = 8;
      cmptparm[i].sgnd = 0;
      cmptparm[i].dx = subsampling_dx;
      cmptparm[i].dy = subsampling_dy;
      cmptparm[i].w = w;
      cmptparm[i].h = h;
    }
    
    /* create the image */
    image = opj_image_create (numcomps, &cmptparm[0], color_space);
    
    if (!image) 
    {
			PCL_ERROR ("Unable to create image structure!\n");
			return (-1);
		}

    /* set image offset and reference grid */
    image->x0 = parameters.image_offset_x0;
    image->y0 = parameters.image_offset_y0;
    image->x1 = parameters.image_offset_x0 + (w - 1) *	subsampling_dx + 1;
    image->y1 = parameters.image_offset_y0 + (h - 1) *	subsampling_dy + 1;
    
    // Go over all the points, and copy the data in the appropriate places
    for (size_t i = 0; i < cloud.width * cloud.height; ++i)
    {
      for (size_t j = 0; j < pters.size (); ++j)
      {
        if (j != rgb_index)
        {
          memcpy (pters[j], &cloud.data[i * cloud.point_step + fields[j].offset], fields_sizes[j]);
          // Increment the pointer
          pters[j] += fields_sizes[j];
        }
        else
        {
          if (numcomps==3)
          {            
            pcl::RGB color;
            memcpy (&color, &cloud.data[i * cloud.point_step + fields[j].offset], sizeof (pcl::RGB));
            unsigned char r = color.r;
            unsigned char g = color.g;
            unsigned char b = color.b;
            
            image->comps[0].data[i]=r;
            image->comps[1].data[i]=g;
            image->comps[2].data[i]=b;
          }
          else if (numcomps==4)
          {
            pcl::RGB color;
            memcpy (&color, &cloud.data[i * cloud.point_step + fields[j].offset], sizeof (pcl::RGB));
            unsigned char r = color.r;
            unsigned char g = color.g;
            unsigned char b = color.b;
            unsigned char a = color.a;
          
            image->comps[0].data[i]=r;
            image->comps[1].data[i]=g;
            image->comps[2].data[i]=b;
            image->comps[3].data[i]=a;
          }
        }
      }
    }

		/* Decide if MCT should be used */
		parameters.tcp_mct = image->numcomps == 3 ? 1 : 0;
    // nizar set to JPEG2 compressed image data
    parameters.cod_format = J2K_CFMT;

    /* get a JP2 compressor handle */
    cinfo = opj_create_compress(CODEC_J2K);
    
    /* catch events using our callbacks and give a local context */
    opj_set_event_mgr((opj_common_ptr)cinfo, &event_mgr, stderr);			
    
    /* setup the encoder parameters using the current image and using user parameters */
    opj_setup_encoder(cinfo, &parameters, image);
    
    /* open a byte stream for writing */
    /* allocate memory for all tiles */
    cio = opj_cio_open((opj_common_ptr)cinfo, NULL, 0);

    is_image_compressed = opj_encode(cinfo, cio, image, NULL);

    if (!is_image_compressed)
    {
      opj_cio_close(cio);
      PCL_ERROR ("failed to encode image\n");
      return (-1);
    }
    codestream_length = cio_tell (cio);
  }

  char* temp_buf = static_cast<char*> (malloc (static_cast<std::size_t> (static_cast<float> (data_size) * 1.5f + 8.0f)));
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
    if (is_image_compressed)
      oss << compressed_final_size << "\njp2k " << codestream_length << "\n";
    else
      oss << "-1\n";
      
    oss.flush ();
    data_idx = oss.tellp ();
    compressed_final_size += codestream_length;
  }
  else
  {
#ifndef _WIN32
    pcl_close (fd);
#endif
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[leica::PTXWriter::writeBinaryCompressed] Error during compression!");
    return (-1);
  }

#ifndef _WIN32
  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + data_size + codestream_length - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Error during write ()!\n");
    return (-1);
  }
#endif

  // Prepare the map
#ifdef _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, compressed_final_size, NULL);
  char *map = static_cast<char*> (MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, compressed_final_size));
  CloseHandle (fm);
#else
  char *map = static_cast<char*> (mmap (0, compressed_final_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Error during mmap ()!\n");
    return (-1);
  }
#endif
  
  // Copy header
  memcpy (&map[0], oss.str ().c_str (), static_cast<size_t> (data_idx));
  // Copy the compressed data
  memcpy (&map[data_idx], temp_buf, data_size);
  if (is_image_compressed)
  {
    // Write out RGB
    memcpy (&map[data_size + data_idx], cio->buffer, codestream_length);
    /* close and free the byte stream */
    opj_cio_close(cio);
    /* free info*/
    opj_destroy_compress(cinfo);
    /* free image data */
    opj_image_destroy(image);
  }
  
#ifndef _WIN32
  // If the user set the synchronization flag on, call msync
  if (map_synchronization_)
    msync (map, compressed_final_size, MS_SYNC);
#endif

  // Unmap the pages of memory
#ifdef _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, (compressed_final_size)) == -1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[leica::PTXWriter::writeBinaryCompressed] Error during munmap ()!\n");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle (h_native_file);
#else
  pcl_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);

  free (only_valid_data);
  free (temp_buf);
  return (0);
}
