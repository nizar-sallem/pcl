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
                              std::size_t &image_idx, int &image_type)
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
  
  int res = readHeader (file_name, cloud, origin, orientation, transformation, data_type, data_idx, image_offset, image_encoding);

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
    readBinary (file_name, cloud, data_idx, image_encoding);
  
  double total_time = tt.toc ();
  PCL_DEBUG ("[leica::PTXReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n", 
             file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time, 
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}

int
leica::PTXReader::readBinary (const std::string& file_name, sensor_msgs::PTXCloudData& cloud, std::size_t data_offset, int image_type)
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
  boost::iostreams::mapped_file_source file;
  file.open (file_name, data_size);
  if (!file.is_open ())
  {
    PCL_ERROR ("[leica::PTXReader::read] Error mapping file, %s\n", file_name.c_str ());
    return (-1);
  }
  char *map = file.data ();
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
      file.close ();
      data_size = compressed_size + data_offset + 8;
      file.open (file_name, data_size);
      map = file.data ();
    }

    if (uncompressed_size != cloud.data.size ())
    {
      PCL_WARN ("[leica::PTXReader::read] The estimated cloud.data size (%u) is different than the saved uncompressed value (%u)! Data corruption?\n", 
                cloud.data.size (), uncompressed_size);
      cloud.data.resize (uncompressed_size);
    }

    char buf = new char [data_size];
    // The size of the uncompressed data better be the same as what we stored in the header
    unsigned int tmp_size = pcl::lzfDecompress (&map[data_offset + 8], compressed_size, buf, static_cast<unsigned int> (data_size));
    if (tmp_size != uncompressed_size)
    {
      delete[] buf;
      file.close ();
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
    cloud.fields[4].name = "rgb";
    cloud.fields[4].datatype = sensor_msgs::PointField::FLOAT32; 
    cloud.fields[4].count = 1;
    cloud.fields[4].offset = data_size + cloud.point_step;
    cloud.point_step+= static_cast<pcl::uint32_t> (sizeof (pcl::float32_t));
    // Resize cloud data
    cloud.data.resize (cloud.data.size () + cloud.width * cloud.height * sizeof (pcl::float32_t));
    // decompression parameters
    opj_dparameters_t parameters;
    opj_image_t* image = NULL;
    // Stream
    opj_stream_t *l_stream = NULL;
    // Handle to a decompressor
    opj_codec_t* l_codec = NULL;
    opj_codestream_index_t* cstr_index = NULL;
    indexed_buffer image_buffer = new (&map[0] + data_offset + data_size, 
                                       file_size - (data_offset + data_size));
    if (!image_buffer)
    {
      PCL_ERROR ("[leica::PTXReader::read] Failed to create the indexed buffer!\n");
      file.close ();
      return (-1);
    }
    
		l_stream = opj_stream_create_buffer_stream (image_buffer,1);    
    if (!l_stream)
    {
      PCL_ERROR ("[leica::PTXReader::read] Failed to create the stream from buffer!\n");
      file.close ();
      return (-1);
    }
    // Set decoding parameters to default values 
    opj_set_default_decoder_parameters (&parameters);
    parameters.in_file = file_name.c_str ();
    parameters.decod_format = J2K_CFMT;
    // Get a decoder handle
    l_codec = opj_create_decompress (OPJ_CODEC_J2K);
    // Catch events using our callbacks and give a local context
		opj_set_info_handler (l_codec, info_callback,00);
		opj_set_warning_handler (l_codec, warning_callback,00);
		opj_set_error_handler (l_codec, error_callback,00);

    // Setup the decoder decoding parameters using user parameters
    if (!opj_setup_decoder (l_codec, &parameters))
    {
      PCL_ERROR ("[leica::PTXReader::read] Failed to setup the decoder!\n");
      opj_stream_destroy (l_stream);
      file.close ();
      opj_destroy_codec (l_codec);
      return (-1);
    }
    
		// Read the main header of the codestream and if necessary the J2K boxes
		if (!opj_read_header (l_stream, l_codec, &image))
    {
			PCL_ERROR ("[leica::PTXReader::read] Failed to read the header!\n");
			opj_stream_destroy (l_stream);
      file.close ();
			opj_destroy_codec (l_codec);
			opj_image_destroy (image);
			return (-1);
		}

    // Get the decoded image
    if (!(opj_decode (l_codec, l_stream, image) && opj_end_decompress (l_codec,	l_stream))) 
    {
      PCL_ERROR ("[leica::PTXReader::read] Failed to decode image!\n");
      opj_destroy_codec (l_codec);
      opj_stream_destroy (l_stream);
      opj_image_destroy (image);
      file.close ();
      return (-1);
    }

    // Close the byte stream 
		opj_stream_destroy (l_stream);
    
    if (image->color_space == OPJ_CLRSPC_SYCC)
			color_sycc_to_rgb (image); /* FIXME */

		if (image->icc_profile_buf)
    {
#if defined (HAVE_LIBLCMS1) || defined (HAVE_LIBLCMS2)
			color_apply_icc_profile (image); /* FIXME */
#endif
			free (image->icc_profile_buf);
			image->icc_profile_buf = NULL; image->icc_profile_len = 0;
		}

    int adjustR, adjustG, adjustB, pad;
		if (image->comps[0].prec > 8) {
			adjustR = image->comps[0].prec - 8;
			PCL_WARN ("Truncating component 0 from %d bits to 8 bits\n", image->comps[0].prec);
		}
		else 
			adjustR = 0;
		if (image->comps[1].prec > 8) {
			adjustG = image->comps[1].prec - 8;
			PCL_WARN ("Truncating component 1 from %d bits to 8 bits\n", image->comps[1].prec);
		}
		else 
			adjustG = 0;
		if (image->comps[2].prec > 8) {
			adjustB = image->comps[2].prec - 8;
			PCL_WARN ("Truncating component 2 from %d bits to 8 bits\n", image->comps[2].prec);
		}
		else 
			adjustB = 0;

    if (image->numcomps >= 3 && image->comps[0].dx == image->comps[1].dx
        && image->comps[1].dx == image->comps[2].dx
        && image->comps[0].dy == image->comps[1].dy
        && image->comps[1].dy == image->comps[2].dy
        && image->comps[0].prec == image->comps[1].prec
        && image->comps[1].prec == image->comps[2].prec) 
    {
      for (i = 0; i < nb_points; i++) 
      {
        int r = image->comps[0].data[nb_points - ((i) / (w) + 1) * w + (i) % (w)];
        r += (image->comps[0].sgnd ? 1 << (image->comps[0].prec - 1) : 0);
        r = ((r >> adjustR)+((r >> (adjustR-1))%2));
        if (r > 255) r = 255; else if (r < 0) r = 0;
        
        int g = image->comps[1].data[nb_points - ((i) / (w) + 1) * w + (i) % (w)];
        g += (image->comps[1].sgnd ? 1 << (image->comps[1].prec - 1) : 0);
        g = ((g >> adjustG)+((g >> (adjustG-1))%2));
        if (g > 255) g = 255; else if (g < 0) g = 0;
        
        int b = image->comps[2].data[nb_points - ((i) / (w) + 1) * w + (i) % (w)];
        b += (image->comps[2].sgnd ? 1 << (image->comps[2].prec - 1) : 0);
        b = ((b >> adjustB)+((b >> (adjustB-1))%2));
        if (b > 255) b = 255; else if (b < 0) b = 0;

        int rgba = r << 16 | g << 8 | b;
        memcpy (&cloud[data_size + i * sizeof (pcl::int32_t)], &rgba, sizeof (pcl::int32_t));
        
        if ((i + 1) % w == 0) 
        {
          for (pad = (3 * w) % 4 ? 4 - (3 * w) % 4 : 0; pad > 0; pad--)	/* ADD */
            // fprintf (fdest, "%c", 0);
            continue;
        }
      }
    }
    
    // free remaining structures
		if (l_codec) 
			opj_destroy_codec (l_codec);

		// free image data structure
		opj_image_destroy (image);

		// destroy the codestream index
		opj_destroy_cstr_index (&cstr_index);
  }
  
  // Unmap the pages of memory
  file.close ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
leica::PTXWriter::writeHeader (const sensor_msgs::PTXCloudData &cloud, 
                             const Eigen::Vector4d &origin, 
                             const Eigen::Quaterniond &orientation, 
                             const Eigen::Affine3d& transformation,
                             int data_type)
{
  bool with_rgb = false;
  
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

  // Write the header information
  fs << generateHeader (cloud, origin, orientation, transformation, 0);
  if (cloud.fields.size () >= 5)
  {
    if (cloud.fields[0].name == "rgb" || cloud.fields[0].name == "rgba")
    {
      fs << "0\n" << "rgb8\n";
    }
  }
  else
    fs << "-1\n";
  
  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());

  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (unsigned int d = 0; d < static_cast<unsigned int> (std::min (cloud.fields.size (), 5)); ++d)
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
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::INT8>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::UINT8:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::UINT8>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::INT16:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::INT16>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::UINT16:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::UINT16>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::INT32:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::INT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::UINT32:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::UINT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::FLOAT32:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::FLOAT32>::type>(cloud, i, point_size, d, c, stream);
            break;
          }
          case sensor_msgs::PointField::FLOAT64:
          {
            copyValueString<pcl::traits::asType<sensor_msgs::PointField::FLOAT64>::type>(cloud, i, point_size, d, c, stream);
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
  if (cloud.fields.size () > 5)
  {
    PCL_ERROR ("[leica::PTXWriter::writeASCII] Input point cloud has more fields than expected!\n");
    return (-1);
  }
  else
  {
    if (cloud.fields[0].name != "x" || cloud.fields[1].name != "y" || 
        cloud.fields[2].name != "z" || cloud.fields[3].name != "i")
    {
      PCL_ERROR ("[leica::PTXWriter::writeBinary] Supported point types are XYZI or XYZIRGB!\n");
      return (-1);
    }

    if ((cloud.fields.size () == 5) && (cloud.fields[4].name != "rgb") && (cloud.fields[4].name != "rgba"))
    {
      PCL_ERROR ("[leica::PTXWriter::writeBinary] Supported point types are XYZI or XYZIRGB!\n");
      return (-1);
    }
  }
  
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << generateHeader (cloud, origin, orientation, transformation, 1);

  if (cloud.fields.size () == 5)
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
  bool compress_image = false;
  if (cloud.fields.size () > 5)
  {
    PCL_ERROR ("[leica::PTXWriter::writeASCII] Input point cloud has more fields than expected!\n");
    return (-1);
  }
  else
  {
    if (cloud.fields[0].name != "x" || cloud.fields[1].name != "y" || 
        cloud.fields[2].name != "z" || cloud.fields[3].name != "i")
    {
      PCL_ERROR ("[leica::PTXWriter::writeBinary] Supported point types are XYZI or XYZIRGB!\n");
      return (-1);
    }

    if ((cloud.fields.size () == 5) && (cloud.fields[4].name != "rgb") && (cloud.fields[4].name != "rgba"))
    {
      PCL_ERROR ("[leica::PTXWriter::writeBinary] Supported point types are XYZI or XYZIRGB!\n");
      return (-1);
    }
    else
      compress_image = true;
  }

  const boost::filesystem::path file_path (file_name);
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << generateHeader (cloud, origin, orientation, transformation, 2);
  if (!compress_image)
    oss << "-1\n";
  else
  {
    oss << "0\n";
    oss <<  << "jp2k\n" << file_path.stem () << ".jp2\n";
  }

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

  // Compute the total size of the fields
  for (size_t i = 0; i < cloud.fields.size (); ++i)
  {
    if (cloud.fields[i].name == "_" || cloud.fields[i].name == "rgb" || cloud.fields[i].name == "rgba")
      continue;
      
    fields_sizes[nri] = cloud.fields[i].count * pcl::getFieldSize (cloud.fields[i].datatype);
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
    if (compress_image)
      oss << compressed_size << "\n";

    oss.flush ();
    data_idx = oss.tellp ();

    char *header = &temp_buf[0];
    memcpy (&header[0], &compressed_size, sizeof (unsigned int));
    memcpy (&header[4], &data_size, sizeof (unsigned int));
    data_size = compressed_size + 8;
    compressed_final_size = static_cast<unsigned int> (data_size + data_idx);
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
  off_t result = pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET);
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
  
  // copy header
  memcpy (&map[0], oss.str ().c_str (), static_cast<size_t> (data_idx));
  // Copy the compressed data
  memcpy (&map[data_idx], temp_buf, data_size);

  // Compress image if necessary
  if (compress_image)
  {
    bool save_alpha = false;
    std::size_t rgb_index = -1;
    std::size_t total = 0;
    for (int i = 0; i < cloud.fields.size (); ++i)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (cloud.fields[i].name == "_")
        total += cloud.fields[i].count; // jump over this many elements in the string token
      else 
      {
        if (cloud.fields[i].name.find ("rgb") != string::npos)
        {
          rgb_index = i;
          break;
        }
      }
    }

    if (rgb_index == -1)
    {
      PCL_ERROR ("No color information found!");
      return (-1);
    }

    std::size_t nr_points  = cloud.width * cloud.height;
    std::size_t point_size = cloud.data.size () / nr_points;

    opj_cparameters_t parameters;	/* compression parameters */
    std::string image_file = file_path.stem () + ".jp2";
    parameters.outfile = image_file.c_str ();
    opj_stream_t *l_stream = 00;
    opj_codec_t* l_codec = 00;
    opj_image_t *image = NULL;
    raw_cparameters_t raw_cp;
    
    OPJ_BOOL is_success;
    OPJ_BOOL use_tiles = OPJ_FALSE; /* OPJ_TRUE */
    OPJ_UINT32 l_nb_tiles = 4;

    /* set encoding parameters to default values */
    opj_set_default_encoder_parameters (&parameters);
    color_space = OPJ_CLRSPC_SRGB;/* RGB, RGBA */
    w = cloud.width;
    h = cloud.height;
    memset (&cmptparm[0], 0, numcomps * sizeof (opj_image_cmptparm_t));
    
    numcomps = (cloud.fields[rgb_index].name == "rgba") ? 4 : 3;
    subsampling_dx = parameters->subsampling_dx;
    subsampling_dy = parameters->subsampling_dy;
    
    for (int i = 0; i < numcomps; i++) 
    {
      cmptparm[i].prec = 8;
      cmptparm[i].bpp = 8;
      cmptparm[i].sgnd = 0;
      cmptparm[i].dx = subsampling_dx;
      cmptparm[i].dy = subsampling_dy;
      cmptparm[i].w = image_width;
      cmptparm[i].h = image_height;
    }
    
    /* create the image */
    image = opj_image_create (numcomps, &cmptparm[0], color_space);

    if (!image)
      return (-1);
    
    /* set image offset and reference grid */
    image->x0 = parameters->image_offset_x0;
    image->y0 = parameters->image_offset_y0;
    image->x1 =	!image->x0 ? (image_width - 1) * subsampling_dx + 1 : image->x0 + (image_width - 1) * subsampling_dx + 1;
    image->y1 =	!image->y0 ? (image_height - 1) * subsampling_dy + 1 : image->y0 + (image_height - 1) * subsampling_dy + 1;
    
    /* set image data */
    for (y=0; y < image_height; y++) 
    {
      int index = y*image_width;
      
      if (numcomps==3)
      {
        for (x=0; x<image_width; x++) 
        {
          std::size_t i = index + x;
          pcl::RGB color;
          memcpy (&color, &cloud.data[i * cloud.point_size + cloud.fields[rgb_index].offset + (total) * sizeof (unsigned int)], sizeof (pcl::RGB));
          unsigned char r = color.r;
          unsigned char g = color.g;
          unsigned char b = color.b;
          
          image->comps[0].data[index]=r;
          image->comps[1].data[index]=g;
          image->comps[2].data[index]=b;
          index++;
        }
      }
      else if (numcomps==4)
      {
        for (x=0; x<image_width; x++) 
        {
          std::size_t i = index + x;
          pcl::RGB color;
          memcpy (&color, &cloud.data[i * point_size + cloud.fields[rgb_index].offset + (total) * sizeof (unsigned int)], sizeof (pcl::RGB));
          unsigned char r = color.r;
          unsigned char g = color.g;
          unsigned char b = color.b;
          unsigned char a = color.a;
          
          image->comps[0].data[index]=r;
          image->comps[1].data[index]=g;
          image->comps[2].data[index]=b;
          image->comps[3].data[index]=a;
          index++;
        }
      }
    }
    
    if (!image) 
    {
			PCL_ERROR ("Unable to load file: got no image!\n");
			return (-1);
		}

		/* Decide if MCT should be used */
		parameters.tcp_mct = image->numcomps == 3 ? 1 : 0;
    // nizar set to JPEG2 compressed image data
    parameters.cod_format = JP2_CFMT;
    l_codec = opj_create_compress (OPJ_CODEC_JP2);
    
    /* catch events using our callbacks and give a local context */		
		opj_set_info_handler (l_codec, info_callback,00);
		opj_set_warning_handler (l_codec, warning_callback,00);
		opj_set_error_handler (l_codec, error_callback,00);

    if (use_tiles) 
    {
      parameters.cp_tx0 = 0;
      parameters.cp_ty0 = 0;
      parameters.tile_size_on = OPJ_TRUE;
      parameters.cp_tdx = 512;
      parameters.cp_tdy = 512;
    }
		opj_setup_encoder (l_codec, &parameters, image);

		/* Open the output file*/
		fout = fopen (parameters.outfile, "wb");
		if (!fout) 
    {
			PCL_ERROR ("Not enable to create output file!\n");
			opj_stream_destroy (l_stream);
			return (-1);
		}

		/* open a byte stream for writing and allocate memory for all tiles */
		l_stream = opj_stream_create_default_file_stream (fout,OPJ_FALSE);
		if (! l_stream){
			return (-1);
		}

		/* encode the image */
    is_success = opj_start_compress (l_codec,image,l_stream);
    if (!is_success)  
    {
      PCL_ERROR ("failed to encode image: opj_start_compress\n");
      return (-1);
    }

    if (use_tiles) 
    {
      OPJ_BYTE *l_data;
      OPJ_UINT32 l_data_size = 512*512*3;
      l_data = (OPJ_BYTE*) malloc ( l_data_size * sizeof (OPJ_BYTE));
      memset (l_data, 0, l_data_size );
      assert ( l_data );
      for (i=0;i<l_nb_tiles;++i) 
      {
        if (!opj_write_tile (l_codec,i,l_data,l_data_size,l_stream)) 
        {
          PCL_ERROR ("ERROR -> test_tile_encoder: failed to write the tile %d!\n",i);
          opj_stream_destroy (l_stream);
          fclose (fout);
          opj_destroy_codec (l_codec);
          opj_image_destroy (image);
          return (-1);
        }
      }
      free (l_data);
    }
    else 
    {
      is_success = is_success && opj_encode (l_codec, l_stream);
      if (!is_success)  
      {
        PCL_ERROR ("failed to encode image: opj_encode\n");
      }
    }
		is_success = is_success && opj_end_compress (l_codec, l_stream);
		if (!is_success)  
    {
			PCL_ERROR ("failed to encode image: opj_end_compress\n");
		}

		if (!is_success)  
    {
			opj_stream_destroy (l_stream);
			fclose (fout);
      opj_destroy_codec (l_codec);
      opj_image_destroy (image);
			PCL_ERROR ("failed to encode image\n");
			return (-1);
		}

		PCL_INFO ("Generated outfile %s\n",parameters.outfile);
		/* close and free the byte stream */
		opj_stream_destroy (l_stream);
		fclose (fout);

		/* free remaining compression structures */
		opj_destroy_codec (l_codec);

		/* free image data */
		opj_image_destroy (image);
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
