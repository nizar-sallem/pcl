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
 * $Id$
 *
 */

#include <fstream>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <pcl/io/boost.h>
#include <pcl/common/io.h>
#include <pcl/io/ptx_io.h>
#include <pcl/console/time.h>

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
#include <boost/lexical_cast.hpp>

int
pcl::PTXReader::tokenizeNextLine (std::ifstream &fs, std::string &line, std::vector<std::string> &st) const
{
  getline (fs, line);
  // Exit on empty lines
  if (line == "")
    return (-1);
  // Tokenize the line
  boost::trim (line);
  boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PTXWriter::setLockingPermissions (const std::string &file_name,
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
    PCL_DEBUG ("[pcl::PTXWriter::setLockingPermissions] File %s locked succesfully.\n", file_name.c_str ());
  else
    PCL_DEBUG ("[pcl::PTXWriter::setLockingPermissions] File %s could not be locked!\n", file_name.c_str ());

  namespace fs = boost::filesystem;
  fs::permissions (fs::path (file_name), fs::add_perms | fs::set_gid_on_exe);
#endif
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PTXWriter::resetLockingPermissions (const std::string &file_name,
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
int
pcl::PTXReader::readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
                            Eigen::Vector4d &origin, Eigen::Quaterniond &orientation, 
                            Eigen::Affine3d& transformation,
                            int &ptx_version, int &data_type, unsigned int &data_idx, const int offset)
{
  // Default values
  data_idx = 0;
  data_type = 0;
  ptx_version = PTX_V1;
  origin      = Eigen::Vector4d::Zero ();
  orientation = Eigen::Quaterniond::Identity ();
  transformation = Eigen::Affine3d::Identity ();
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  //cloud.is_dense = true;
  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] Could not find file '%s'.\n", file_name.c_str ());
    return (-1);
  }

  int nr_points = 0;
  std::ifstream fs;
  std::string line;

  // Open file in binary mode to avoid problem of 
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] Could not open file '%s'! Error : %s\n", file_name.c_str (), strerror(errno)); 
    fs.close ();
    return (-1);
  }

  // Seek at the given offset
  fs.seekg (offset, std::ios::beg);

  std::vector<std::string> st;  
  // Cloud width
  if (tokenizeNextLine (fs, line, st))
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] empty line!\n");
    fs.close ();
    return (-1);
  }
  try
  {  
    cloud.width = boost::lexical_cast<int>(st.at (0));
  }
  catch(const boost::bad_lexical_cast& e)
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] incorrect cloud width: %s\n", e.what ());
    return (-2);
  }
  // Cloud height
  if (tokenizeNextLine (fs, line, st))
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] empty line!\n");
    fs.close ();
    return (-1);
  }
  try
  {  
    cloud.height = boost::lexical_cast<int>(st.at (0));
  }
  catch(const boost::bad_lexical_cast& e)
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] incorrect cloud height: %s\n", e.what ());
    return (-2);
  }
  // If both height/width are not given, exit
  if (cloud.height == 0 && cloud.height == 0)
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] cloud height and width are null!\n");
    fs.close ();
    return (-1);
  }
  // Sensor position
  if (tokenizeNextLine (fs, line, st))
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] empty line!\n");
    fs.close ();
    return (-1);
  }
  if (st.size () != 3)
  {
    PCL_ERROR ("[pcl::PTXReader::readHeader] error reading sensor position!\n");
    fs.close ();
    return (-1);
  }
  for (std::size_t i = 0; i < st.size (); ++i)
  {
    try
    {  
      double tmp = boost::lexical_cast<double>(st.at (i));
      origin[i] = static_cast<float> (tmp);
    }
    catch(const boost::bad_lexical_cast& e)
    {
      PCL_ERROR ("[pcl::PTXReader::readHeader] incorrect position value: %s\n", e.what ());
      return (-2);
    }
  }
  std::cout <<  "origin " << origin.transpose () << std::endl;
  // Sensor orientation
  for (int l = 0; l < 3; ++l)
  {
    if (tokenizeNextLine (fs, line, st))
    {
      PCL_ERROR ("[pcl::PTXReader::readHeader] empty line!\n");
      fs.close ();
      return (-1);
    }
    if (st.size () != 3)
    {
      PCL_ERROR ("[pcl::PTXReader::readHeader] error reading sensor orientation line %d!\n", l);
      fs.close ();
      return (-1);
    }
    for (std::size_t i = 0; i < st.size (); ++i)
    {
      try
      {  
        double tmp = boost::lexical_cast<double>(st.at (i));
        orientation.matrix ().coeffRef (i,l) = static_cast<float> (tmp);
      }
      catch(const boost::bad_lexical_cast& e)
      {
        PCL_ERROR ("[pcl::PTXReader::readHeader] incorrect orientation value: %s\n", e.what ());
        return (-2);
      }
    }
  }
  std::cout <<  "orientation " << orientation.matrix () << std::endl;
  // Transformation matrix
  for (int l = 0; l < 4; ++l)
  {
    if (tokenizeNextLine (fs, line, st))
    {
      PCL_ERROR ("[pcl::PTXReader::readHeader] empty line!\n");
      fs.close ();
      return (-1);
    }
    if (st.size () != 4)
    {
      PCL_ERROR ("[pcl::PTXReader::readHeader] error reading transformation matrix line %d!\n", l);
      fs.close ();
      return (-1);
    }
    for (std::size_t i = 0; i < st.size (); ++i)
    {
      try
      {  
        double tmp = boost::lexical_cast<double>(st.at (i));
        transformation.matrix ().coeffRef (l,i) = static_cast<float> (tmp);
      }
      catch(const boost::bad_lexical_cast& e)
      {
        PCL_ERROR ("[pcl::PTXReader::readHeader] incorrect transformation matrix value: %s\n", e.what ());
        return (-2);
      }
    }
  }
  
  std::cout <<  "transformation " << transformation.matrix () << std::endl;
  
  data_idx = static_cast<int> (fs.tellg ());
  // For now assume only ASCII
  data_type = 0;
  // Close file
  fs.close ();

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PTXReader::read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                      Eigen::Vector4d &origin, Eigen::Quaterniond &orientation, Eigen::Affine3d& transformation,
                      int &ptx_version, const int offset)
{
  pcl::console::TicToc tt;
  tt.tic ();

  int data_type;
  unsigned int data_idx;

  int res = readHeader (file_name, cloud, origin, orientation, transformation, ptx_version, data_type, data_idx, offset);

  if (res < 0)
    return (res);

  unsigned int idx = 0;

  // Get the number of points the cloud should have
  unsigned int nr_points = cloud.width * cloud.height;

  // Setting the is_dense property to true by default
  cloud.is_dense = true;

  if (file_name == "" || !boost::filesystem::exists (file_name))
  {
    PCL_ERROR ("[pcl::PTXReader::read] Could not find file '%s'.\n", file_name.c_str ());
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
      PCL_ERROR ("[pcl::PTXReader::read] Could not open file %s.\n", file_name.c_str ());
      return (-1);
    }

    fs.seekg (data_idx);

    std::string line;
    std::vector<std::string> st;
    // Read the first line to assert the point type
    tokenizeNextLine (fs, line, st);
    
    switch (st.size ())
    {
      case 4:
      {
        // X Y Z I
        cloud.fields.resize (4);
        cloud.fields[0].name = "x"; 
        cloud.fields[0].offset = static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[0].count = 1;
        cloud.fields[1].name = "y";
        cloud.fields[1].offset = static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[1].count = 1;
        cloud.fields[2].name = "z";
        cloud.fields[2].offset = static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[2].count = 1;
        cloud.fields[3].name = "intensity";
        cloud.fields[3].offset = static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[3].count = 1;
        cloud.point_step = 4 * 1 * static_cast<pcl::uint32_t> (sizeof(float));
      }
      break;
      case 7:
      {
        std::cout << "x y z i r g b" << std::endl;
        // X Y Z I RGB
        cloud.fields.resize (5);
        cloud.fields[0].name = "x"; 
        cloud.fields[0].offset = 0;
        cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[0].count = 1;
        cloud.fields[1].name = "y";
        cloud.fields[1].offset = static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[1].count = 1;
        cloud.fields[2].name = "z";
        cloud.fields[2].offset = 2 * static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[2].count = 1;
        cloud.fields[3].name = "intensity";
        cloud.fields[3].offset = 3 * static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[3].count = 1;
        cloud.fields[4].name = "rgb";
        cloud.fields[4].offset = 4 * static_cast<pcl::uint32_t> (sizeof(float));
        cloud.fields[4].datatype = sensor_msgs::PointField::FLOAT32; 
        cloud.fields[4].count = 1;
        cloud.point_step = 5 * 1 * static_cast<pcl::uint32_t> (sizeof(float));
      }
      break;
      default:
      {
        PCL_ERROR ("[pcl::PTXReader::read] Unknown data type in file %s.\n", file_name.c_str ());
        return (-1);
      }
    }
    std::cout << "cloud.point_step " << cloud.point_step << std::endl;
    cloud.data.resize (nr_points * cloud.point_step);
    std::cout << "cloud data size " <<  cloud.data.size () << std::endl;
    std::cout << "cloud.fields.size () " << cloud.fields.size () << std::endl;
    std::size_t total = 0;
    // Copy first line of data
    for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
    {
      float value = 0;
      if (cloud.fields[d].name == "rgb")
      {
        total += cloud.fields[d].count; // jump over this many elements in the string token
        int r = boost::lexical_cast<int> (st.at (d));
        int g = boost::lexical_cast<int> (st.at (d+1));
        int b = boost::lexical_cast<int> (st.at (d+2));
        value = r << 16 | g << 8 | b;
      }
      else
      {
        value = boost::lexical_cast<float> (st.at (d));
        // copyStringValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT32>::type> (st.at (total), cloud, idx, d, 1);
      }
      std::cout << "value " << value << std::endl;
      std::cout << "address " << idx * cloud.point_step + cloud.fields[d].offset << std::endl;
      memcpy (&cloud.data[idx * cloud.point_step + cloud.fields[d].offset], 
              reinterpret_cast<char*> (&value), sizeof (float));
      total += cloud.fields[d].count; // jump over this many elements in the string token
    }
    idx++;
    
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
          PCL_WARN ("[pcl::PTXReader::read] input file %s has more points (%d) than advertised (%d)!\n", file_name.c_str (), idx, nr_points);
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
      PCL_ERROR ("[pcl::PTXReader::read] %s\n", exception);
      fs.close ();
      return (-1);
    }

    // Close file
    fs.close ();
  }

  double total_time = tt.toc ();
  PCL_DEBUG ("[pcl::PTXReader::read] Loaded %s as a %s cloud in %g ms with %d points. Available dimensions: %s.\n", 
             file_name.c_str (), cloud.is_dense ? "dense" : "non-dense", total_time, 
             cloud.width * cloud.height, pcl::getFieldsList (cloud).c_str ());
  return (0);
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// int
// pcl::PTXReader::read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, const int offset)
// {
//   int ptx_version;
//   Eigen::Vector4f origin;
//   Eigen::Quaternionf orientation;
//   Eigen::Affine3d transformation;
//   // Load the data
//   int res = read (file_name, cloud, origin, orientation, transformation, ptx_version, offset);

//   if (res < 0)
//     return (res);

//   return (0);
// }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PTXWriter::generateHeaderASCII (const sensor_msgs::PointCloud2 &cloud, 
                                     const Eigen::Vector4d &origin, 
                                     const Eigen::Quaterniond &orientation, 
                                     const Eigen::Affine3d& transformation)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << cloud.width << "\n" << cloud.height << "\n";
  oss << origin[0] << " " << origin[1] << " " << origin[2] << "\n";
  const Eigen::Matrix3d &R = orientation.toRotationMatrix ();
  oss << R.coeff (0,0) << " " << R.coeff (1,0) << " " << R.coeff (2,0) << "\n";
  oss << R.coeff (0,1) << " " << R.coeff (1,1) << " " << R.coeff (2,1) << "\n";
  oss << R.coeff (0,2) << " " << R.coeff (1,2) << " " << R.coeff (2,2) << "\n";
  oss << transformation.matrix () << "\n";
  return (oss.str ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::PTXWriter::generateHeaderBinary (const sensor_msgs::PointCloud2 &cloud, 
                                      const Eigen::Vector4d &origin, 
                                      const Eigen::Quaterniond &orientation, 
                                      const Eigen::Affine3d& transformation)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << cloud.width << "\n" << cloud.height << "\n";
  oss << origin[0] << " " << origin[1] << " " << origin[2] << "\n";
  Eigen::Matrix3d R = orientation.toRotationMatrix ();
  oss << R.coeff (0,0) << " " << R.coeff (1,0) << " " << R.coeff (2,0) << "\n";
  oss << R.coeff (0,1) << " " << R.coeff (1,1) << " " << R.coeff (2,1) << "\n";
  oss << R.coeff (0,2) << " " << R.coeff (1,2) << " " << R.coeff (2,2) << "\n";
  oss << transformation.matrix () << "\n";
  return (oss.str ());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PTXWriter::writeASCII (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                            const Eigen::Vector4d &origin, const Eigen::Quaterniond &orientation,
                            const Eigen::Affine3d &transformation, const int precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PTXWriter::writeASCII] Input point cloud has no data!\n");
    return (-1);
  }

  std::ofstream fs;
  fs.precision (precision);
  fs.imbue (std::locale::classic ());
  fs.open (file_name.c_str ());      // Open file
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR("[pcl::PTXWriter::writeASCII] Could not open file '%s' for writing! Error : %s\n", file_name.c_str (), strerror(errno)); 
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  int nr_points  = cloud.width * cloud.height;
  int point_size = static_cast<int> (cloud.data.size () / nr_points);

  // Write the header information
  fs << generateHeaderASCII (cloud, origin, orientation, transformation) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());

  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (unsigned int d = 0; d < static_cast<unsigned int> (cloud.fields.size ()); ++d)
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
            PCL_WARN ("[pcl::PTXWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
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
pcl::PTXWriter::writeBinary (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                             const Eigen::Vector4d &origin, const Eigen::Quaterniond &orientation,
                             const Eigen::Affine3d &transformation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Input point cloud has no data!\n");
    return (-1);
  }
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << generateHeaderBinary (cloud, origin, orientation, transformation) << "DATA binary\n";
  oss.flush();
  data_idx = static_cast<unsigned int> (oss.tellp ());

#ifdef _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Error during CreateFile (%s)!\n", file_name.c_str());
    return (-1);
  }
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
  {
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Error during open (%s)!\n", file_name.c_str());
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
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Error during write ()!\n");
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
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Error during mmap ()!\n");
    return (-1);
  }
#endif

  // copy header
  memcpy (&map[0], oss.str().c_str (), static_cast<size_t> (data_idx));

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
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] Error during munmap ()!\n");
    return (-1);
  }
#endif
  // Close file
#ifdef _WIN32
  CloseHandle(h_native_file);
#else
  pcl_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PTXWriter::writeBinaryCompressed (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                                       const Eigen::Vector4d &origin, const Eigen::Quaterniond &orientation, const Eigen::Affine3d& transformation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Input point cloud has no data!\n");
    return (-1);
  }
  std::streamoff data_idx = 0;
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  oss << generateHeaderBinary (cloud, origin, orientation, transformation);
  oss.flush ();
  data_idx = oss.tellp ();

#ifdef _WIN32
  HANDLE h_native_file = CreateFile (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Error during CreateFile (%s)!\n", file_name.c_str ());
    return (-1);
  }
#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
  {
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Error during open (%s)!\n", file_name.c_str());
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
    if (cloud.fields[i].name == "_")
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
    throw pcl::IOException ("[pcl::PTXWriter::writeBinaryCompressed] Error during compression!");
    return (-1);
  }

#ifndef _WIN32
  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Error during lseek ()!\n");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Error during write ()!\n");
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
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Error during mmap ()!\n");
    return (-1);
  }
#endif

  // copy header
  memcpy (&map[0], oss.str ().c_str (), static_cast<size_t> (data_idx));
  // Copy the compressed data
  memcpy (&map[data_idx], temp_buf, data_size);

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
    PCL_ERROR ("[pcl::PTXWriter::writeBinaryCompressed] Error during munmap ()!\n");
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

