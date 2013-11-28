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

#ifndef PCL_IO_PTX_IO_IMPL_H_
#define PCL_IO_PTX_IO_IMPL_H_

#include <fstream>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <pcl/io/boost.h>
#include <pcl/console/print.h>
#include <pcl/io/ptx_io.h>

#ifdef _WIN32
# include <io.h>
# ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
# endif // WIN32_LEAN_AND_MEAN
# ifndef NOMINMAX
#  define NOMINMAX
# endif // NOMINMAX
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

#include <pcl/io/lzf.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::string
pcl::PTXWriter::generateHeader (const pcl::PointCloud<PointT> &cloud,
                                const Eigen::Affine3d& transformation,
                                const int nr_points)
{
  std::ostringstream oss;
  oss.imbue (std::locale::classic ());

  // If the user passes in a number of points value, use that instead
  if (nr_points != std::numeric_limits<int>::max ())
    oss << nr_points << "\n" << 1 << "\n";
  else
    oss << cloud.width << "\n" << cloud.height << "\n";
  // Write out sensor position
  oss << cloud.sensor_origin_[0] << " " << cloud.sensor_origin_[1] << " " << cloud.sensor_origin_[2] << 1 << "\n";
  // Write out sensor orientation
  Eigen::Matrix3d R = cloud.sensor_orientation_.toRotationMatrix ();
  oss << R.coeff (0,0) << " " << R.coeff (1,0) << " " << R.coeff (2,0) << 0 << "\n";
  oss << R.coeff (0,1) << " " << R.coeff (1,1) << " " << R.coeff (2,1) << 0 << "\n";
  oss << R.coeff (0,2) << " " << R.coeff (1,2) << " " << R.coeff (2,2) << 0 << "\n";
  // Write out transformation matrix
  oss << transformation.matrix () << "\n";
  // Return the header
  return (oss.str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PTXWriter::writeBinary (const std::string &file_name, 
                             const pcl::PointCloud<PointT> &cloud,
                             const Eigen::Affine3d& transformation)
{
  if (cloud.empty ())
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Input point cloud has no data!");
    return (-1);
  }
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeader<PointT> (cloud, transformation) << "binary\n";
  oss.flush ();
  data_idx = static_cast<int> (oss.tellp ());

#if _WIN32
  HANDLE h_native_file = CreateFileA (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during CreateFile!");
    return (-1);
  }
#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during open!");
    return (-1);
  }
#endif
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  std::vector<pcl::PCLPointField> fields;
  std::vector<int> fields_sizes;
  size_t fsize = 0;
  size_t data_size = 0;
  size_t nri = 0;
  pcl::getFields (cloud, fields);
  // Compute the total size of the fields
  for (size_t i = 0; i < fields.size (); ++i)
  {
    if (fields[i].name == "_")
      continue;
    
    int fs = fields[i].count * getFieldSize (fields[i].datatype);
    fsize += fs;
    fields_sizes.push_back (fs);
    fields[nri++] = fields[i];
  }
  fields.resize (nri);
  
  data_size = cloud.points.size () * fsize;

  // Prepare the map
#if _WIN32
  HANDLE fm = CreateFileMappingA (h_native_file, NULL, PAGE_READWRITE, 0, (DWORD) (data_idx + data_size), NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
  CloseHandle (fm);

#else
  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET);

  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] lseek errno: %d strerror: %s\n", errno, strerror (errno));

    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during lseek ()!");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during write ()!");
    return (-1);
  }

  char *map = static_cast<char*> (mmap (0, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during mmap ()!");
    return (-1);
  }
#endif

  // Copy the header
  memcpy (&map[0], oss.str ().c_str (), data_idx);

  // Copy the data
  char *out = &map[0] + data_idx;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    int nrj = 0;
    for (size_t j = 0; j < fields.size (); ++j)
    {
      memcpy (out, reinterpret_cast<const char*> (&cloud.points[i]) + fields[j].offset, fields_sizes[nrj]);
      out += fields_sizes[nrj++];
    }
  }

  // If the user set the synchronization flag on, call msync
#if !_WIN32
  if (map_synchronization_)
    msync (map, data_idx + data_size, MS_SYNC);
#endif

  // Unmap the pages of memory
#if _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, (data_idx + data_size)) == -1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during munmap ()!");
    return (-1);
  }
#endif
  // Close file
#if _WIN32
  CloseHandle (h_native_file);
#else
  pcl_close (fd);
#endif
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PTXWriter::writeASCII (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, 
                            const Eigen::Affine3d& transformation, const int precision)
{
  if (cloud.empty ())
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeASCII] Input point cloud has no data!");
    return (-1);
  }

  if (cloud.width * cloud.height != cloud.points.size ())
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeASCII] Number of points different than width * height!");
    return (-1);
  }

  std::ofstream fs;
  fs.open (file_name.c_str ());      // Open file
  
  if (!fs.is_open () || fs.fail ())
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeASCII] Could not open file for writing!");
    return (-1);
  }
  
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  fs.precision (precision);
  fs.imbue (std::locale::classic ());

  std::vector<pcl::PCLPointField> fields;
  pcl::getFields (cloud, fields);

  // Write the header information
  fs << generateHeader<PointT> (cloud, transformation) << "DATA ascii\n";

  std::ostringstream stream;
  stream.precision (precision);
  stream.imbue (std::locale::classic ());
  // Iterate through the points
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    for (size_t d = 0; d < fields.size (); ++d)
    {
      // Ignore invalid padded dimensions that are inherited from binary data
      if (fields[d].name == "_")
        continue;

      int count = fields[d].count;
      if (count == 0) 
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)

      for (int c = 0; c < count; ++c)
      {
        switch (fields[d].datatype)
        {
          case pcl::PCLPointField::INT8:
          {
            int8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (int8_t), sizeof (int8_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT8:
          {
            uint8_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (uint8_t), sizeof (uint8_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::INT32:
          {
            int32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (int32_t), sizeof (int32_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<int32_t>(value);
            break;
          }
          case pcl::PCLPointField::UINT32:
          {
            uint32_t value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (uint32_t), sizeof (uint32_t));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<uint32_t>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT32:
          {
            float value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (float), sizeof (float));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<float>(value);
            break;
          }
          case pcl::PCLPointField::FLOAT64:
          {
            double value;
            memcpy (&value, reinterpret_cast<const char*> (&cloud.points[i]) + fields[d].offset + c * sizeof (double), sizeof (double));
            if (pcl_isnan (value))
              stream << "nan";
            else
              stream << boost::numeric_cast<double>(value);
            break;
          }
          default:
            PCL_WARN ("[pcl::PTXWriter::writeASCII] Incorrect field data type specified (%d)!\n", fields[d].datatype);
            break;
        }

        if (d < fields.size () - 1 || c < static_cast<int> (fields[d].count - 1))
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::PTXWriter::writeBinary (const std::string &file_name, 
                             const pcl::PointCloud<PointT> &cloud,
                             const Eigen::Affine3d &transformation,
                             const std::vector<int> &indices)
{
  if (cloud.points.empty () || indices.empty ())
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Input point cloud has no data or empty indices given!");
    return (-1);
  }
  int data_idx = 0;
  std::ostringstream oss;
  oss << generateHeader<PointT> (cloud, static_cast<int> (indices.size ()), transformation) << "DATA binary\n";
  oss.flush ();
  data_idx = static_cast<int> (oss.tellp ());

#if _WIN32
  HANDLE h_native_file = CreateFileA (file_name.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during CreateFile!");
    return (-1);
  }
#else
  int fd = pcl_open (file_name.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
  {
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during open!");
    return (-1);
  }
#endif
  // Mandatory lock file
  boost::interprocess::file_lock file_lock;
  setLockingPermissions (file_name, file_lock);

  std::vector<pcl::PCLPointField> fields;
  std::vector<int> fields_sizes;
  size_t fsize = 0;
  size_t data_size = 0;
  size_t nri = 0;
  pcl::getFields (cloud, fields);
  // Compute the total size of the fields
  for (size_t i = 0; i < fields.size (); ++i)
  {
    if (fields[i].name == "_")
      continue;
    
    int fs = fields[i].count * getFieldSize (fields[i].datatype);
    fsize += fs;
    fields_sizes.push_back (fs);
    fields[nri++] = fields[i];
  }
  fields.resize (nri);
  
  data_size = indices.size () * fsize;

  // Prepare the map
#if _WIN32
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, data_idx + data_size, NULL);
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_idx + data_size));
  CloseHandle (fm);

#else
  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, getpagesize () + data_size - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    PCL_ERROR ("[pcl::PTXWriter::writeBinary] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during lseek ()!");
    return (-1);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during write ()!");
    return (-1);
  }

  char *map = static_cast<char*> (mmap (0, data_idx + data_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1)) //MAP_FAILED)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during mmap ()!");
    return (-1);
  }
#endif

  // Copy the header
  memcpy (&map[0], oss.str ().c_str (), data_idx);

  char *out = &map[0] + data_idx;
  // Copy the data
  for (size_t i = 0; i < indices.size (); ++i)
  {
    int nrj = 0;
    for (size_t j = 0; j < fields.size (); ++j)
    {
      memcpy (out, reinterpret_cast<const char*> (&cloud.points[indices[i]]) + fields[j].offset, fields_sizes[nrj]);
      out += fields_sizes[nrj++];
    }
  }

#if !_WIN32
  // If the user set the synchronization flag on, call msync
  if (map_synchronization_)
    msync (map, data_idx + data_size, MS_SYNC);
#endif

  // Unmap the pages of memory
#if _WIN32
    UnmapViewOfFile (map);
#else
  if (munmap (map, (data_idx + data_size)) == -1)
  {
    pcl_close (fd);
    resetLockingPermissions (file_name, file_lock);
    throw pcl::IOException ("[pcl::PTXWriter::writeBinary] Error during munmap ()!");
    return (-1);
  }
#endif
  // Close file
#if _WIN32
  CloseHandle(h_native_file);
#else
  pcl_close (fd);
#endif
  
  resetLockingPermissions (file_name, file_lock);
  return (0);
}

#endif  //#ifndef PCL_IO_PTX_IO_H_

