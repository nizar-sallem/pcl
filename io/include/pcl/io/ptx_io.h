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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_IO_PTX_IO_H_
#define PCL_IO_PTX_IO_H_

#include <pcl/point_cloud.h>
#include <pcl/io/file_io.h>
#include <pcl/common/transforms.h>

namespace pcl
{
  /** \brief Point Cloud Data (PTX) file format reader.
    * \author Radu B. Rusu
    * \ingroup io
    */
  class PCL_EXPORTS PTXReader : public FileReader
  {
    public:
      /** Empty constructor */
      PTXReader () : FileReader () {}
      /** Empty destructor */
      ~PTXReader () {}

      /** \brief Various PTX file versions.
        *
        * PTX represents PTX files, which contain the following fields:
        *   - width
        *   - height
        *   - scanner position (three double precision)
        *   - scanner orientation along X axis (three double precision)
        *   - scanner orientation along Y axis (three double precision)
        *   - scanner orientation along Z axis (three double precision)
        *   - transformation matrix row 1 (four double precision)
        *   - transformation matrix row 2 (four double precision)
        *   - transformation matrix row 3 (four double precision)
        *   - transformation matrix row 4 (four double precision)
        *   - ASCII/BINARY
        *   - data
        * 
        * Everything that follows \b ASCII/BINARY is intepreted as data points and
        * will be read accordingly.
        *
        */
      enum
      {
        PTX_V1 = 0,
        PTX_V2 = 1
      };

      /** \brief Read a point cloud data header from a PTX file. 
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PTX file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \attention The PTX data is \b always stored in ROW major format! The
        * read/write PTX methods will detect column major input and automatically convert it.
        *
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
        * \param[out] origin the sensor acquisition origin (only for > PTX_V7 - null if not present)
        * \param[out] orientation the sensor acquisition orientation (only for > PTX_V7 - identity if not present)
        * \param[out] transformation transformation matrix from UCS
        * \param[out] ptx_version the PTX version of the file (i.e., PTX_V6, PTX_V7)
        * \param[out] data_type the type of data (0 = ASCII, 1 = Binary, 2 = Binary compressed) 
        * \param[out] data_idx the offset of cloud data within the file
        * \param[in] offset the offset of where to expect the PTX Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PTX files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int 
      readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
                  Eigen::Vector4d &origin, Eigen::Quaterniond &orientation, 
                  Eigen::Affine3d &transformation, int &ptx_version,
                  int &data_type, unsigned int &data_idx, const int offset = 0);

      int 
      readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
                  int &ptx_version, int &data_type, unsigned int &data_idx, const int offset = 0)
      {
        Eigen::Vector4d origin_d;
        Eigen::Quaterniond orientation_d;
        Eigen::Affine3d transformation;
        int res = readHeader (file_name, cloud, origin_d, orientation_d, transformation, 
                              ptx_version, data_type, data_idx, offset);
        origin = origin_d.cast<float> ();
        orientation = orientation_d.cast<float> ();
        return (res);
      }
      

      /** \brief Read a point cloud data header from a PTX file. 
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PTX file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \attention The PTX data is \b always stored in ROW major format! The
        * read/write PTX methods will detect column major input and automatically convert it.
        *
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
        * \param[in] offset the offset of where to expect the PTX Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PTX files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int 
      readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, const int offset = 0);

      /** \brief Read a point cloud data from a PTX file and store it into a sensor_msgs/PointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[out] origin the sensor acquisition origin
        * \param[out] orientation the sensor acquisition orientation
        * \param[out] ptx_version the PTX version of the file (either PTX_V6 or PTX_V7)
        * \param[in] offset the offset of where to expect the PTX Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PTX files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      int 
      read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
            Eigen::Vector4d &origin, Eigen::Quaterniond &orientation, Eigen::Affine3d& transform,
            int &ptx_version, const int offset = 0);

      int 
      read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud, 
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, 
            int &ptx_version, const int offset)
      {
        Eigen::Vector4d origin_d;
        Eigen::Quaterniond orientation_d;
        Eigen::Affine3d transformation;
        int res = read (file_name, cloud, origin_d, orientation_d, transformation, ptx_version, offset);
        origin = origin_d.cast<float> ();
        orientation = orientation_d.cast<float> ();
        return (res);
      }
      
      /** \brief Read a point cloud data from any PTX file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset of where to expect the PTX Header in the
        * file (optional parameter). One usage example for setting the offset
        * parameter is for reading data from a TAR "archive containing multiple
        * PTX files: TAR files always add a 512 byte header in front of the
        * actual file, so set the offset to the next byte after the header
        * (e.g., 513).
        *
        * \return
        *  * < 0 (-1) on error
        *  * == 0 on success
        */
      template<typename PointT> int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud, const int offset = 0)
      {
        sensor_msgs::PointCloud2 blob;
        Eigen::Affine3d transformation;
        int ptx_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_, transformation,
                        ptx_version, offset);

        // If no error, convert the data and transform
        if (res == 0)
        {
          pcl::fromROSMsg (blob, cloud);
          transformPointCloud<PointT, double> (cloud, cloud, transformation);
        } 
        return (res);
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      private:
      int
      tokenizeNextLine (std::ifstream &fs, std::string &line, std::vector<std::string> &st) const;
  };

  /** \brief Point Cloud Data (PTX) file format writer.
    * \author Radu Bogdan Rusu
    * \ingroup io
    */
  class PCL_EXPORTS PTXWriter : public FileWriter
  {
    public:
      PTXWriter() : FileWriter(), map_synchronization_(false) {}
      ~PTXWriter() {}

      /** \brief Set whether mmap() synchornization via msync() is desired before munmap() calls. 
        * Setting this to true could prevent NFS data loss (see
        * http://www.pcl-developers.org/PTX-IO-consistency-on-NFS-msync-needed-td4885942.html).
        * Default: false
        * \note This option should be used by advanced users only!
        * \note Please note that using msync() on certain systems can reduce the I/O performance by up to 80%!
        * \param[in] sync set to true if msync() should be called before munmap()
        */
      void
      setMapSynchronization (bool sync)
      {
        map_synchronization_ = sync;
      }

      /** \brief Generate the header of a PTX file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderBinary (const sensor_msgs::PointCloud2 &cloud, 
                            const Eigen::Vector4d &origin, 
                            const Eigen::Quaterniond &orientation,
                            const Eigen::Affine3d &transformation);

      /** \brief Generate the header of a BINARY_COMPRESSED PTX file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderBinaryCompressed (const sensor_msgs::PointCloud2 &cloud, 
                                      const Eigen::Vector4d &origin, 
                                      const Eigen::Quaterniond &orientation,
                                      const Eigen::Affine3d &transformation);

      /** \brief Generate the header of a PTX file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      std::string
      generateHeaderASCII (const sensor_msgs::PointCloud2 &cloud, 
                           const Eigen::Vector4d &origin, 
                           const Eigen::Quaterniond &orientation,
                           const Eigen::Affine3d &transformation);

      /** \brief Generate the header of a PTX file format
        * \param[in] cloud the point cloud data message
        * \param[in] nr_points if given, use this to fill in WIDTH, HEIGHT (=1), and POINTS in the header
        * By default, nr_points is set to INTMAX, and the data in the header is used instead.
        */
      template <typename PointT> static std::string
      generateHeader (const pcl::PointCloud<PointT> &cloud, 
                      const Eigen::Affine3d &transformation,
                      const int nr_points = std::numeric_limits<int>::max ());

      /** \brief Save point cloud data to a PTX file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] precision the specified output numeric stream precision (default: 8)
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        *
        * As an intermediary solution, precision 8 is used, which guarantees lossless storage for RGB.
        */
      int 
      writeASCII (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                  const Eigen::Vector4d &origin = Eigen::Vector4d::Zero (), 
                  const Eigen::Quaterniond &orientation = Eigen::Quaternionf::Identity (),
                  const Eigen::Affine3d &transformation = Eigen::Affine3d::Identity (),
                  const int precision = 6);

      /** \brief Save point cloud data to a PTX file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      int 
      writeBinary (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                   const Eigen::Vector4d &origin = Eigen::Vector4d::Zero (), 
                   const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity (),
                   const Eigen::Affine3d &transformation = Eigen::Affine3d::Identity ());

      /** \brief Save point cloud data to a PTX file containing n-D points, in BINARY_COMPRESSED format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        */
      int 
      writeBinaryCompressed (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
                             const Eigen::Vector4d &origin = Eigen::Vector4d::Zero (), 
                             const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity (),
                             const Eigen::Affine3d &transformation = Eigen::Affine3d::Identity ());

      /** \brief Save point cloud data to a PTX file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PTX format, false (default) for ASCII
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        *
        * As an intermediary solution, precision 8 is used, which guarantees lossless storage for RGB.
        */
      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
             const Eigen::Vector4d &origin = Eigen::Vector4d::Zero (), 
             const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity (),
             const Eigen::Affine3d &transformation = Eigen::Affine3d::Identity (),
             const bool binary = false)
      {
        if (binary)
          return (writeBinary (file_name, cloud, origin, orientation, transformation));
        else
          return (writeASCII (file_name, cloud, origin, orientation, transformation, 6));
      }

      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false)
      {
        const Eigen::Vector4d &origin_d = origin.cast<double> ();
        const Eigen::Quaterniond &orientation_d = orientation.cast<double> ();
        if (binary)
          return (writeBinary (file_name, cloud, origin_d, orientation_d, Eigen::Affine3d::Identity ()));
        else
          return (writeASCII (file_name, cloud, origin_d, orientation_d, Eigen::Affine3d::Identity (), 6));
      }

      /** \brief Save point cloud data to a PTX file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message (boost shared pointer)
        * \param[in] binary set to true if the file is to be written in a binary PTX format, 
        * false (default) for ASCII
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        */
      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2::ConstPtr &cloud, 
             const Eigen::Vector4d &origin = Eigen::Vector4d::Zero (), 
             const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity (),
             const Eigen::Affine3d &transformation = Eigen::Affine3d::Identity (),
             const bool binary = false)
      {
        return (write (file_name, *cloud, origin, orientation, transformation, binary));
      }

      inline int
      write (const std::string &file_name, const sensor_msgs::PointCloud2::ConstPtr &cloud, 
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false)
      {
        const Eigen::Vector4d &origin_d = origin.cast<double> ();
        const Eigen::Quaterniond &orientation_d = orientation.cast<double> ();
        return (write (file_name, cloud, origin_d, orientation_d, Eigen::Affine3d::Identity (), binary));
      }
  
      /** \brief Save point cloud data to a PTX file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        */
      template <typename PointT> int 
      writeBinary (const std::string &file_name, 
                   const pcl::PointCloud<PointT> &cloud,
                   const Eigen::Affine3d& transformation);

      /** \brief Save point cloud data to a binary comprssed PTX file
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        */
      template <typename PointT> int 
      writeBinaryCompressed (const std::string &file_name, 
                             const pcl::PointCloud<PointT> &cloud,
                             const Eigen::Affine3d& transformation);

      /** \brief Save point cloud data to a PTX file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] indices the set of point indices that we want written to disk
        */
      template <typename PointT> int 
      writeBinary (const std::string &file_name, 
                   const pcl::PointCloud<PointT> &cloud,
                   const Eigen::Affine3d& transformation, 
                   const std::vector<int> &indices);

      /** \brief Save point cloud data to a PTX file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] precision the specified output numeric stream precision (default: 8)
        */
      template <typename PointT> int 
      writeASCII (const std::string &file_name, 
                  const pcl::PointCloud<PointT> &cloud,
                  const Eigen::Affine3d& transformation,
                  const int precision = 8);

       /** \brief Save point cloud data to a PTX file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] indices the set of point indices that we want written to disk
        * \param[in] precision the specified output numeric stream precision (default: 8)
        */
      template <typename PointT> int 
      writeASCII (const std::string &file_name, 
                  const pcl::PointCloud<PointT> &cloud,
                  const Eigen::Affine3d& transformation,
                  const std::vector<int> &indices,
                  const int precision = 8);

      /** \brief Save point cloud data to a PTX file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] binary set to true if the file is to be written in a binary
        * PTX format, false (default) for ASCII
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        */
      template<typename PointT> inline int
      write (const std::string &file_name, 
             const pcl::PointCloud<PointT> &cloud, 
             const bool binary = false)
      {
        if (binary)
          return (writeBinary<PointT> (file_name, cloud, Eigen::Affine3d::Identity ()));
        else
          return (writeASCII<PointT> (file_name, cloud, Eigen::Affine3d::Identity ()));
      }

      /** \brief Save point cloud data to a PTX file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] indices the set of point indices that we want written to disk
        * \param[in] binary set to true if the file is to be written in a binary
        * PTX format, false (default) for ASCII
        *
        * Caution: PointCloud structures containing an RGB field have
        * traditionally used packed float values to store RGB data. Storing a
        * float as ASCII can introduce variations to the smallest bits, and
        * thus significantly alter the data. This is a known issue, and the fix
        * involves switching RGB data to be stored as a packed integer in
        * future versions of PCL.
        */
      template<typename PointT> inline int
      write (const std::string &file_name, 
             const pcl::PointCloud<PointT> &cloud, 
             const std::vector<int> &indices,
             bool binary = false)
      {
        if (binary)
          return (writeBinary<PointT> (file_name, cloud, Eigen::Affine3d::Identity (), indices));
        else
          return (writeASCII<PointT> (file_name, cloud, Eigen::Affine3d::Identity (), indices));
      }

    protected:
      /** \brief Set permissions for file locking (Boost 1.49+).
        * \param[in] file_name the file name to set permission for file locking
        * \param[in,out] lock the file lock
        */
      void
      setLockingPermissions (const std::string &file_name,
                             boost::interprocess::file_lock &lock);

      /** \brief Reset permissions for file locking (Boost 1.49+).
        * \param[in] file_name the file name to reset permission for file locking
        * \param[in,out] lock the file lock
        */
      void
      resetLockingPermissions (const std::string &file_name,
                               boost::interprocess::file_lock &lock);

    private:
      /** \brief Set to true if msync() should be called before munmap(). Prevents data loss on NFS systems. */
      bool map_synchronization_;
  };

  namespace io
  {
    /** \brief Load any PTX file into a templated PointCloud type.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \param[out] origin the sensor acquisition origin (only for > PTX_V7 - null if not present)
      * \param[out] orientation the sensor acquisition orientation (only for >
      * PTX_V7 - identity if not present)
      * \ingroup io
      */
    inline int 
    loadPTXFile (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                 Eigen::Vector4d &origin, Eigen::Quaterniond &orientation, Eigen::Affine3d transformation)
    {
      pcl::PTXReader p;
      int ptx_version;
      return (p.read (file_name, cloud, origin, orientation, transformation, ptx_version));
    }

    /** \brief Load any PTX file into a templated PointCloud type
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \ingroup io
      */
    template<typename PointT> inline int
    loadPTXFile (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
    {
      pcl::PTXReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Save point cloud data to a PTX file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor acquisition origin
      * \param[in] orientation the sensor acquisition orientation
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    inline int 
    savePTXFile (const std::string &file_name, const sensor_msgs::PointCloud2 &cloud, 
                 const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                 const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                 const bool binary_mode = false)
    {
      PTXWriter w;
      return (w.write (file_name, cloud, origin, orientation, binary_mode));
    }

    /** \brief Templated version for saving point cloud data to a PTX file
      * containing a specific given cloud format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    template<typename PointT> inline int
    savePTXFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, bool binary_mode = false)
    {
      PTXWriter w;
      return (w.write<PointT> (file_name, cloud, binary_mode));
    }

    /** 
      * \brief Templated version for saving point cloud data to a PTX file
      * containing a specific given cloud format.
      *
      *      This version is to retain backwards compatibility.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    template<typename PointT> inline int
    savePTXFileASCII (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PTXWriter w;
      return (w.write<PointT> (file_name, cloud, false));
    }

    /** 
      * \brief Templated version for saving point cloud data to a PTX file
      * containing a specific given cloud format. The resulting file will be an uncompressed binary.
      *
      *      This version is to retain backwards compatibility.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePTXFileBinary (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PTXWriter w;
      return (w.write<PointT> (file_name, cloud, true));
    }

    /** 
      * \brief Templated version for saving point cloud data to a PTX file
      * containing a specific given cloud format
      *
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] indices the set of indices to save
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      *
      * Caution: PointCloud structures containing an RGB field have
      * traditionally used packed float values to store RGB data. Storing a
      * float as ASCII can introduce variations to the smallest bits, and
      * thus significantly alter the data. This is a known issue, and the fix
      * involves switching RGB data to be stored as a packed integer in
      * future versions of PCL.
      * \ingroup io
      */
    template<typename PointT> int
    savePTXFile (const std::string &file_name, 
                 const pcl::PointCloud<PointT> &cloud,
                 const std::vector<int> &indices, 
                 const bool binary_mode = false)
    {
      // Save the data
      PTXWriter w;
      return (w.write<PointT> (file_name, cloud, indices, binary_mode));
    }


    /**
      * \brief Templated version for saving point cloud data to a PTX file
      * containing a specific given cloud format. This method will write a compressed binary file.
      *
      *      This version is to retain backwards compatibility.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePTXFileBinaryCompressed (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PTXWriter w;
      return (w.writeBinaryCompressed<PointT> (file_name, cloud));
    }

  }
}

#include <pcl/io/impl/ptx_io.hpp>

#endif  //#ifndef PCL_IO_PTX_IO_H_
