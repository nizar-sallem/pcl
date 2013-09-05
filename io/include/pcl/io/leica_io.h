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
 */
#ifndef PCL_IO_LEICA_IO_H
#define PCL_IO_LEICA_IO_H

#include <pcl/io/leica/ptx_cloud.h>
#include <pcl/io/leica/ptx_cloud_data.h>
#include <pcl/io/file_io.h>

namespace leica
{
  
  class PTXReader
  {
    public:
      PTXReader () {}
      ~PTXReader () {}
    
      /** \brief Various PTX file versions.
        *
        * PTX represents PTX files, which contain the following fields:
        *   - width
        *   - height
        *   - scanner position (three simple precision)
        *   - scanner orientation along X axis (three simple precision)
        *   - scanner orientation along Y axis (three simple precision)
        *   - scanner orientation along Z axis (three simple precision)
        *   - transformation matrix row 1 (four simple precision)
        *   - transformation matrix row 2 (four simple precision)
        *   - transformation matrix row 3 (four simple precision)
        *   - transformation matrix row 4 (four simple precision)
        *   - data_type (binary, ascii, binary_compressed)
        *   - image_offset (-1 if no image is present)
        *   - image_encoding (bgr8, bgr8_lzf, jp2k) present only if image
        * 
        * Everything that follows image_encoding is intepreted as data
        * points (optinnally folloed by image) and will be read
        * accordingly.
        */

      int 
      readHeader (const std::string &file_name, sensor_msgs::PTXCloudData &cloud, 
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, Eigen::Affine3f& transformation,
                  int &data_type, std::size_t &data_idx,
                  std::size_t &image_idx, int& image_type, std::size_t& image_size);
      int
      read (const std::string &file_name, sensor_msgs::PTXCloudData &cloud,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, Eigen::Affine3f& transformation,
            int &ptx_version, const int offset = 0);

      int
      readBinary (const std::string& file_name, sensor_msgs::PTXCloudData& cloud, 
                  int data_type, std::size_t data_offset, 
                  int image_type, std::size_t image_size,
                  unsigned int nr_points);
    
    private:
      /** Tokenizes file \a fs line \a line into a vector of strings \a st possibly
        * checking length.
        * \return 0 on success and negative value otherwise.
        * \param[in/out] fs file stream to be read
        * \param[out] read file line
        * \param[out] st the tokeized line
        * \param[in] supposed number of tokens in line
        */
      int
      tokenizeNextLine (std::ifstream &fs, std::string &line, std::vector<std::string> &st, 
                        std::size_t length_control = 0) const;
    
  };

  class PTXWriter
  {
    public:
      PTXWriter ()
        : map_synchronization_(false) {}

      ~PTXWriter () {}

      /** \brief Set whether mmap() synchornization via msync() is desired before munmap() calls. 
        * Setting this to true could prevent NFS data loss (see
        * http://www.pcl-developers.org/PCD-IO-consistency-on-NFS-msync-needed-td4885942.html).
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

      std::string writeHeader (const sensor_msgs::PTXCloudData& cloud, 
                               const Eigen::Vector4d& origin, 
                               const Eigen::Quaterniond& orientation, 
                               const Eigen::Affine3d& transformation, 
                               int data_type, 
                               bool& with_rgb);
    

      int writeASCII (const std::string &file_name, 
                      const sensor_msgs::PTXCloudData &cloud, 
                      const Eigen::Vector4d &origin, 
                      const Eigen::Quaterniond &orientation, 
                      const Eigen::Affine3d& transformation,
                      int precision);
    
      int writeBinary (const std::string &file_name, 
                       const sensor_msgs::PTXCloudData &cloud, 
                       const Eigen::Vector4d &origin, 
                       const Eigen::Quaterniond &orientation, 
                       const Eigen::Affine3d& transformation,
                       const int precision);
    
  
      int writeBinaryCompressed (const std::string &file_name, 
                                 const sensor_msgs::PTXCloudData &cloud,
                                 const Eigen::Vector4d &origin, 
                                 const Eigen::Quaterniond &orientation,
                                 const Eigen::Affine3d& transformation,
                                 bool debug_image = false);
    
    private:

      void
      setLockingPermissions (const std::string &file_name, 
                             boost::interprocess::file_lock &lock);
    

      void
      resetLockingPermissions (const std::string &file_name,
                               boost::interprocess::file_lock &lock);

      /** \brief Set to true if msync() should be called before munmap(). Prevents data loss on NFS systems. */
      bool map_synchronization_;
  };

  /** warning callback expecting a FILE* client object */
  void 
  error_callback (const char *msg, void *client_data);
  
  /** warning callback expecting a FILE* client object */
  void 
  warning_callback (const char *msg, void *client_data);
  
  /** debug callback expecting no client object */
  void 
  info_callback (const char *msg, void *client_data);

}

#endif
