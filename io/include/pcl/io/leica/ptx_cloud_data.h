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

#ifndef PCL_SENSOR_MSGS_MESSAGE_PTX_CLOUD_DATA_H
#define PCL_SENSOR_MSGS_MESSAGE_PTX_CLOUD_DATA_H

#include <pcl/PCLPointCloud2.h>

namespace pcl
{
  struct PTXCloudData : public PCLPointCloud2
  {
    PTXCloudData ()
      : PCLPointCloud2 (), image_offset (-1), image_encoding (), image_step (0) { }

    std::size_t   image_offset;
    std::string   image_encoding;
    pcl::uint32_t image_step;

    typedef boost::shared_ptr<PTXCloudData> Ptr;
    typedef boost::shared_ptr<PTXCloudData  const> ConstPtr;
  };

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::PTXCloudData &v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "height: ";
    s << "  " << v.height << std::endl;
    s << "width: ";
    s << "  " << v.width << std::endl;
    s << "is_bigendian: ";
    s << "  " << v.is_bigendian << std::endl;
    s << "point_step: ";
    s << "  " << v.point_step << std::endl;
    s << "row_step: ";
    s << "  " << v.row_step << std::endl;

    if (v.image_offset > -1)
    {
      s << "image_offset: ";
      s << "  " << v.image_offset << std::endl;
      s << "image_encoding: ";
      s << "  " << v.image_encoding << std::endl;
      s << "image_step: ";
      s << "  " << v.image_step << std::endl;
    }

    s << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size (); ++i)
    {
      s << "  data[" << i << "]: ";
      s << "  " << v.data[i] << std::endl;
    }
    return (s);
  }
}

#endif
