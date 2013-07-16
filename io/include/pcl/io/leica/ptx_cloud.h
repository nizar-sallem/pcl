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

#ifndef LEICA_PTX_CLOUD_H
#define LEICA_PTX_CLOUD_H

#include <pcl/point_cloud.h>

namespace leica
{
  template <typename PointT>
  class PTXCloud : public pcl::PointCloud<PointT>
  {
    public:
    Eigen::Affine3f transformation_;
  };

  template <typename PointT> std::ostream&
  operator << (std::ostream& s, const leica::PTXCloud<PointT> &p)
  {
    s << "points[]: " << p.points.size () << std::endl;
    s << "width: " << p.width << std::endl;
    s << "height: " << p.height << std::endl;
    s << "is_dense: " << p.is_dense << std::endl;
    s << "sensor origin (xyz): [" << p.sensor_origin_.x ();
    s << ", " <<  p.sensor_origin_.y ();
    s << ", " << p.sensor_origin_.z () << "]"; 
    s <<" / orientation (xyzw): [" << p.sensor_orientation_.x ();
    s << ", " << p.sensor_orientation_.y ();
    s << ", " << p.sensor_orientation_.z ();
    s << ", " << p.sensor_orientation_.w () << "]" << std::endl;
    s << "cloud transformation: " << p.transformation.matrix () << std::endl;
    return (s);
  }
}

#endif
