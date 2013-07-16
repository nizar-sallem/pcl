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

#if defined __GNUC__
#  pragma GCC system_header
#endif

#include <Eigen/Core>
#include <ostream>

namespace leica
{
#define LEICA_ADD_POINT4D \
  union EIGEN_ALIGN16 {   \
    float data[4];        \
    struct {              \
      float x;            \
      float y;            \
      float z;            \
      float i;            \
    };                    \
  };                      \
  
  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); } \
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); } \
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); } \
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

#define LEICA_ADD_RGB                           \
  union                                         \
  {                                             \
    struct                                      \
    {                                           \
      uint8_t r;                                \
      uint8_t g;                                \
      uint8_t b;                                \
      uint8_t a;                                \
    };                                          \
    uint32_t rgba;                              \
  };
  
  struct _PointXYZI
  {
    LEICA_ADD_POINT4D;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
  struct EIGEN_ALIGN16 PointXYZI : public _PointXYZI
  {
    inline PointXYZI (const _PointXYZI &p)
    {
      x = p.x; y = p.y; z = p.z; i = p.i;
    }

    inline PointXYZI ()
    {
      x = y = z = i = 0.f;
    }

    inline PointXYZI (float _x, float _y, float _z, float _i)
    {
      x = _x; y = _y; z = _z; i = _i;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZI& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }
  
  struct EIGEN_ALIGN16 _PointXYZIRGBA
  {
    LEICA_ADD_POINT4D; 
    LEICA_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZIRGBA& p);

  struct EIGEN_ALIGN16 PointXYZIRGBA : public _PointXYZIRGBA
  {
    inline PointXYZIRGBA (const _PointXYZIRGBA &p)
    {
      x = p.x; y = p.y; z = p.z; i = p.i;
      rgba = p.rgba;
    }

    inline PointXYZIRGBA ()
    {
      x = y = z = i = 0.f;
      r = g = b = a = 0;
    }

    inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); }
    inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); }
    inline Eigen::Vector4i getRGBVector4i () { return (Eigen::Vector4i (r, g, b, a)); }
    inline const Eigen::Vector4i getRGBVector4i () const { return (Eigen::Vector4i (r, g, b, a)); }
    friend std::ostream& operator << (std::ostream& os, const PointXYZIRGBA& p);
  };
}

#endif
