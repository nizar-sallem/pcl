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

#ifndef LEICA_POINT_TYPES_HPP
#define LEICA_POINT_TYPES_HPP

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
  };                                                                    \
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
      uint8_t b;                                \
      uint8_t g;                                \
      uint8_t r;                                \
    };                                          \
    uint8_t rgb[3];                             \
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
  };
  
  struct EIGEN_ALIGN16 _PointXYZIRGB
  {
    LEICA_ADD_POINT4D; 
    LEICA_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZIRGB& p);

  struct EIGEN_ALIGN16 PointXYZIRGB : public _PointXYZIRGB
  {
    inline PointXYZIRGB (const _PointXYZIRGB &p)
    {
      x = p.x; y = p.y; z = p.z; i = p.i;
      r = p.r; g = p.g; b = p.b;
    }

    inline PointXYZIRGB ()
    {
      x = y = z = i = 0.f;
      r = g = b = 0;
    }

    inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); }
    inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); }
    friend std::ostream& operator << (std::ostream& os, const PointXYZIRGB& p);
  };

  struct _RGB
  {
    LEICA_ADD_RGB;
  };

  PCL_EXPORTS std::ostream& operator << (std::ostream& os, const RGB& p);

  struct RGB: public _RGB
  {
    inline RGB (const _RGB &p)
    {
      r = p.r;
      g = p.g;
      b = p.b;
    }

    inline RGB ()
    {
      r = g = b = 0;
    }
  
    friend std::ostream& operator << (std::ostream& os, const RGB& p);
  };

  // class float24
  // {
  //   float24 (const float24 &f) { memcpy (data, &f.data, 3*sizeof(char)); }
  //   float24 (float f)
  //   {
  //     int cnt = 0;
  //     while (f != floor (f)) 
  //     {
  //       f *= 10.0; 
  //       cnt++;
  //     }
      
  //     int val = (cnt << 16) | ((int) (value));
  //     memcpy (data, val, 3*sizeof(char));
  //   }
    
  //   operator float()
  //   {
  //     int cnt = data >> 16;
  //     float result = data & 0xffff;
      
  //     while (cnt > 0) 
  //     {
  //       result /= 10.0; 
  //       cnt--;
  //     }
      
  //     return result;
  //   }
    
  //   private:
  //   char data[3];
  // };
} // namespace leica


POINT_CLOUD_REGISTER_POINT_STRUCT (leica::_PointXYZI,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, i, i))

POINT_CLOUD_REGISTER_POINT_WRAPPER(leica::PointXYZI, leica::_PointXYZI)

POINT_CLOUD_REGISTER_POINT_STRUCT (leica::_PointXYZIRGB,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, i, i)
                                   (uint8_t, r, r)
                                   (uint8_t, g, g)
                                   (uint8_t, b, b))

POINT_CLOUD_REGISTER_POINT_WRAPPER(leica::PointXYZIRGB, leica::_PointXYZIRGB)

POINT_CLOUD_REGISTER_POINT_STRUCT (leica::_RGB,
                                   (uint8_t, r, r)
                                   (uint8_t, g, g)
                                   (uint8_t, b, b))

POINT_CLOUD_REGISTER_POINT_WRAPPER(leica::RGB, leica::_RGB)

#endif
