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
#include <pcl/common/angles.h>

namespace leica
{
  using half_float::half;

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

#define LEICA_ADD_POINTSP     \
  union EIGEN_ALIGN16 {       \
    float data[4];            \
    struct {                  \
      float r;              \
      float theta;            \
      float phi;              \
      float i;                \
    };                        \
  };                          \
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

  struct _SphericalPoint
  {
    LEICA_ADD_POINTSP;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  
  struct EIGEN_ALIGN16 SphericalPoint : public _SphericalPoint
  {
    inline SphericalPoint (const _SphericalPoint &p)
    {
      r = p.r; theta = p.theta; phi = p.phi; i = p.i;
    }

    inline SphericalPoint ()
    {
      r = theta = phi = i = 0.f;
    }

    inline SphericalPoint (float _r, float _theta, float _phi, float _i)
    {
      r = _r; theta = _theta; phi = _phi; i = _i;
    }

    inline SphericalPoint (const PointXYZI &p)
    {
      r = sqrt (p.z*p.z + p.y*p.y + p.x*p.x);
      theta = pcl::rad2deg (acos (p.z/r));
      phi = pcl::rad2deg (atan2 (p.y, p.x));
      i = p.i;
    }
    
    friend std::ostream& operator << (std::ostream& os, const SphericalPoint& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct IndexedSpherical
  {
    IndexedSpherical () 
      : radius (0), indexed_theta (0), indexed_phi (0), i (0)
    {}
    
    IndexedSpherical (const IndexedSpherical& p)
      : radius (p.radius), indexed_theta (p.indexed_theta), indexed_phi (p.indexed_phi), i (p.i)
    {}

    IndexedSpherical (float x, float y, float z, float intensity)
    {
      radius = sqrt (z*z + y*y + x*x);
      if (radius)
      {
        indexed_theta = static_cast<uint16_t> (acos (z/radius) * 10000);
        indexed_phi = static_cast<int16_t> (atan2 (y, x) * 10000);
      }
      else
      {
        indexed_theta = 0;
        indexed_phi = 0;
      }
      i = half_float::half_cast<half,std::round_to_nearest> (intensity);
    }

    float radius;
    uint16_t indexed_theta;
    int16_t indexed_phi;
    half i;
  };

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

POINT_CLOUD_REGISTER_POINT_STRUCT (leica::_SphericalPoint,
                                   (float, r, r)
                                   (float, theta, theta)
                                   (float, phi, phi)
                                   (float, i, i))

POINT_CLOUD_REGISTER_POINT_WRAPPER(leica::SphericalPoint, leica::_SphericalPoint)

POINT_CLOUD_REGISTER_POINT_STRUCT (leica::IndexedSpherical,
                                   (float, radius, radius)
                                   (uint16_t, indexed_theta, indexed_theta)
                                   (int16_t, indexed_phi, indexed_phi)
                                   (::half_float::half, i, i))

#endif
