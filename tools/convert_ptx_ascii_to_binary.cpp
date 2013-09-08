/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
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

#include <pcl/io/pcd_io.h>
#include <pcl/io/leica_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.ptx output.ptx -format format\n", argv[0]);
}

bool
loadCloud (const std::string &filename, sensor_msgs::PTXCloudData &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  leica::PTXReader reader;
  Eigen::Vector4f origin; Eigen::Quaternionf orientation; Eigen::Affine3f transform;
  int ptx_version;
  
  tt.tic ();
  if (reader.read (filename, cloud, origin, orientation, transform, ptx_version) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
saveCloud (const std::string &filename, const sensor_msgs::PTXCloudData &cloud, int format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  // pcl::PCDWriter writer;
  // writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);
  leica::PTXWriter writer;
  switch (format)
  {
    case 0 :
      writer.writeBinary (filename, cloud, Eigen::Vector4d::Zero (), Eigen::Quaterniond::Identity (), Eigen::Affine3d::Identity ());
      break;
    case 1 : 
      writer.writeLZFCompressed (filename, cloud, Eigen::Vector4d::Zero (), Eigen::Quaterniond::Identity (), Eigen::Affine3d::Identity ());
      break;
    case 2 : 
      writer.writeBinaryCompressed (filename, cloud, Eigen::Vector4d::Zero (), Eigen::Quaterniond::Identity (), Eigen::Affine3d::Identity (), false);
      break;
    default : print_error ("Unknown format %d!\n", format);
  }  
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert an ASCII PTX file to binary PTX format. For more information, use: %s -h\n", argv[0]);
  
  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .ptx files
  std::vector<int> ptx_file_indices = parse_file_extension_argument (argc, argv, ".ptx");
  std::cout << "ptx_file_indices.size () " << ptx_file_indices.size () << std::endl;
  for (std::size_t i = 0; i < ptx_file_indices.size (); ++i)
    std::cout << ptx_file_indices[i] << " ";
  std::cout << std::endl;
  
  if (ptx_file_indices.size () != 2)
  {
    print_error ("Need one input PTX file and one output PTX file.\n");
    return (-1);
  }
  std::cout << "PTX ascii file " << argv[ptx_file_indices[0]] << std::endl;
  std::cout << "PTX binary file " << argv[ptx_file_indices[1]] << std::endl;
  
  // Command line parsing
  int format = 0;
  parse_argument (argc, argv, "-format", format);
  switch (format)
  {
    case 0 : print_info ("PCD output format: binary\n"); break;
    case 1 : print_info ("PCD output format: lzf\n"); break;
    case 2 : print_info ("PCD output format: hybrid\n"); break;
    default : print_error ("Unknown format %d!\n", format); return (-1);
  }
  
  // Load the first file
  sensor_msgs::PTXCloudData cloud;
  if (!loadCloud (argv[ptx_file_indices[0]], cloud)) 
    return (-1);

  // Convert to PTX and save
  saveCloud (argv[ptx_file_indices[1]], cloud, format);

  return (0);
}

