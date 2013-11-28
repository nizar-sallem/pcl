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
  print_error ("Syntax is: %s input.pcd output.pcd\n", argv[0]);
}

bool
loadCloud (const std::string &filename, pcl::PTXCloudData &cloud)
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
saveCloud (const std::string &filename, const pcl::PTXCloudData &cloud, bool format)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  
  pcl::PCDWriter writer;
  writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);
  
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Convert a PTX file to PCD format. For more information, use: %s -h\n", argv[0]);
  std::cout << "argc " << argc << std::endl;
  
  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd and .ptx files
  std::vector<int> ptx_file_indices = parse_file_extension_argument (argc, argv, ".ptx");
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1 || ptx_file_indices.size () != 1)
  {
    print_error ("Need one input PTX file and one output PCD file.\n");
    return (-1);
  }
  std::cout << "PTX file " << argv[ptx_file_indices[0]] << std::endl;
  std::cout << "PCD file " << argv[pcd_file_indices[0]] << std::endl;
  
  // Command line parsing
  int format = 0;
  parse_argument (argc, argv, "-format", format);
  print_info ("PCD output format: "); 
  print_value ("%s\n", (format ? "binary" : "ascii"));

  // Load the first file
  pcl::PTXCloudData cloud;
  if (!loadCloud (argv[ptx_file_indices[0]], cloud)) 
    return (-1);

  // Convert to PTX and save
  saveCloud (argv[pcd_file_indices[0]], cloud, format);

  return (0);
}

