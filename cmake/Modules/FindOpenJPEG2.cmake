###############################################################################
# Find OpenJPEG2
#
# This sets the following variables:
# OpenJPEG2_FOUND - True if OPENNI was found.
# OpenJPEG2_INCLUDE_DIR - Directories containing the OpenJPEG2 include files.
# OpenJPEG2_LIBRARIES - Libraries needed to use OpenJPEG2.
# OpenJPEG2_LIBRARY - OpenJPEG2 library
# OpenJPEG2_LIBRARY_DEBUG - OpenJPEG2 library with debug symbols
# OpenJPEG2_IMAGE_TO_J2K - OpenJPEG2 compression tool
# OpenJPEG2_J2K_TO_IMAGE - OpenJPEG2 decompression tool

find_path(OpenJPEG2_INCLUDE_DIR openjpeg.h
          HINTS "${OpenJPEG2_ROOT}" "$ENV{OpenJPEG2_ROOT}"
          PATH_SUFFIXES include)

find_library(OpenJPEG2_LIBRARY 
             NAMES openjpeg
             HINTS "${OpenJPEG2_ROOT}" "$ENV{OpenJPEG2_ROOT}"
             PATH_SUFFIXES Lib Lib64)

find_library(OpenJPEG2_LIBRARY_DEBUG 
             NAMES openjpeg
             HINTS /usr/lib/debug/usr/lib "${OpenJPEG2_ROOT}" "$ENV{OpenJPEG2_ROOT}"
             PATH_SUFFIXES debug)

find_program(OpenJPEG2_IMAGE_TO_J2K
             NAMES image_to_j2k
             HINTS "${OpenJPEG2_ROOT}" "$ENV{OpenJPEG2_ROOT}"
             PATH_SUFFIXES bin)

find_program(OpenJPEG2_J2K_TO_IMAGE
             NAMES j2k_to_image
             HINTS "${OpenJPEG2_ROOT}" "$ENV{OpenJPEG2_ROOT}"
             PATH_SUFFIXES bin)

find_program(OpenJPEG2_INDEX_CREATE
             NAMES index_create
             HINTS "${OpenJPEG2_ROOT}" "$ENV{OpenJPEG2_ROOT}"
             PATH_SUFFIXES bin)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenJPEG2 DEFAULT_MSG OpenJPEG2_LIBRARY OpenJPEG2_INCLUDE_DIR)
    
mark_as_advanced(OpenJPEG2_LIBRARY OpenJPEG2_INCLUDE_DIR)

if(OPENJPEG2_FOUND)
  # Add the include directories
  set(OpenJPEG2_INCLUDE_DIRS ${OpenJPEG2_INCLUDE_DIR})
  set(OpenJPEG2_LIBRARIES ${OpenJPEG2_LIBRARY} ${OpenJPEG2_LIBRARY_DEBUG})
  message(STATUS "OpenJPEG2 found (include: ${OpenJPEG2_INCLUDE_DIRS}, lib: ${OpenJPEG2_LIBRARY})")
endif(OPENJPEG2_FOUND)

set (OpenJPEG2_FOUND ${OPENJPEG2_FOUND})
