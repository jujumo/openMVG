// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2016 Romuald Perrot, Pierre Moulon

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/cameras/Camera_Pinhole_Radial.hpp"
#include "openMVG/cameras/Camera_Pinhole_Fisheye.hpp"
#include "openMVG/cameras/Camera_Intrinsics.hpp"
#include "openMVG/cameras/Camera_undistort_image.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_colorization.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <iomanip>
#include <cstdlib>
#include <sstream>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::features;


int main( int argc , char ** argv )
{

  CmdLine cmd;
  std::string sSfM_Data_Filename;
  std::string sOutDir = "";

  cmd.add( make_option( 'i', sSfM_Data_Filename, "sfmdata" ) );
  cmd.add( make_option( 'o', sOutDir, "outdir" ) );

  std::cout << "Note:  this program writes output in kapture file format.\n";

  try
  {
    if ( argc == 1 )
    {
      throw std::string( "Invalid command line parameter." );
    }
    cmd.process( argc, argv );
  }
  catch ( const std::string& s )
  {
    std::cerr << "Usage: " << argv[0] << '\n'
              << "[-i|--sfmdata] filename, the SfM_Data file to convert\n"
              << "[-o|--outdir] path where kapture files will be saved\n"
              << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if ( !stlplus::folder_exists( sOutDir ) )
  {
    stlplus::folder_create( sOutDir );
  }

  // Read the input SfM scene
  SfM_Data sfm_data;
  if ( !Load( sfm_data, sSfM_Data_Filename, ESfM_Data( ALL ) ) )
  {
    std::cerr << std::endl
              << "The input SfM_Data file \"" << sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  /*
  if ( ! exportToKapture( sfm_data , sOutDir ) )
  {
    std::cerr << "There was an error during export of the file" << std::endl;
    return EXIT_FAILURE;
  }*/

  return EXIT_SUCCESS;
}
