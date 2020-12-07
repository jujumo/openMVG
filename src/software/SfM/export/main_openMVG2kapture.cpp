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

bool CreateLineCameraFile(  const IndexT camera_id,
                            std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic,
                            std::string & camera_linie)
{
  std::stringstream came_line_ss;
  EINTRINSIC current_type = intrinsic->getType();
  switch(current_type)
  {
    case PINHOLE_CAMERA:
      //OpenMVG's PINHOLE_CAMERA corresponds to kapture's SIMPLE_PINHOLE
      //Parameters: f, cx, cy
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic> pinhole_intrinsic(
          dynamic_cast<openMVG::cameras::Pinhole_Intrinsic * >(intrinsic->clone()));

        came_line_ss << camera_id << " " <<
          "SIMPLE_PINHOLE" << " " <<
          pinhole_intrinsic->w() << " " <<
          pinhole_intrinsic->h() << " " <<
          pinhole_intrinsic->focal() << " " <<
          pinhole_intrinsic->principal_point().x() << " " <<
          pinhole_intrinsic->principal_point().y() << "\n";
      }
      break;
    case PINHOLE_CAMERA_RADIAL1:
      //OpenMVG's PINHOLE_CAMERA_RADIAL1 corresponds to kapture's SIMPLE_RADIAL
      //Parameters: f, cx, cy, k1
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Radial_K1> pinhole_intrinsic_radial(
            dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Radial_K1 * >(intrinsic->clone()));

        came_line_ss << camera_id << " " <<
          "SIMPLE_RADIAL" << " " <<
          pinhole_intrinsic_radial->w() << " " <<
          pinhole_intrinsic_radial->h() << " " <<
          pinhole_intrinsic_radial->focal() << " " <<
          pinhole_intrinsic_radial->principal_point().x() << " " <<
          pinhole_intrinsic_radial->principal_point().y() << " " <<
          pinhole_intrinsic_radial->getParams().at(3) << "\n";   //k1
      }
      break;
    case PINHOLE_CAMERA_RADIAL3:
      //OpenMVG's PINHOLE_CAMERA_RADIAL3 corresponds to kapture's FULL_OPENCV
      //Parameters: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Radial_K3> pinhole_intrinsic_radial(
            dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 * >(intrinsic->clone()));

        came_line_ss << camera_id << " " <<
          "FULL_OPENCV" << " " <<
          pinhole_intrinsic_radial->w() << " " <<
          pinhole_intrinsic_radial->h() << " " <<
          pinhole_intrinsic_radial->focal() << " " <<
          pinhole_intrinsic_radial->focal() << " " <<
          pinhole_intrinsic_radial->principal_point().x() << " " <<
          pinhole_intrinsic_radial->principal_point().y() << " " <<
          pinhole_intrinsic_radial->getParams().at(3) << " " << //k1
          pinhole_intrinsic_radial->getParams().at(4) << " " << //k2
          0.0 << " " << //p1
          0.0 << " " << //p2
          pinhole_intrinsic_radial->getParams().at(5) << " " << //k3
          0.0 << " " << // k4
          0.0 << " " << // k5
          0.0 << "\n";  // k6
      }
      break;
    case PINHOLE_CAMERA_BROWN:
      std::cout << "PINHOLE_CAMERA_BROWN is not supported. Aborting ..." << std::endl;
      return false;
      break;
    case PINHOLE_CAMERA_FISHEYE:
      //OpenMVG's PINHOLE_CAMERA_FISHEYE corresponds to kapture's OPENCV_FISHEYE
      //Parameters: fx, fy, cx, cy, k1, k2, k3, k4
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Fisheye> pinhole_intrinsic_fisheye(
            dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Fisheye * >(intrinsic->clone()));

        came_line_ss << camera_id << " " <<
          "OPENCV_FISHEYE" << " " <<
          pinhole_intrinsic_fisheye->w() << " " <<
          pinhole_intrinsic_fisheye->h() << " " <<
          pinhole_intrinsic_fisheye->focal() << " " <<
          pinhole_intrinsic_fisheye->focal() << " " <<
          pinhole_intrinsic_fisheye->principal_point().x() << " " <<
          pinhole_intrinsic_fisheye->principal_point().y() << " " <<
          pinhole_intrinsic_fisheye->getParams().at(3) << " " << //k1
          pinhole_intrinsic_fisheye->getParams().at(4) << " " << //k2
          pinhole_intrinsic_fisheye->getParams().at(5) << " " << //k3
          pinhole_intrinsic_fisheye->getParams().at(6) << " " << "\n"; //k4
      }
      break;
    default: std::cout << "Camera Type " << current_type << " is not supported. Aborting ..." << std::endl;
    return false;
  }
  camera_linie = came_line_ss.str();
  return true;
}


bool CreateSensorsFile( const SfM_Data & sfm_data,
                        const std::string & sSensorsFilename)
{
   /* sensors/sensors.txt
      # Each line of the file is composed of :
      sensor_device_id, name, sensor_type, [sensor_params]+
  */
  std::ofstream sensors_file( sSensorsFilename );
  if ( ! sensors_file )
  {
    std::cerr << "Cannot write file" << sSensorsFilename << std::endl;
    return false;
  }

  sensors_file << "# kapture format: 1.0\n";
  sensors_file << "# sensor_id, name, sensor_type, [sensor_params]+\n";

  std::vector<std::string> camera_lines;
  C_Progress_display my_progress_bar( sfm_data.GetIntrinsics().size(), std::cout, "\n- CREATE CAMERA FILE -\n" );
  for (Intrinsics::const_iterator iter = sfm_data.GetIntrinsics().begin();
    iter != sfm_data.GetIntrinsics().end(); ++iter, ++my_progress_bar)
  {
    const IndexT camera_id = iter->first;
    std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic = iter->second;
    std::string camera_line;
    if (CreateLineCameraFile(camera_id, intrinsic, camera_line))
    {
      camera_lines.push_back(camera_line);
    }
    else
    {
      return false;
    }
  }
  for (auto const& camera_line: camera_lines)
  {
    sensors_file << camera_line << "\n";
  }
  return true;
}


/**
* @brief Convert OpenMVG reconstruction result to kapture's format.
* @param sfm_data Structure from Motion file
* @param sOutDirectory Output directory
*/
bool CreateKaptureFolder( const SfM_Data & sfm_data,
                         const std::string & sOutDirectory)
{
  const std::string sSensorsFilename = stlplus::create_filespec( sOutDirectory , "sensors/sensors.txt" );

  if (!CreateSensorsFile(sfm_data, sSensorsFilename))
  {
    return false;
  }
  return true;
}

/**
* @brief Main function used to export a kapture folder
* @param sfm_data Structure from Motion file to export
* @param sOutDirectory Output kapture directory
*/
bool exportToKapture( const SfM_Data & sfm_data , const std::string & sOutDirectory  )
{
  // Create output directory
  bool bOk = false;
  if ( !stlplus::is_folder( sOutDirectory ) )
  {
    std::cout << "\033[1;31mCreating directory:  " << sOutDirectory << "\033[0m\n";
    stlplus::folder_create( sOutDirectory );
    bOk = stlplus::is_folder( sOutDirectory );
  }
  else
  {
    bOk = true;
  }

  if ( !bOk )
  {
    std::cerr << "Cannot access one of the desired output directories" << std::endl;
    return false;
  }


  //const std::string sCamerasFilename = stlplus::create_filespec( sOutDirectory , "cameras.txt" );
  //const std::string sImagesFilename = stlplus::create_filespec( sOutDirectory , "images.txt" );
  //const std::string sPoints3DFilename = stlplus::create_filespec( sOutDirectory , "points3D.txt" );
  if ( ! CreateKaptureFolder( sfm_data , sOutDirectory) )
  {
    std::cerr << "There was an error exporting project" << std::endl;
    return false;
  }
  return true;

}


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

  if ( ! exportToKapture( sfm_data , sOutDir ) )
  {
    std::cerr << "There was an error during export of the file" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
