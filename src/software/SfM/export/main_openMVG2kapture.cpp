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

#include <algorithm>
#include <iomanip>
#include <cstdlib>
#include <sstream>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;
using namespace openMVG::features;

static const std::string SEP = ", ";
static const std::string KAPTURE_SENSOR_DIR = "sensors";
static const std::string KAPTURE_RECONSTRUCTION_DIR = "reconstruction";

bool CreateLineCameraFile(  const IndexT camera_id,
                            std::shared_ptr<openMVG::cameras::IntrinsicBase> intrinsic,
                            std::string & camera_linie)
{
  // kapture sensors.txt is formated like:
  // sensor_device_id, name, sensor_type, [sensor_params]+
  std::stringstream came_line_ss;
  came_line_ss << camera_id << SEP << "cam_" << camera_id << SEP;
  EINTRINSIC current_type = intrinsic->getType();
  switch(current_type)
  {
    case PINHOLE_CAMERA:
      //OpenMVG's PINHOLE_CAMERA corresponds to kapture's SIMPLE_PINHOLE
      //Parameters: w, h, f, cx, cy
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic> pinhole_intrinsic(
          dynamic_cast<openMVG::cameras::Pinhole_Intrinsic * >(intrinsic->clone()));
        came_line_ss << "SIMPLE_PINHOLE" << SEP <<
                        pinhole_intrinsic->w() << SEP <<
                        pinhole_intrinsic->h() << SEP <<
                        pinhole_intrinsic->focal() << SEP <<
                        pinhole_intrinsic->principal_point().x() << SEP <<
                        pinhole_intrinsic->principal_point().y() << std::endl;
      }
      break;
    case PINHOLE_CAMERA_RADIAL1:
      //OpenMVG's PINHOLE_CAMERA_RADIAL1 corresponds to kapture's SIMPLE_RADIAL
      //Parameters: w, h, f, cx, cy, k
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Radial_K1> pinhole_intrinsic_radial(
            dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Radial_K1 * >(intrinsic->clone()));
        came_line_ss << "SIMPLE_RADIAL" << SEP <<
                        pinhole_intrinsic_radial->w() << SEP <<
                        pinhole_intrinsic_radial->h() << SEP <<
                        pinhole_intrinsic_radial->focal() << SEP <<
                        pinhole_intrinsic_radial->principal_point().x() << SEP <<
                        pinhole_intrinsic_radial->principal_point().y() << SEP <<
                        pinhole_intrinsic_radial->getParams().at(3) << std::endl;   //k1
      }
      break;
    case PINHOLE_CAMERA_RADIAL3:
      //OpenMVG's PINHOLE_CAMERA_RADIAL3 corresponds to kapture's FULL_OPENCV
      //Parameters: w, h, fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Radial_K3> pinhole_intrinsic_radial(
            dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 * >(intrinsic->clone()));
        came_line_ss << "FULL_OPENCV" << SEP <<
                        pinhole_intrinsic_radial->w() << SEP <<
                        pinhole_intrinsic_radial->h() << SEP <<
                        pinhole_intrinsic_radial->focal() << SEP <<
                        pinhole_intrinsic_radial->focal() << SEP <<
                        pinhole_intrinsic_radial->principal_point().x() << SEP <<
                        pinhole_intrinsic_radial->principal_point().y() << SEP <<
                        pinhole_intrinsic_radial->getParams().at(3) << SEP << //k1
                        pinhole_intrinsic_radial->getParams().at(4) << SEP << //k2
                        0.0 << SEP << //p1
                        0.0 << SEP << //p2
                        pinhole_intrinsic_radial->getParams().at(5) << SEP << //k3
                        0.0 << SEP << // k4
                        0.0 << SEP << // k5
                        0.0 << std::endl;  // k6
      }
      break;
    case PINHOLE_CAMERA_FISHEYE:
      //OpenMVG's PINHOLE_CAMERA_FISHEYE corresponds to kapture's OPENCV_FISHEYE
      //Parameters: w, h, fx, fy, cx, cy, k1, k2, k3, k4
      {
        std::shared_ptr<openMVG::cameras::Pinhole_Intrinsic_Fisheye> pinhole_intrinsic_fisheye(
            dynamic_cast<openMVG::cameras::Pinhole_Intrinsic_Fisheye * >(intrinsic->clone()));
        came_line_ss << "OPENCV_FISHEYE" << SEP <<
                        pinhole_intrinsic_fisheye->w() << SEP <<
                        pinhole_intrinsic_fisheye->h() << SEP <<
                        pinhole_intrinsic_fisheye->focal() << SEP <<
                        pinhole_intrinsic_fisheye->focal() << SEP <<
                        pinhole_intrinsic_fisheye->principal_point().x() << SEP <<
                        pinhole_intrinsic_fisheye->principal_point().y() << SEP <<
                        pinhole_intrinsic_fisheye->getParams().at(3) << SEP << //k1
                        pinhole_intrinsic_fisheye->getParams().at(4) << SEP << //k2
                        pinhole_intrinsic_fisheye->getParams().at(5) << SEP << //k3
                        pinhole_intrinsic_fisheye->getParams().at(6) << std::endl; //k4
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
    std::cerr << "Cannot write file " << sSensorsFilename << std::endl;
    return false;
  }

  sensors_file << "# kapture format: 1.0\n";
  sensors_file << "# sensor_id, name, sensor_type, [sensor_params]+\n";

  std::vector<std::string> camera_lines;
  C_Progress_display my_progress_bar( sfm_data.GetIntrinsics().size(), std::cout, "\n- CREATE SENSORS FILE -\n" );
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



bool CreateRecordsCameraFile( const SfM_Data & sfm_data,
                              const std::string & sRecordsCameraFilename)
{
   /* sensors/records_camera.txt
      # Each line of the file is composed of :
      timestamp, device_id, image_path
  */
  std::ofstream records_camera_file( sRecordsCameraFilename );
  if ( ! records_camera_file )
  {
    std::cerr << "Cannot write file " << sRecordsCameraFilename << std::endl;
    return false;
  }

  records_camera_file << "# kapture format: 1.0\n";
  records_camera_file << "# timestamp, device_id, image_path\n";

  {
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(), std::cout, "\n- CREATE RECORDS CAMERA FILE -\n" );

    for (Views::const_iterator iter = sfm_data.GetViews().begin();
         iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      const std::string & image_name = view->s_Img_path;
      const IndexT timestamp = view->id_view; // since there is no timestamp in openMVG, consider each view at a different timestamp.
      const IndexT camera_id = view->id_intrinsic;
      records_camera_file << timestamp << SEP
                          << camera_id  << SEP
                          << sfm_data.s_root_path + image_name << std::endl;
    }
  }

  return true;
}


bool CreateTrajectoriesFile( const SfM_Data & sfm_data,
                             const std::string & sTrajectoriesFilename)
{
   /* sensors/trajectories.txt
      # Each line of the file is composed of :
      timestamp, device_id, qw, qx, qy, qz, tx, ty, tz
  */
  std::ofstream trajectories_file( sTrajectoriesFilename );
  if ( ! trajectories_file )
  {
    std::cerr << "Cannot write file " << sTrajectoriesFilename << std::endl;
    return false;
  }

  trajectories_file << "# kapture format: 1.0" << std::endl;
  trajectories_file << "# timestamp, device_id, qw, qx, qy, qz, tx, ty, tz" << std::endl;

  {
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(), std::cout, "\n- CREATE TRAJECTORIES FILE -\n" );

    for (Views::const_iterator iter = sfm_data.GetViews().begin();
         iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
    {
      const View * view = iter->second.get();
      if ( !sfm_data.IsPoseAndIntrinsicDefined( view ) )
      {
        continue; // skip if no extrinsic
      }
      const IndexT timestamp = view->id_view; // since there is no timestamp in openMVG, consider each view at a different timestamp.
      const IndexT camera_id = view->id_intrinsic;

      const Pose3 pose = sfm_data.GetPoseOrDie( view );
      const Mat3 rotation = pose.rotation();
      const Vec3 translation = pose.translation();
      const double tx = translation[0];
      const double ty = translation[1];
      const double tz = translation[2];
      Eigen::Quaterniond q( rotation );
      const double qx = q.x();
      const double qy = q.y();
      const double qz = q.z();
      const double qw = q.w();

      trajectories_file << timestamp << SEP
                        << camera_id  << SEP
                        << qw << SEP << qx << SEP << qy << SEP << qz << SEP
                        << tx << SEP << ty << SEP << tz << std::endl;
    }
  }

  return true;
}

static bool key_compare(const std::pair<int, Landmark>& a, const std::pair<int, Landmark>& b)
{
    return (a.first < b.first);
}

bool CreatePoints3dFile( const SfM_Data & sfm_data,
                         const std::string & sPoints3dFilename)
{
    /* reconstruction/points3d.txt
       # Each line of the file is composed of :
        X, Y, Z, [R, G, B]
   */
    const Landmarks landmarks = sfm_data.GetLandmarks();
    if (landmarks.size() == 0) {
        std::cout << "\n- NO POINT3D FILE  -\n";
        return true;
    }

    std::ofstream points3d_file( sPoints3dFilename );
    if ( ! points3d_file )
    {
      std::cerr << "Cannot write file " << sPoints3dFilename << std::endl;
      return false;
    }

    points3d_file << "# kapture format: 1.0" << std::endl;
    points3d_file << "# X, Y, Z, [R, G, B]" << std::endl;

    std::vector<Vec3> vec_3dPoints, vec_tracksColor;
    if (!ColorizeTracks(sfm_data, vec_3dPoints, vec_tracksColor)) {
      return false;
    }

    C_Progress_display my_progress_bar( vec_3dPoints.size(), std::cout, "\n- CREATE POINT3D FILE  -\n" );
    for (size_t point3d_idx = 0 ; point3d_idx < vec_3dPoints.size() ; ++point3d_idx, ++my_progress_bar) {
        const Vec3 coords = vec_3dPoints[point3d_idx];
        const Vec3 color = vec_tracksColor[point3d_idx];
        points3d_file << coords.x() << SEP
                      << coords.y() << SEP
                      << coords.z() << SEP
                      << color.x() << SEP
                      << color.y() << SEP
                      << color.z()
                      << std::endl;
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
  const std::string sSensorsDirpath = sOutDirectory + "/" + KAPTURE_SENSOR_DIR + "/";
  // sensors/sensors (cameras instrinsics)
  const std::string sSensorsFilename = stlplus::create_filespec( sSensorsDirpath , "sensors.txt" );
  if (!CreateSensorsFile(sfm_data, sSensorsFilename))
  {
    return false;
  }

  // sensors/records_camera (images)
  const std::string sRecordsCameraFilename = stlplus::create_filespec( sSensorsDirpath, "records_camera.txt" );
  if (!CreateRecordsCameraFile(sfm_data, sRecordsCameraFilename))
  {
    return false;
  }

  // sensors/trajectories (extrinsics)
  const std::string sTrajectoriesFilename = stlplus::create_filespec( sSensorsDirpath , "trajectories.txt" );
  if (!CreateTrajectoriesFile(sfm_data, sTrajectoriesFilename))
  {
    return false;
  }

  const std::string sReconstructionDirpath = sOutDirectory + "/" + KAPTURE_RECONSTRUCTION_DIR + "/";
  // reconstruction/points3d
  const std::string sPoints3dFilename = stlplus::create_filespec( sReconstructionDirpath , "points3d.txt" );
  if (!CreatePoints3dFile(sfm_data, sPoints3dFilename))
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
  bool bOk = true;
  const std::array< const std::string, 3 > output_dir_list = {
      sOutDirectory, sOutDirectory + "/sensors", sOutDirectory + "/reconstruction"
  };

  for (auto output_dirpath : output_dir_list) {
      if ( !stlplus::is_folder( output_dirpath ) )
      {
        std::cout << "\033[1;31mCreating kapture directory in :  " << output_dirpath << "\033[0m\n";
        stlplus::folder_create( output_dirpath );
        bOk = stlplus::is_folder( output_dirpath );
      }
      else
      {
        bOk = bOk && true;
      }
  }

  if ( !bOk )
  {
    std::cerr << "Cannot access one of the desired output directories" << std::endl;
    return false;
  }

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
