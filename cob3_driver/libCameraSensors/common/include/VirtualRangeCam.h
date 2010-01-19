/// @file VirtualRangeCam.h
/// Virtual range camera representation.
/// @author Jan Fischer 
/// @date 2009

#ifndef __VIRTUALRANGECAM_H__
#define __VIRTUALRANGECAM_H__

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <MathUtils.h>
#include <ThreeDUtils.h>
#include <OpenCVUtils.h>
#include <AbstractRangeImagingSensor.h>
#include <tinyxml/tinyxml.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"

#ifdef SWIG
%module Sensors3D

%{
	#include "SR31.h"
%}
#endif

namespace fs = boost::filesystem;
using namespace ipa_Utils;

namespace ipa_CameraSensors {

static const int SWISSRANGER_COLUMNS = 176;
static const int SWISSRANGER_ROWS = 144;

/// @ingroup VirtualCameraDriver
/// Interface class to virtual range camera like Swissranger 3000/4000.
/// The class offers an interface to a virtual range camera, that is equal to the interface of a real range camera.
/// However, pictures are read from a directory instead of the camera.
class VirtualRangeCam : public AbstractRangeImagingSensor
{
public:

	VirtualRangeCam();
	~VirtualRangeCam();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepOneChannel, char* RangeImage=NULL, char* IntensityImage=NULL, char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true);
	unsigned long AcquireImages(IplImage* rangeImage=NULL, IplImage* intensityImage=NULL, IplImage* cartesianImage=NULL, bool getLatestFrame = true, bool undistort = true);
	unsigned long AcquireImages2(IplImage** rangeImage=NULL, IplImage** intensityImage=NULL, IplImage** cartesianImage=NULL, bool getLatestFrame = true, bool undistort = true);

	unsigned long GetCalibratedUV(double x, double y, double z, double& u, double& v);

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}


private:
	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	unsigned long GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated);
	unsigned long GetCalibratedZNative(int u, int v, IplImage* coordinateImage, float& zCalibrated);
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y);
	unsigned long GetCalibratedXYNative(int u, int v, IplImage* coordinateImage, float& x, float& y);
	
	/// Load general range camera parameters .
	/// @param filename Configuration file-path and file-name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return code
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	bool m_CoeffsInitialized;

	/// Given a 32 bit swissranger depth value, the real depth value in meteres is given by:
	/// z(u,v)=a0(u,v)+a1(u,v)*d(u,v)+a2(u,v)*d(u,v)^2
	///       +a3(u,v)*d(u,v)^3+a4(u,v)*d(u,v)^4+a5(u,v)*d(u,v)^5
	///       +a6(u,v)*d(u,v)^6;
	DblMatrix m_CoeffsA0; ///< a0 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA1; ///< a1 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA2; ///< a2 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA3; ///< a3 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA4; ///< a4 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA5; ///< a5 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA6; ///< a6 z-calibration parameters. One matrix entry corresponds to one pixel

	std::string m_CameraDataDirectory; ///< Directory where the image data resides
	int m_CameraIndex; ///< Index of the specified camera. Important, when several cameras of the same type are present

	std::vector<std::string> m_IntensityImageFileNames ;
	std::vector<std::string> m_RangeImageFileNames ;
	std::vector<std::string> m_CoordinateImageFileNames ;

	unsigned int m_ImageCounter; ///< Holds the index of the image that is extracted during the next call of <code>AcquireImages</code>
	int m_ImageWidth;  ///< Image width
	int m_ImageHeight; ///< Image height
};


} // end namespace ipa_CameraSensors
#endif // __VIRTUALRANGECAM_H__


