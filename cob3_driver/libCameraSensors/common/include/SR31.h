/// @file SR31.h
/// Platform independent interface to SwissRanger camera SR-3000. Implementation depends on
/// libusbSR library.
/// 8@author Jan Fischer
/// @date 200

#ifndef __SR3000CAM_H__
#define __SR3000CAM_H__

// Windows
#ifndef __LINUX__
#include <windows.h>
// Windows with MinGW
#ifdef __MINGW__
typedef short __wchar_t;
#endif
#endif

// Linux
#ifdef __LINUX__
typedef unsigned long DWORD;
#endif

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <assert.h>

#include <libMesaSR.h>


#include <sstream>

#include <libMesaSR.h>

#include <MathUtils.h>
#include <ThreeDUtils.h>
#include <OpenCVUtils.h>
#include <AbstractRangeImagingSensor.h>
#include <tinyxml/tinyxml.h>

#ifdef SWIG
%module Sensors3D

%{
	#include "SR31.h"
%}
#endif


using namespace ipa_Utils;

namespace ipa_CameraSensors {

// former SR31Consts.h entries
#define SAFE_FREE(p)       { if(p) { delete (p); (p)=0; } }
#define SWISSRANGER_COLUMNS 176
#define SWISSRANGER_ROWS 144

/// Callback function to catch annoying debug message from swissranger camera
/// @param srCam Swissranger camera instance
/// @param msg the received message of type CM_XXX
/// @param param is a message specific parameter
/// @param is a message specific pointer
/// @return 
int LibMesaCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data);

/// @ingroup RangeCameraDriver
/// Interface class to SwissRanger camera SR-3000.
/// Platform independent interface to SwissRanger camera SR-3000. Implementation depends on
/// libusbSR library.
class SR31 : public AbstractRangeImagingSensor
{
public:

	SR31();
	~SR31();

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

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

private:
	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	unsigned long GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated);
	unsigned long GetCalibratedZSwissranger(int u, int v, int width, float& zCalibrated);
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y);
	unsigned long GetCalibratedXYSwissranger(int u, int v, int width, float& x, float& y);

	/// Load general SR31 parameters and previously determined calibration parameters.
	/// @param filename Swissranger parameter path and file name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return value 
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	/// Parses the data extracted by <code>LoadParameters</code> and calls the
	/// corresponding <code>SetProperty</code> functions.
	/// @return Return code
	unsigned long SetParameters();

	SRCAM m_SRCam; 			 ///< Handle to USB SR3000 camera
	int m_NumOfImages;		 ///< Number of images the siwssranger returns (i.e. an intensity and a range image)
	ImgEntry* m_DataBuffer;  ///< Image array

	// Stores for cartesian data, when native swissranger calibration is used
	float m_X[SWISSRANGER_COLUMNS * SWISSRANGER_ROWS];
	float m_Y[SWISSRANGER_COLUMNS * SWISSRANGER_ROWS];
	float m_Z[SWISSRANGER_COLUMNS * SWISSRANGER_ROWS];

	bool m_CoeffsInitialized; ///< True, when m_CoeffsAx have been initialized

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
};


} // End namespace ipa_CameraSensors
#endif // __SR3000CAM_H__


