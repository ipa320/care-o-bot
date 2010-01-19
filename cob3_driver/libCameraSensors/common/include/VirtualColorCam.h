/// @file VirtualColorCam.h
/// Virtual color camera representation.
/// @author Jan Fischer
/// @date 2009.

#ifndef __VIRTUALCOLORCAM_H__
#define __VIRTUALCOLORCAM_H__

#include "AbstractColorCamera.h"

#include <highgui.h>
#include <cv.h>
#include <cxcore.h>

#include <vector>
#include <iostream>
#include <cstdlib>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"

namespace fs = boost::filesystem;
using namespace std;

#ifdef SWIG
%module Sensors
%include "AbstractColorCamera.h"

%{
	#include "VirtualColorCam.h"
%}
#endif

namespace ipa_CameraSensors {
/// @ingroup VirtualCameraDriver
/// The class offers an interface to a virtual color camera, that is equivalent
/// to the interface of a real color camera.
/// However, pictures are read from a directory instead of the camera.
class VirtualColorCam : public AbstractColorCamera
{
	private:
		int m_ImageWidth;
		int m_ImageHeight;

		std::string m_CameraDataDirectory; ///< Directory where the image data resides
		int m_CameraIndex; ///< Index of the specified camera. Important, when several cameras of the same type are present

		std::vector<std::string> m_ColorImageFileNames;

		unsigned int m_ImageCounter; ///< Holds the index of the image that is extracted during the next call of <code>AcquireImages</code>

		/// Parses the XML configuration file, that holds the camera settings
		/// @param filename The file name and path of the configuration file
		/// @param cameraIndex The index of the camera within the configuration file
		///		   i.e. AVT_PIKE_CAM_0 or AVT_PIKE_CAM_1
		/// @return Return value
		unsigned long LoadParameters(const char* filename, int cameraIndex);
		
		unsigned long SetParameters(){return RET_OK;};

	public:

		VirtualColorCam ();
		~VirtualColorCam ();

		//*******************************************************************************
		// AbstractColorCamera interface implementation
		//*******************************************************************************

		unsigned long Init(std::string directory, int cameraIndex = 0);

		unsigned long Open();
		unsigned long Close();

		unsigned long GetColorImage(char* colorImageData, bool getLatestFrame);
		unsigned long GetColorImage(IplImage * Img, bool getLatestFrame);
		unsigned long GetColorImage2(IplImage ** Img, bool getLatestFrame);
		unsigned long SaveParameters(const char* filename);		//speichert die Parameter in das File
		unsigned long SetProperty(t_cameraProperty* cameraProperty);
		unsigned long SetPropertyDefaults();
		unsigned long GetProperty(t_cameraProperty* cameraProperty);
		unsigned long PrintCameraInformation();
		unsigned long TestCamera(const char* filename);

		//*******************************************************************************
		// Camera specific functions
		//*******************************************************************************
		
};

} // end namespace ipa_CameraSensors

#endif //__VIRTUALCOLORCAM_H__


