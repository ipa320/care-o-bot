#include "AbstractRangeImagingSensor.h"

using namespace ipa_CameraSensors;

#ifdef __cplusplus
extern "C" {
#endif
__DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ void APIENTRY ReleaseRangeImagingSensor(AbstractRangeImagingSensor* rangeImagingSensor)
{
	delete rangeImagingSensor;
}
#ifdef __cplusplus
}
#endif


AbstractRangeImagingSensor::~AbstractRangeImagingSensor()
{
	if (m_intrinsicMatrix) cvReleaseMat(&m_intrinsicMatrix);
	if (m_distortionParameters)
	{
		cvReleaseMat(&m_distortionParameters);
		cvReleaseImage(&m_undistortMapX);
		cvReleaseImage(&m_undistortMapY);
	}
}

unsigned long AbstractRangeImagingSensor::SetIntrinsicParameters(double fx, double fy, double cx, double cy)
{ //[fx 0 cx; 0 fy cy; 0 0 1]

	if (m_intrinsicMatrix == NULL)
	{	
		m_intrinsicMatrix = cvCreateMatHeader( 3, 3, CV_64FC1 );//Initialisierung
		cvCreateData( m_intrinsicMatrix );	
		cvSet(m_intrinsicMatrix, cvRealScalar(0), NULL);//Defaultwerte:0	
	}

	cvmSet(m_intrinsicMatrix,0,0,fx);
	cvmSet(m_intrinsicMatrix,1,1,fy);
	cvmSet(m_intrinsicMatrix,0,2,cx);
	cvmSet(m_intrinsicMatrix,1,2,cy);
	
	return RET_OK;
}


unsigned long AbstractRangeImagingSensor::GetIntrinsicParameters(CvMat** _intrinsic_matrix)
{
	if (m_intrinsicMatrix == 0)
	{
		return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
	}
	else 
	{
		*_intrinsic_matrix = cvCreateMat( 3, 3, CV_64FC1 );
		cvCopy(m_intrinsicMatrix, *_intrinsic_matrix);
		return RET_OK; 
	}
} 
		
unsigned long AbstractRangeImagingSensor::SetDistortionParameters(double k1, double k2, double p1, double p2, int width, int height)
{ //[k1, k2, p1=0/*hier!!!*/, p2=0/*hier!!!*/]

	if (m_intrinsicMatrix == 0)
	{
		std::cerr << "ERROR - AbstractRangeImagingSensor::SetDistortionParameters:\n";
		std::cerr << "\t ... Could not init undistortion matrix because intrinsic matrix is not set.";
		return RET_FAILED;
	}

	if (m_distortionParameters == NULL)
	{	
		m_distortionParameters = cvCreateMatHeader( 1, 4, CV_64FC1 );//Initialisierung
		cvCreateData( m_distortionParameters );
		cvSet(m_distortionParameters,cvRealScalar(0), NULL);//Defaultwerte:0

		m_undistortMapX = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
		m_undistortMapY = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
	}
	
	cvmSet(m_distortionParameters,0,0,k1);
	cvmSet(m_distortionParameters,0,1,k2);
		
	cvmSet(m_distortionParameters,0,2,p1);
	cvmSet(m_distortionParameters,0,3,p2);

	/*std::cout << "\t ... distortion coeffs \n'"; 
	std::cout << k1 << " " << k2 << " " << p1 << " " << p2 << std::endl;
	std::cout << "\t ... m_intrinsic is \n'"
	<< "\t ... '" << cvmGet(m_intrinsicMatrix,0,0) << "', '"<< cvmGet(m_intrinsicMatrix,0,1) << "', '" << cvmGet(m_intrinsicMatrix,0,2) << "'\n" 
	<< "\t ... '" << cvmGet(m_intrinsicMatrix,1,0) << "', '"<< cvmGet(m_intrinsicMatrix,1,1) << "', '" << cvmGet(m_intrinsicMatrix,1,2) << "'\n" 
	<< "\t ... '" << cvmGet(m_intrinsicMatrix,2,0) << "', '"<< cvmGet(m_intrinsicMatrix,2,1) << "', '" << cvmGet(m_intrinsicMatrix,2,2) << "'\n";
	*/
	ipa_Utils::InitUndistortMap(m_intrinsicMatrix, m_distortionParameters, m_undistortMapX, m_undistortMapY);

	/*for (int i = 70; i < 74; i++)
	{
		for (int j = 0; j < 176; j++)
		{
			std::cout << ((float*)(m_undistortMapX->imageData))[i*176+j] << " ";
		}
		std::cout << "\n \n";
	}*/


	return RET_OK;
}


unsigned long AbstractRangeImagingSensor::GetDistortionParameters(CvMat** _distortion_coeffs)
{
	if (m_distortionParameters == 0)
	{
		return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
	}
	else 
	{
		*_distortion_coeffs = cvCreateMat( 1, 4, CV_64FC1 );
		cvCopy(m_distortionParameters, *_distortion_coeffs);
		return RET_OK; 
	}
}

unsigned long AbstractRangeImagingSensor::RemoveDistortion(const CvArr* src, CvArr* dst)
{
	if ((m_intrinsicMatrix != NULL) && (m_distortionParameters != NULL)) 
	{
		cvRemap(src, dst, m_undistortMapX, m_undistortMapY);
		return RET_OK;
	}

	return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
}
