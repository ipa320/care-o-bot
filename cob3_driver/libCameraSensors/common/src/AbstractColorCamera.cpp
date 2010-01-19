#include "AbstractColorCamera.h"

using namespace ipa_CameraSensors;

#ifdef __cplusplus
extern "C" {
#endif
__DLL_ABSTRACTCOLORCAMERA_H__ void APIENTRY ReleaseColorCamera(AbstractColorCamera* colorCamera)
{
	delete colorCamera;
}
#ifdef __cplusplus
}
#endif

AbstractColorCamera::~AbstractColorCamera()
{
}

t_cameraType AbstractColorCamera::GetCameraType()
{
	return m_CameraType;
}

unsigned long AbstractColorCamera::TestCamera(const char* filename)
{
	std::cout << "AbstractColorCamera::TestCamera: Testing camera interface class AbstractColorCamera..." << std::endl;
	std::cout << std::endl; 
	if (!isInitialized())
	{
		std::cout << "AbstractColorCamera::TestCamera: Initializing camera device..." << std::endl;
		if (Init(filename) & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED)) 
		{
			std::cout << "AbstractColorCamera::TestCamera: Initializing camera device...          FAILED" << std::endl;
			return (RET_FAILED | RET_INIT_CAMERA_FAILED);
		}
		std::cout << "AbstractColorCamera::TestCamera: Initializing camera device...          OK" << std::endl;
	}
	std::cout << std::endl;

	if (!isOpen())
	{
		std::cout << "AbstractColorCamera::TestCamera: Opening camera device..." << std::endl;
		if (Open() & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED))
		{
			std::cout << "AbstractColorCamera::TestCamera: Opening camera device...          FAILED" << std::endl;
			return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
		}
		std::cout << "AbstractColorCamera::TestCamera: Opening camera device...          OK" << std::endl;
	}
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: Displaying camera information..." << std::endl;
	unsigned long ret = PrintCameraInformation();
	if (ret & RET_FAILED)
	{
		std::cout << "AbstractColorCamera::TestCamera: Displaying camera information...          FAILED." << std::endl;
		return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
	}
	else if (ret & RET_FUNCTION_NOT_IMPLEMENTED)
	{	
		std::cout << "AbstractColorCamera::TestCamera: Displaying camera information...          NOT IMPLEMENTED" << std::endl;  
	}
	else 
	{
		std::cout << "AbstractColorCamera::TestCamera: Displaying camera information...          OK." << std::endl;
	}
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking isInitialized()..." << std::endl;
	if (!isInitialized()) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking isInitialized()...          FAILED" << std::endl;
		return (RET_FAILED | RET_INIT_CHECK_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking isInitialized()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()..." << std::endl;
	if (!isOpen()) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          FAILED" << std::endl;
		return (RET_FAILED | RET_OPEN_CHECK_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking Close()..." << std::endl;
	if (Close() & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED)) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking Close()...          FAILED" << std::endl;
		return (RET_FAILED | RET_CLOSE_CAMERA_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking Close()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()..." << std::endl;
	if (isOpen()) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          FAILED" << std::endl;
		return (RET_FAILED | RET_OPEN_CHECK_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking Open()..." << std::endl;
	if (Open() & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED)) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking Open()...          FAILED" << std::endl;
		return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking Open()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()..." << std::endl;
	ret = SaveParameters("testSaveParams.xml");
	if (ret & RET_FAILED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()...          FAILED" << std::endl;
		return (RET_FAILED | RET_SAVE_PARAMS_FAILED);
	}
	else if (ret & RET_FUNCTION_NOT_IMPLEMENTED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()...          NOT IMPLEMENTED" << std::endl;
	} 
	else 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()...          OK" << std::endl;
	}
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()..." << std::endl;
	ret = SetPropertyDefaults();
	if (ret & RET_FAILED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()...          FAILED" << std::endl;
		return (RET_FAILED | RET_SET_PROPERTY_DEFAULTS_FAILED);
	}
	else if (ret & RET_FUNCTION_NOT_IMPLEMENTED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()...          NOT IMPLEMENTED" << std::endl;
	} 
	else 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()...          OK" << std::endl;
	}
	std::cout << std::endl;

	return RET_OK;
}
