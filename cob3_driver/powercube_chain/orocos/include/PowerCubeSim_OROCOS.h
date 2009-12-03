#ifndef POWERCUBE_SIM_OROCOS
#define POWERCUBE_SIM_OROCOS


#include "OrocosRTTArmDriverInterface.h"
#include "PowerCubeSim.h"
#include <vector>



class PowerCubeSim_OROCOS : public OrocosRTTArmDriverInterface
{
public:

	PowerCubeSim_OROCOS(std::string name);
	~PowerCubeSim_OROCOS();


	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook(){}

private:

	void setMaxVelocityF(Jointd radpersec){}
	void setMaxVelocityFloatF(float radpersec){}
	void setMaxAccelerationF(Jointd radpersec){}
	void setMaxAccelerationFloatF(float radpersec){}

	PowerCubeSim m_powercubectrl;
	bool stopArm();
	bool isArmStopped();

};

#endif

