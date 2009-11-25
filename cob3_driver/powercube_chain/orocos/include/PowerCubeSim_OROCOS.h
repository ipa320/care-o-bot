#ifndef POWERCUBE_SIM_OROCOS
#define POWERCUBE_SIM_OROCOS


#include <rtt/TaskContext.hpp>
#include <rtt/Command.hpp>
#include <rtt/Ports.hpp>
#include "simulatedArm.h"
#include <vector>

class simulatedArm;

// NEVER do using namespace in a header file!!
// using namespace RTT;

class PowerCubeSim_OROCOS : public RTT::TaskContext
{
	public:

	PowerCubeSim_OROCOS(std::string name);
	~PowerCubeSim_OROCOS();


	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook(){}
	
	/* The dataports have the following names:
	"in_Angles"
	"in_Velocities"
	"in_Currents"
	"out_Angles"
	"out_Velocities"
	*/


	protected:

	bool stopArm();
	bool isArmStopped();

	RTT::ReadDataPort< std::vector<double> > m_in_Angles;
	bool m_in_Angles_connected;

	RTT::ReadDataPort< std::vector<double> > m_in_Velocities; // MoveVel
	bool m_in_Velocities_connected;

	RTT::ReadDataPort< std::vector<double> > m_in_Currents;
	bool m_in_Currents_connected;

	RTT::WriteDataPort< std::vector<double> > m_out_Angles; // getConfig
	RTT::WriteDataPort< std::vector<double> > m_out_Velocities; // getJointVelocities
	//WriteDataPort< std::vector<double> > m_out_Currents;

	RTT::Method<bool(void)> m_stop_method;

	simulatedArm m_powercubectrl;

};

#endif

