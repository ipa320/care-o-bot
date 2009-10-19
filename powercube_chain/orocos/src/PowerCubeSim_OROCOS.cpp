#include "PowerCubeSim_OROCOS.h"
#include "simulatedArm.h"
#include <vector>

using namespace RTT;

PowerCubeSim_OROCOS::PowerCubeSim_OROCOS(std::string name)
: TaskContext(name),
	// initialize dataports:
	m_in_Angles("in_Angles"),
	m_in_Velocities("in_Velocities"),
	m_in_Currents("in_Currents"),
	m_out_Angles("out_Angles"),  // note: initial value
	m_out_Velocities("out_Velocities"),  // note: initial value
	// initialize method
	m_stop_method("stop_movement",&PowerCubeSim_OROCOS::stopArm, this),
	m_powercubectrl()
{
    this->ports()->addPort( &m_in_Angles, "Port for desired Angles" );
    m_in_Angles_connected = false;
    this->ports()->addPort( &m_in_Velocities, "Port for desired velocities" );
    m_in_Velocities_connected = false;
    this->ports()->addPort( &m_in_Currents, "Port for desired currents" );
    m_in_Currents_connected = false;

    this->ports()->addPort( &m_out_Angles, "Port for current positions" );
    this->ports()->addPort( &m_out_Velocities, "Port for current velocities" );

    this->methods()->addMethod(&m_stop_method, "Stop the Arm");
}

PowerCubeSim_OROCOS::~PowerCubeSim_OROCOS()
{
}

bool PowerCubeSim_OROCOS::configureHook()
{
	if ( m_powercubectrl.Init("stringnotused") )
	{
		log(Info) << "PowerCubeSim initialized successfully." << endlog();
		return true;
	}
	else
	{
		log(Info) << "Error while initializing PowerCubeSim:" << endlog();
		log(Info) << m_powercubectrl.getErrorMessage() <<  endlog();
		return false;
	}
}

bool PowerCubeSim_OROCOS::startHook()
{
    if ( m_in_Angles.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Velocities.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Angles_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Angles" << endlog();
	    return true;
    }
    if ( m_in_Velocities.connected() )
    {
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Angles.connected() || m_in_Currents.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();
    		return false;
    	}
    	m_in_Velocities_connected = true;
    	log(Info) << "PowerCubeSim succesfully detected connection at in_Velocities" << endlog();
	    return true;
    }
    if ( m_in_Currents.connected() )
    {
    	log(Info) << "Error, port \"setCur_R\" is connected to PowerCubeSim_OROCOS.\"";
    	log(Info) << "Current movement is not yet supported by PowerCubeSim." << endlog();
    	return false;
    	/*
    	// only one of the input ports should be used simultaniously
    	if ( m_in_Angles.connected() || m_in_Velocities.connected() ) {
    		log(Info) << "Error in PowerCubeSim.startHook(): more than one input port is connected!" << endlog();    		
    		return false;
    	}
    	m_in_Current_connected = true;
	    return true;
	    */
    }
    else
    {
    	log(Info) << "No Input port is connected, PowerCubeSim will only return the current state." << endlog();
    }
    return true;
}

void PowerCubeSim_OROCOS::updateHook()
{
	//log(Info) << "updateHook is being executed." << endlog();
	
	if ( m_in_Angles_connected )
	{
		std::vector<double> angles_desired(7);
		angles_desired = m_in_Angles.Get();
	    m_powercubectrl.MovePos( angles_desired );
	    //log(Info) << "MovePos Command Executed" << endlog();
	}
	
	if ( m_in_Velocities_connected )
	{
		std::vector<double> vel_desired(7);
		vel_desired = m_in_Velocities.Get();
	    m_powercubectrl.MoveVel( vel_desired );
	    //log(Info) << "MoveVel Command Executed" << endlog();
	}
	/*
	if ( m_in_Current_connected )
	{
		std::vector<double> current_desired(7);
		current_desired = m_in_Current.Get();
	    m_powercubectrl.MoveCur( current_desired );
	}
	*/
	

    // get current angles & velocities
    std::vector<double> curConfig(7);
	m_powercubectrl.getConfig(curConfig);
    m_out_Angles.Set(curConfig);

	/*
	log(Info) << "pos: (";
	for (unsigned int  i=0; i<curConfig.size(); i++)
		log(Info) << curConfig[i] << ", ";
	log(Info) << ")" << endlog();
	*/
	
    std::vector<double> curVelocities(7);
    m_powercubectrl.getJointVelocities(curVelocities);
    m_out_Velocities.Set(curVelocities);	
}

void PowerCubeSim_OROCOS::stopHook()
{
	stopArm();
}

bool PowerCubeSim_OROCOS::stopArm()
{
    //stop
    m_powercubectrl.Stop();		
    return true;
}

/*bool PowerCubeSim_OROCOS::stopCondition()		
{
    return true;
}*/

bool PowerCubeSim_OROCOS::isArmStopped()
{
    //isStopped
    if(m_powercubectrl.statusMoving())
        return false;
    return true;
}
