/*
 * OrocosRTTArmDriverInterface.h
 *
 *  Created on: 03.12.2009
 *      Author: aub
 */

#ifndef OROCOSRTTARMDRIVERINTERFACE_H_
#define OROCOSRTTARMDRIVERINTERFACE_H_
#include "Joint.h"
#include <rtt/TaskContext.hpp>
#include <rtt/Command.hpp>
#include <rtt/Ports.hpp>

using namespace RTT;

class OrocosRTTArmDriverInterface : public TaskContext
{

public:

	Method<void(Jointd)> setMaxVelocity;
	Method<void(float)> setMaxVelocityFloat;
	Method<void(Jointd)> setMaxAcceleration;
	Method<void(float)> setMaxAccelerationFloat;

	ReadDataPort<Jointd>  set_position_inport;
	ReadDataPort<Jointd>  set_velocity_inport;

	WriteDataPort<Jointd>  current_position_outport;
	WriteDataPort<Jointd>  current_velocity_outport;

	OrocosRTTArmDriverInterface(std::string name) : TaskContext(name),
			set_position_inport("SetPositionPort"),
			set_velocity_inport("SetVelocityPort"),
			current_position_outport("CurrentPositionPort"),
			current_velocity_outport("CurrentVelocityPort"),
			setMaxVelocity("setMaxVelocity",
			        &OrocosRTTArmDriverInterface::setMaxVelocityF,
			        this),
			setMaxVelocityFloat("setMaxVelocityFloat",
					&OrocosRTTArmDriverInterface::setMaxVelocityFloatF,
			        this),
			setMaxAcceleration("setMaxAcceleration",
					 &OrocosRTTArmDriverInterface::setMaxAccelerationF,
					 this),
			setMaxAccelerationFloat("setMaxAccelerationFloat",
					 &OrocosRTTArmDriverInterface::setMaxAccelerationFloatF,
			         this)
	{
		this->ports()->addEventPort(&set_position_inport);
		this->ports()->addEventPort(&set_velocity_inport);
		this->ports()->addPort(&current_position_outport);
		this->ports()->addPort(&current_velocity_outport);

		this->methods()->addMethod( &setMaxVelocity,
		                "testing method call for jointd submission.");
		this->methods()->addMethod( &setMaxVelocityFloat,
			            "testing method call for jointd submission.");
		this->methods()->addMethod( &setMaxAcceleration,
			            "testing method call for jointd submission.");
		this->methods()->addMethod( &setMaxAccelerationFloat,
			            "testing method call for jointd submission.");

	}
	~OrocosRTTArmDriverInterface();

private:
	virtual void setMaxVelocityF(Jointd radpersec) = 0;
	virtual void setMaxVelocityFloatF(float radpersec) = 0;
	virtual void setMaxAccelerationF(Jointd radpersec) = 0;
	virtual void setMaxAccelerationFloatF(float radpersec) = 0;


};


#endif /* OROCOSRTTARMDRIVERINTERFACE_H_ */
