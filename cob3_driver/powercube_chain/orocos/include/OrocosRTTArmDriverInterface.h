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
#include <rtt/Method.hpp>

using namespace RTT;

class OrocosRTTArmDriverInterface : public TaskContext
{

public:



	ReadDataPort<Jointd>  set_position_inport;
	ReadDataPort<Jointd>  set_velocity_inport;

	WriteDataPort<Jointd>  current_position_outport;
	WriteDataPort<Jointd>  current_velocity_outport;

	Method<void(Jointd)> setMaxVelocity;
	Method<void(float)> setMaxVelocityFloat;
	Method<void(Jointd)> setMaxAcceleration;
	Method<void(float)> setMaxAccelerationFloat;

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
		                "Setting maximal velocity of joints.", "Jointd", "maximal velocity in rad per second");
		this->methods()->addMethod( &setMaxVelocityFloat,
			            "Setting maximal velocity of joints.", "float", "maximal velocity in rad per second");
		this->methods()->addMethod( &setMaxAcceleration,
			            "Setting maximal acceleration of joints.", "Jointd", "maximal acceleration in rad per second squared");
		this->methods()->addMethod( &setMaxAccelerationFloat,
			            "Setting maximal acceleration of joints.", "float", "maximal acceleration in rad per second squared");

	}
	~OrocosRTTArmDriverInterface() {};

private:
	virtual void setMaxVelocityF(Jointd radpersec) = 0;
	virtual void setMaxVelocityFloatF(float radpersec) = 0;
	virtual void setMaxAccelerationF(Jointd radpersec) = 0;
	virtual void setMaxAccelerationFloatF(float radpersec) = 0;


};


#endif /* OROCOSRTTARMDRIVERINTERFACE_H_ */
