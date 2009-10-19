/**@file
 * @brief This class simulates a PowerCube
 * @author Ulrich Reiser
 *  extended by Felix Geibel
 * @date August 2007
 */

#include "simulatedMotor.h"
#include <math.h>

#include <rtt/TaskContext.hpp>
using namespace RTT;
		
simulatedMotor::simulatedMotor(double lowLimit, double upLimit, double maxAcc, double maxVel)
	: m_lastMove(0, 0, 0, 1, 1)
{
	m_ll = lowLimit;
	m_ul = upLimit;
	m_amax = fabs(maxAcc);
	m_vmax = fabs(maxVel);
	//deb = NULL;
	m_lastMove.start(); // Will be finished immediately, so that motor is at x=0, v=0
	T0 = 0.0008;
}

/////////////////////////////////////////
// Zunächst die Steuerungs-Funktionen: //
/////////////////////////////////////////

/// @brief executes a rampmove
void simulatedMotor::moveRamp(double targetAngle, double vmax, double amax)
{
	double x = m_lastMove.pos();
	double v = m_lastMove.vel();
	
	// Range Check:
	if ( targetAngle < m_ll ) targetAngle = m_ll;
	else if ( targetAngle > m_ul ) targetAngle = m_ul;
	
	double vm = fabs(vmax);
	double am = fabs(amax);
	
	if ( vm > m_vmax ) vm = m_vmax;
	if ( am > m_amax ) am = m_amax;
	
	m_lastMove = RampCommand(x, v, targetAngle, am, vm);
	m_lastMove.start();
}


/// @brief Moves the motor with the given velocity
void simulatedMotor::moveVel(double vel)
{
	double x = m_lastMove.pos();
	double v = m_lastMove.vel();
	
	double targetAngle;
	
	// Move with constant v is RampMove to the corresponding limit!
	if ( vel > 0 ) 
		targetAngle = m_ul;
	else if ( vel < 0)
		targetAngle = m_ll;
	else
		targetAngle = x; // v=0, stay at current position
	
	double vm = fabs(vel);
	if ( vm > m_vmax ) vm = m_vmax;	
	
	double a = fabs(vel-v) / T0;
	// RampCommand does not work properly if a is zero!
	if ( a < 0.001 )
	{
		a = m_amax;
	}
	// this is just a workaround. It would be desirable to simulate a PT2 element!
	
	//log(Info) << "RampCommand(" << x << ", " << v << ", " << targetAngle << ", " << a;
	//log(Info) << ", " << vm << ")" << endlog();
	
	m_lastMove = RampCommand(x, v, targetAngle, a, vm);
	m_lastMove.start();
}


/// @brief Moves the motor with the given velocity
void simulatedMotor::movePos(double pos)
{
	double x = m_lastMove.pos();
	double v = m_lastMove.vel();
	
	double targetAngle = pos;

	// Range Check:
	if ( targetAngle < m_ll ) targetAngle = m_ll;
	else if ( targetAngle > m_ul ) targetAngle = m_ul;
	
	double vm = fabs(m_vmax);
	
	// move backwards?
	if ( pos < x )
		vm = -vm;
	
	double a = fabs(vm-v) / T0;
	
	m_lastMove = RampCommand(x, v, targetAngle, a, vm);
	m_lastMove.start();
}


/// @brief Stops the motor immediately
void simulatedMotor::stop()
{
	// Stops immediately (would not be possible with a real motor)
	double x = m_lastMove.pos();
	double v = 0;
	
	m_lastMove = RampCommand(x, v, x, 1, 1);
	m_lastMove.start();
}

/// @brief Calculate time that a rampMove WOULD take (no movement is executed)
RampCommand simulatedMotor::getRampMove(double targetAngle, double vmax, double amax)
{
	double x = m_lastMove.pos();
	double v = m_lastMove.vel();
	
	// Range Check:
	if ( targetAngle < m_ll ) targetAngle = m_ll;
	else if ( targetAngle > m_ul ) targetAngle = m_ul;
	
	double vm = fabs(vmax);
	double am = fabs(amax);
	
	if ( vm > m_vmax ) vm = m_vmax;
	if ( am > m_amax ) am = m_amax;
	
	RampCommand rc(x, v, targetAngle, am, vm);
	return rc;
}
