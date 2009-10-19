/**@file
 * @brief These classes describe motion commands
 * @author Felix Geibel
 * @date Sept 2007
 */
 
#ifndef __MOVE_COMMAND_H__
#define __MOVE_COMMAND_H__


class RampCommand
{
	public:
		RampCommand(double x0, double v0, double xtarget, double amax, double vmax);
		RampCommand(const RampCommand& rc);
		
		virtual RampCommand& operator=(const RampCommand& rc);
			  
		virtual ~RampCommand() { if (m_nachumkehr) delete m_nachumkehr; }
		
		/// @brief returns the planned position for TimeElapsed (seconds)
		virtual double getPos(double TimeElapsed);
		
		/// @brief returns the planned velocity for TimeElapsed (seconds)
		virtual double getVel(double TimeElapsed);
		
		/// @brief returns the planned total time for the movement (in seconds)
		virtual double getTotalTime();
		
		static void calculateAV(double x0, double v0, double xtarget, double time, double T3, double amax, double vmax, double& a, double& v);
		
		double T1() const { return m_T1; }
		double T2() const { return m_T2; }
		double T3() const { return m_T3; }
		
	private:
		double m_x0, m_v0;
		double m_xtarget;
		double m_amax, m_vmax;
		
		double m_T1, m_T2, m_T3;
		double m_a1, m_v2, m_a3;
		bool m_umkehr;
		RampCommand * m_nachumkehr;
};
		
#endif
