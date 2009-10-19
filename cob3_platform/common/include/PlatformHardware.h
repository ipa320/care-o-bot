

class PltfHardwareCoB3;

class PlatformHardware
{
public:
    PlatformHardware();
    ~PlatformHardware();

    /**
	 * Initializes system.
	 */
    bool initPltf();

	/// Disable motors, enable brake, disconnect.
    bool shutdownPltf();

    /**
	 * Sets the platform velocity.
	 *
	 * @param dVelLongMMS = Longitudinale Geschwindigkeit der Plattform
	 * @param dVelLatMMS = Transversale Geschwindigkeit der Plattform
	 * @param dRotRobRadS = Rotationsrate der Plattform
	 * @param dRotVelRadS = Rotationsrate des Geschwindigkeitsvektors der Plattform (derzeit nicht benutzt)
	 */
    void setVelPltf(double dVelLongMMS, double dVelLatMMS, double dRotRobRadS, double dRotVelRadS);

	/**
	 * Updates of the can buffer or simulation (deactivated in omnidir. Pltf's - COb3).
	 */
    int update();

	/**
	 * Gets the measured position increment and the measured velocities.
	 */
    void getDeltaPosePltf(double& dDeltaLongMM, double& dDeltaLatMM, double& dDeltaRotRobRad, double& dDeltaRotVelRad,
    					  double& dVelLongMMS, double& dVelLatMMS, double& dRotRobRadS, double& dRotVelRadS);

private:
    PltfHardwareCoB3* d;
};
