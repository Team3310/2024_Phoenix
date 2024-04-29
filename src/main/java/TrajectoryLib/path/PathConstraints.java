package TrajectoryLib.path;

public class PathConstraints {
    private final double maxVelocityMps;
    private final double maxAccelerationMpsSq;
    private final double maxAngularVelocityRps;
    private final double maxAngularAccelerationRpsSq;

    /**
     * Create a new path constraints object
     *
     * @param maxVelocityMps              Max linear velocity (M/S)
     * @param maxAccelerationMpsSq        Max linear acceleration (M/S^2)
     * @param maxAngularVelocityRps       Max angular velocity (Rad/S)
     * @param maxAngularAccelerationRpsSq Max angular acceleration (Rad/S^2)
     */
    public PathConstraints(
            double maxVelocityMps,
            double maxAccelerationMpsSq,
            double maxAngularVelocityRps,
            double maxAngularAccelerationRpsSq) {
        this.maxVelocityMps = maxVelocityMps;
        this.maxAccelerationMpsSq = maxAccelerationMpsSq;
        this.maxAngularVelocityRps = maxAngularVelocityRps;
        this.maxAngularAccelerationRpsSq = maxAngularAccelerationRpsSq;
    }

    /**
     * Get the max linear velocity
     *
     * @return Max linear velocity (M/S)
     */
    public double getMaxVelocityMps() {
        return maxVelocityMps;
    }

    /**
     * Get the max linear acceleration
     *
     * @return Max linear acceleration (M/S^2)
     */
    public double getMaxAccelerationMpsSq() {
        return maxAccelerationMpsSq;
    }

    /**
     * Get the max angular velocity
     *
     * @return Max angular velocity (Rad/S)
     */
    public double getMaxAngularVelocityRps() {
        return maxAngularVelocityRps;
    }

    /**
     * Get the max angular acceleration
     *
     * @return Max angular acceleration (Rad/S^2)
     */
    public double getMaxAngularAccelerationRpsSq() {
        return maxAngularAccelerationRpsSq;
    }
}
