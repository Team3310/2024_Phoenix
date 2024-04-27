package TrajectoryLib.path;

import java.util.ArrayList;
import java.util.List;

import TrajectoryLib.geometry.GeometryUtil;
import TrajectoryLib.geometry.Vector2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Trajectory {
    private final List<State> states;

    public Trajectory(
            Path path, ChassisSpeeds startingSpeeds, Rotation2d startingRotation) {
        this.states = generateStates(path, startingSpeeds, startingRotation);
    }

    public static List<State> generateStates(Path path, ChassisSpeeds startingSpeeds, Rotation2d startingRotation) {
        List<State> states = new ArrayList<>();

        double prevRotationTargetDist = 0.0;
        Rotation2d prevRotationTargetRot = startingRotation;
        int nextRotationTargetIdx = getNextRotationTargetIdx(path, 0);
        double distanceBetweenTargets = path.getPoint(nextRotationTargetIdx).distanceAlongPath;

        // initial pass: creates states and handles accel
        for (int i = 0; i < path.numPoints(); i++) {
            State state = new State();

            PathConstraints constraints = path.getPoint(i).constraints;
            state.constraints = constraints;

            if (i > nextRotationTargetIdx) {
                prevRotationTargetDist = path.getPoint(nextRotationTargetIdx).distanceAlongPath;
                prevRotationTargetRot = path.getPoint(nextRotationTargetIdx).rotationTarget.getTarget();
                nextRotationTargetIdx = getNextRotationTargetIdx(path, i);
                distanceBetweenTargets = path.getPoint(nextRotationTargetIdx).distanceAlongPath
                        - prevRotationTargetDist;
            }

            RotationTarget nextTarget = path.getPoint(nextRotationTargetIdx).rotationTarget;

            state.targetPose = path.getPoint(i).position.getPose();

            if (i == path.numPoints() - 1) {
                state.deltaPos = path.getPoint(i).distanceAlongPath - path.getPoint(i - 1).distanceAlongPath;
                state.targetVelocity = path.getGoalEndState().getSpeeds();
                state.targetVectorVelocity = new Vector2d(path.getPoint(i).position.getVelocities().vxMetersPerSecond, path.getPoint(i).position.getVelocities().vyMetersPerSecond);
            } else if (i == 0) {
                state.deltaPos = 0;
                state.targetVelocity = startingSpeeds;
                state.targetVectorVelocity = new Vector2d(path.getPoint(i).position.getVelocities().vxMetersPerSecond, path.getPoint(i).position.getVelocities().vyMetersPerSecond);
            } else {
                state.deltaPos = path.getPoint(i + 1).distanceAlongPath - path.getPoint(i).distanceAlongPath;
                state.targetVelocity = path.getPoint(i).position.getVelocities();
                state.targetVectorVelocity = new Vector2d(path.getPoint(i).position.getVelocities().vxMetersPerSecond, path.getPoint(i).position.getVelocities().vyMetersPerSecond);

                double v0 = states.get(states.size() - 1).targetVectorVelocity.getMagnitude();
                double vMax = Math.sqrt(
                        Math.abs(
                                Math.pow(v0, 2)
                                        + (2 * constraints.getMaxAccelerationMpsSq() * state.deltaPos)));
                state.adjustVectorVelocityMagnitude(Math.min(vMax, path.getPoint(i).maxV));
            }

            if (nextTarget.shouldRotateFast()) {
                state.adjustRotation(nextTarget.getTarget());
            } else {
                double t = (path.getPoint(i).distanceAlongPath - prevRotationTargetDist) / distanceBetweenTargets;
                t = Math.min(Math.max(0.0, t), 1.0);
                if (!Double.isFinite(t)) {
                    t = 0.0;
                }

                state.adjustRotation(prevRotationTargetRot.interpolate(nextTarget.getTarget(), t));
            }

            System.out.println(state.targetVelocity);

            states.add(state);
        }

        // second pass: handles deccel
        for (int i = states.size() - 2; i > 1; i--) {
            PathConstraints constraints = states.get(i).constraints;

            double v0 = states.get(i + 1).targetVectorVelocity.getMagnitude();

            double vMax = Math.sqrt(
                    Math.abs(
                            Math.pow(v0, 2)
                                    + (2 * constraints.getMaxAccelerationMpsSq() * states.get(i + 1).deltaPos)));
            states.get(i).adjustVectorVelocityMagnitude(Math.min(vMax, states.get(i).targetVectorVelocity.getMagnitude()));

            System.out.println(states.get(i).targetVelocity);
        }

        double time = 0;
        states.get(0).time = 0;
        states.get(0).targetVelocity.omegaRadiansPerSecond = startingSpeeds.omegaRadiansPerSecond;

        // final pass: calculates time
        for (int i = 1; i < states.size(); i++) {
            double v0 = states.get(i - 1).targetVectorVelocity.getMagnitude();
            double v = states.get(i).targetVectorVelocity.getMagnitude();
            double dt = (2 * states.get(i).deltaPos) / (v + v0);

            System.out.println(states.get(i).targetVelocity);

            time += dt;
            states.get(i).time = time;

            Rotation2d headingDelta = states.get(i).targetPose.getRotation().minus(states.get(i - 1).targetPose.getRotation());
            states.get(i).targetVelocity.omegaRadiansPerSecond = headingDelta.getRadians() / dt;
        }

        return states;
    }

    private static int getNextRotationTargetIdx(Path path, int startingIndex) {
        int idx = path.numPoints() - 1;

        for (int i = startingIndex; i < path.numPoints() - 1; i++) {
            if (path.getPoint(i).rotationTarget != null) {
                idx = i;
                break;
            }
        }

        return idx;
    }

    /**
     * Get the total run time of the trajectory
     *
     * @return Total run time in seconds
     */
    public double getTotaltime() {
        return getEndState().time;
    }

    /**
     * Get the goal state at the given index
     *
     * @param index Index of the state to get
     * @return The state at the given index
     */
    public State getState(int index) {
        return getStates().get(index);
    }

    /**
     * Get the initial state of the trajectory
     *
     * @return The initial state
     */
    public State getInitialState() {
        return getState(0);
    }

    /**
     * Get the end state of the trajectory
     *
     * @return The end state
     */
    public State getEndState() {
        return getState(getStates().size() - 1);
    }

    /**
     * Get the initial target pose for a holonomic drivetrain NOTE: This is a
     * "target" pose, meaning
     * the rotation will be the value of the next rotation target along the path,
     * not what the
     * rotation should be at the start of the path
     *
     * @return The initial target pose
     */
    public Pose2d getInitialTargetPose() {
        return getInitialState().getTargetPose();
    }

    /**
     * Get the target state at the given point in time along the trajectory
     *
     * @param time The time to sample the trajectory at in seconds
     * @return The target state
     */
    public State sample(double time) {
        if (time <= getInitialState().time)
            return getInitialState();
        if (time >= getTotaltime())
            return getEndState();

        int low = 1;
        int high = getStates().size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (getState(mid).time < time) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        State sample = getState(low);
        State prevSample = getState(low - 1);

        if (Math.abs(sample.time - prevSample.time) < 1E-3)
            return sample;

        return prevSample.interpolate(
                sample, (time - prevSample.time) / (sample.time - prevSample.time));
    }

    /**
     * Get all of the pre-generated states in the trajectory
     *
     * @return List of all states
     */
    public List<State> getStates() {
        return states;
    }

    public static class State {
        private double time;
        private Pose2d targetPose;
        private ChassisSpeeds targetVelocity;
        private Vector2d targetVectorVelocity;
        private PathConstraints constraints;

        private double deltaPos;

        /**
         * Interpolate between this state and the given state
         *
         * @param endVal State to interpolate with
         * @param t      Interpolation factor (0.0-1.0)
         * @return Interpolated state
         */
        public State interpolate(State endVal, double t) {
            State lerpedState = new State();

            lerpedState.time = GeometryUtil.doubleLerp(time, endVal.time, t);
            double deltaT = lerpedState.time - time;

            if (deltaT < 0) {
                return endVal.interpolate(this, 1 - t);
            }

            lerpedState.targetVectorVelocity = targetVectorVelocity.interpolate(endVal.targetVectorVelocity, t);
            lerpedState.targetVelocity = new ChassisSpeeds(
                lerpedState.targetVectorVelocity.getX(), 
                lerpedState.targetVectorVelocity.getX(), 
                GeometryUtil.doubleLerp(targetVelocity.omegaRadiansPerSecond, endVal.targetVelocity.omegaRadiansPerSecond, t));
            lerpedState.targetPose = targetPose.interpolate(endVal.targetPose, t);

            if (t < 0.5) {
                lerpedState.constraints = constraints;
            } else {
                lerpedState.constraints = endVal.constraints;
            }

            return lerpedState;
        }

        /**
         * Get the target pose for a holonomic drivetrain NOTE: This is a "target" pose,
         * meaning the
         * rotation will be the value of the next rotation target along the path, not
         * what the rotation
         * should be at the start of the path
         *
         * @return The target pose
         */
        public Pose2d getTargetPose() {
            return targetPose;
        }

        /**
         * Get the target vector velocity
         *
         * @return The target vector velocity
         */
        public Vector2d getTargetVectorVelocity() {
            return targetVectorVelocity;
        }

        public void adjustVectorVelocityMagnitude(double newValue){
            //adjusts magnitude
            targetVectorVelocity.adjustMagnitude(newValue);

            //updates ChassisSpeeds object
            targetVelocity = new ChassisSpeeds(targetVectorVelocity.getX(), targetVectorVelocity.getY(), targetVelocity.omegaRadiansPerSecond);
        }

        public void adjustRotation(Rotation2d newRot){
            targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), newRot);
        }

        /**
         * Get the target velocity
         *
         * @return The target velocity
         */
        public ChassisSpeeds getTargetVelocity() {
            return targetVelocity;
        }

        @Override
        public String toString(){
            return String.format("pose: (%.2f, %.2f) velocity: (%.2f m/s, %.2f m/s, %.2f rads/s)", targetPose.getX(), targetPose.getY(), targetVelocity.vxMetersPerSecond, targetVelocity.vyMetersPerSecond, targetVelocity.omegaRadiansPerSecond);
        }
    }
}
