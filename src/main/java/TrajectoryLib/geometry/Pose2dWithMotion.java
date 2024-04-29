package TrajectoryLib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import javafx.geometry.Pos;

public class Pose2dWithMotion implements Interpolatable<Pose2dWithMotion> {
    private ChassisSpeeds velocity;
    private Pose2d pose;

    public Pose2dWithMotion(double x, double y, Rotation2d theta, double dx, double dy, double dtheta) {
        this.pose = new Pose2d(x, y, theta);
        this.velocity = new ChassisSpeeds(dx, dy, dtheta);
    }

    public Pose2dWithMotion(Pose2d pose, ChassisSpeeds velocites) {
        this.pose = pose;
        this.velocity = velocites;
    }

    public ChassisSpeeds getVelocities() {
        return velocity;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    private ChassisSpeeds interpVelocity(ChassisSpeeds other, double t){
        return new ChassisSpeeds(
            GeometryUtil.doubleLerp(velocity.vxMetersPerSecond, other.vxMetersPerSecond, t), 
            GeometryUtil.doubleLerp(velocity.vyMetersPerSecond, other.vyMetersPerSecond, t), 
            GeometryUtil.doubleLerp(velocity.omegaRadiansPerSecond, other.omegaRadiansPerSecond, t));
    }

    @Override
    public Pose2dWithMotion interpolate(Pose2dWithMotion other, double t) {
        return new Pose2dWithMotion(
                pose.interpolate(other.pose, t),
                interpVelocity(other.velocity, t));
    }

    @Override
    public boolean equals(Object o){
        if(o instanceof Pose2dWithMotion){
            Pose2dWithMotion other = (Pose2dWithMotion)o;
            if(other.getVelocities().equals(getVelocities()) && other.getPose().equals(getPose())){
                return true;
            }
        }

        return false;
    }

    @Override
    public String toString(){
        return pose.toString()+"  "+velocity.toString();
    }
}