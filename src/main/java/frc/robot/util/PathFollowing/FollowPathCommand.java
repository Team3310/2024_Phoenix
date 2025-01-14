package frc.robot.util.PathFollowing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;

/** Base command for following a path */
public class FollowPathCommand{
  private final Timer timer = new Timer();
  private PathPlannerPath originalPath;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final PathFollowingController controller;
  private final ReplanningConfig replanningConfig;
  private final BooleanSupplier shouldFlipPath;

  // For event markers
  private final Map<Command, Boolean> currentEventCommands = new HashMap<>();
  private final List<EventMarker> untriggeredMarkers = new ArrayList<>();

  private PathPlannerPath path;
  private PathPlannerTrajectory generatedTrajectory;

  /**
   * Construct a base path following command
   *
   * @param path The path to follow
   * @param poseSupplier Function that supplies the current field-relative pose of the robot
   * @param speedsSupplier Function that supplies the current robot-relative chassis speeds
   * @param outputRobotRelative Function that will apply the robot-relative output speeds of this
   *     command
   * @param controller Path following controller that will be used to follow the path
   * @param replanningConfig Path replanning configuration
   * @param shouldFlipPath Should the path be flipped to the other side of the field? This will
   *     maintain a global blue alliance origin.
   * @param requirements Subsystems required by this command, usually just the drive subsystem
   */
  public FollowPathCommand(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      PathFollowingController controller,
      ReplanningConfig replanningConfig,
      BooleanSupplier shouldFlipPath) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.controller = controller;
    this.replanningConfig = replanningConfig;
    this.shouldFlipPath = shouldFlipPath;
  }

  public void initialize(Drivetrain drivetrain, boolean resetPose) {
    System.out.println("should flip:"+shouldFlipPath.getAsBoolean()+"::preventFlip:"+originalPath.preventFlipping);
    if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
      path = originalPath.flipPath();
    } else {
      path = originalPath;
    }

    //TODO test getting rid of this when using targetingOdo @freddytums
    if(resetPose){
        drivetrain.seedFieldRelative(path.getPreviewStartingHolonomicPose());
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    controller.reset(currentPose, currentSpeeds);

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
    Rotation2d currentHeading =
        new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    Rotation2d targetHeading =
        path.getPoint(1).position.minus(path.getPoint(0).position).getAngle();
    Rotation2d headingError = currentHeading.minus(targetHeading);
    boolean onHeading =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
            || Math.abs(headingError.getDegrees()) < 30;

    if (!path.isChoreoPath()
        && replanningConfig.enableInitialReplanning
        && (currentPose.getTranslation().getDistance(path.getPoint(0).position) > 0.25
            || !onHeading)) {
      replanPath(currentPose, currentSpeeds);
    } else {
      generatedTrajectory = path.getTrajectory(currentSpeeds, currentPose.getRotation());
      PathPlannerLogging.logActivePath(path);
      PPLibTelemetry.setCurrentPath(path);
    }

    replanning = true;

    timer.reset();
    timer.start();

    drivetrain.setDriveMode(DriveMode.AUTON);
  }

  
  public ChassisSpeeds update() {
    double currentTime = timer.get();
    // SmartDashboard.putNumber("auton timer", currentTime);
    PathPlannerTrajectory.State targetState = generatedTrajectory.sample(currentTime);
    if (!controller.isHolonomic() && path.isReversed()) {
      targetState = targetState.reverse();
    }

    Pose2d currentPose = poseSupplier.get();
    ChassisSpeeds currentSpeeds = speedsSupplier.get();

    if (!path.isChoreoPath() && replanningConfig.enableDynamicReplanning) {
      double previousError = Math.abs(controller.getPositionalError());
      double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

      //TODO maybe if we are note tracking we ignore this for simplicity
      //if(!TunerConstants.DriveTrain.isTrackingNote){
        if(replanning){
          if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
              || currentError - previousError
                  >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
                    //TODO try increasing this error spike threshold if changing the poseSupplier
                    //doesn't work, maybe whene just getting rid of it @freddytums
            replanPath(currentPose, currentSpeeds);
            timer.reset();
            targetState = generatedTrajectory.sample(0);
          }
        }
      //}
    }

    ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

    // TunerConstants.DriveTrain.seedFieldRelative(targetState.getTargetHolonomicPose());
    // TunerConstants.DriveTrain.seedFieldRelativeWithOffset(targetState.getTargetHolonomicPose().getRotation());

    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    if(Utils.isSimulation()){
      TunerConstants.DriveTrain.seedFieldRelative(targetState.getTargetHolonomicPose());
      TunerConstants.DriveTrain.seedFieldRelativeWithOffset(targetState.getDifferentialPose().getRotation());
    }

    PPLibTelemetry.setCurrentPose(currentPose);
    PathPlannerLogging.logCurrentPose(currentPose);

    if (controller.isHolonomic()) {
      PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
      PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
    } else {
      PPLibTelemetry.setTargetPose(targetState.getDifferentialPose());
      PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
    }

    PPLibTelemetry.setVelocities(
        currentVel,
        targetState.velocityMps,
        currentSpeeds.omegaRadiansPerSecond,
        targetSpeeds.omegaRadiansPerSecond);
    PPLibTelemetry.setPathInaccuracy(controller.getPositionalError());

    return targetSpeeds;
  }

  public void setPath(PathPlannerPath path){
    this.originalPath = path;
    this.path = path;
  }

  private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    PathPlannerPath replanned = path.replan(currentPose, currentSpeeds);
    generatedTrajectory =
        new PathPlannerTrajectory(replanned, currentSpeeds, currentPose.getRotation());
    PathPlannerLogging.logActivePath(replanned);
    PPLibTelemetry.setCurrentPath(replanned);
  }

  private boolean replanning = true;
  public void setReplanning(boolean replan){
    this.replanning = replan;
  }

  public boolean pathDone() {
    return timer.hasElapsed(generatedTrajectory.getTotalTimeSeconds());
  }

  public double getPathTimer() {
    return timer.get();
  }

  public double getPathTime() {
    return generatedTrajectory.getTotalTimeSeconds();
  }

  public void stopPath(){
    timer.stop();
  }

  public void startPath(){
    timer.start();
  }

  public Rotation2d lastAngle() {
      return path.getGoalEndState().getRotation();
  }

  public Pose2d getPathEnd(){
    return generatedTrajectory.getEndState().getTargetHolonomicPose();
  }
}
