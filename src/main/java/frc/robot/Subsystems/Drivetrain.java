package frc.robot.Subsystems;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Swerve.SwerveDrivetrain;
import frc.robot.Swerve.SwerveModule;
import frc.robot.Swerve.SwerveModule.DriveRequestType;
import frc.robot.Swerve.SwerveRequest;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.UpdateManager;
import frc.robot.util.Camera.Limelight;
import frc.robot.util.Camera.LimelightHelpers;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.Target;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Control.PidConstants;
import frc.robot.util.Control.PidController;
import frc.robot.util.Control.TimeDelayedBoolean;
import frc.robot.util.PathFollowing.FollowPathCommand;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem, UpdateManager.Updatable {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private DriveMode mControlMode = DriveMode.JOYSTICK;
    private SideMode sideMode = SideMode.RED;
    private String drivetrain_state = "INIT";

    private PidController aimAtTargetController = new PidController(new PidConstants(1.0, 0.0, 0.01));
    private PidController noteTrackController = new PidController(new PidConstants(0.4, 0.00, 0.02));
    private PidController joystickController = new PidController(new PidConstants(1.0, 0, 0.0));

    private Limelight limelight = new Limelight("front");
    private Limelight noteLimelight = new Limelight("note");
    private Targeting frontCamera = new Targeting("front", false);
    private Targeting odometryTargeting = new Targeting(true);

    private int pidVisionUpdateCounter = 0;
    private final int VISON_COUNTER_MAX = 4;
    private int pidNoteUpdateCounter = 0;
    private final int NOTE_COUNTER_MAX = 4;

    private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;

    private FollowPathCommand pathFollower;

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean isSnapping;
    private double mLimelightVisionAlignGoal;
    private double mGoalTrackVisionAlignGoal;
    private double mVisionAlignAdjustment;

    private ProfiledPIDController snapPIDController;
    private ProfiledPIDController snapPIDControllerAuton;
    private PIDController visionPIDController;

    public enum DriveOrientation {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    private DriveOrientation driveOrientation = DriveOrientation.FIELD_CENTRIC;

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        aimAtTargetController.setContinuous(true);
        aimAtTargetController.setInputRange(-Math.PI, Math.PI);
        aimAtTargetController.setOutputRange(-1.0, 1.0);

        noteTrackController.setContinuous(true);
        noteTrackController.setInputRange(-Math.PI, Math.PI);
        noteTrackController.setOutputRange(-1.0, 1.0);

        joystickController.setContinuous(true);
        joystickController.setInputRange(-Math.PI, Math.PI);
        joystickController.setOutputRange(-1.0, 1.0);
        joystickController.setSetpoint(0.0);

        Targeting.setTarget(Target.REDSPEAKER);

        snapPIDController = new ProfiledPIDController(Constants.SnapConstants.kP, Constants.SnapConstants.kI,
                Constants.SnapConstants.kD, Constants.SnapConstants.kThetaControllerConstraints);
        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        snapPIDControllerAuton = new ProfiledPIDController(Constants.SnapAutonConstants.kP, Constants.SnapAutonConstants.kI,
                Constants.SnapAutonConstants.kD, Constants.SnapAutonConstants.kThetaControllerConstraints);
        snapPIDControllerAuton.enableContinuousInput(-Math.PI, Math.PI);

        visionPIDController = new PIDController(Constants.VisionAlignConstants.kP, Constants.VisionAlignConstants.kI,
                Constants.VisionAlignConstants.kD);
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
        visionPIDController.setTolerance(0.0);

        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        ReplanningConfig replan = new ReplanningConfig();

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                new PIDConstants(5.0, 0, 0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                new ReplanningConfig());

        pathFollower = new FollowPathCommand(
                () -> this.getPose(),
                this::getCurrentRobotChassisSpeeds,
                new PPHolonomicDriveController(
                        config.translationConstants, config.rotationConstants, config.period, config.maxModuleSpeed,
                        config.driveBaseRadius),
                config.replanningConfig,
                () -> false);

        configurePathPlanner();
        // if (Utils.isSimulation()) {
        //     startSimThread();
        // }
    }

    //#region DriveTrain things... 
    private void fromChassisSpeed(ChassisSpeeds speeds) {
        var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for (int i = 0; i < this.Modules.length; i++) {
            this.Modules[i].apply(states[i],
                    SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        }
    }

    public boolean canSeeNote(){
        return noteLimelight.hasTarget();
    }

    public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
        this.setControl(requestSupplier.get());
    }

    //#endregion

    // public boolean odosnap = false;
    public void setDriveMode(DriveMode mode) {
        // odosnap = false;
        if (mode == mControlMode){
            return;
        }

        aimAtTargetController.integralAccum = 0;
        noteTrackController.integralAccum = 0;
        joystickController.integralAccum = 0;

        if (lockedOn) {
            lockedOn = false;
        }

        // Runs once on mode change to JOYSTICK, to set the current field-relative yaw
        // of the robot to the hold angle.
        if (mode == DriveMode.JOYSTICK) {
            setJoystickDrive_holdAngle(getBotAz_FieldRelative());
            maybeStopSnap(true);
        } else if (mode == DriveMode.AIM_AT_NOTE){
            pidNoteUpdateCounter = NOTE_COUNTER_MAX + 1;
        } else if (mode == DriveMode.AIMATTARGET){
            // Get JSON Dump from Limelight-front
            LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

            // Go through limelight JSON dump, and look for Target ID
            // If ID found, save TX value to offset for targeting.
            pidVisionUpdateCounter = VISON_COUNTER_MAX + 1;
            boolean canSeeTarget = false;
            double offset = 0;
            for (var aprilTagResults : llresults.targetingResults.targets_Fiducials) {
                if (aprilTagResults.fiducialID == Targeting.getTargetID()) {
                    offset = -(Math.toRadians(aprilTagResults.tx));
                    canSeeTarget = true;
                }
            }
            if (canSeeTarget) {
                if (Math.abs(Math.toDegrees(offset)) > 3.0) {
                    drivetrain_state = "LIME SNAP";
                    startSnap(Math.toDegrees(getBotAz_FieldRelative() -  offset));
                }
            }
            else {
                // odosnap = true;
                drivetrain_state = "NO TARGET";
                
                // startSnap(Math.toDegrees(odometryTargeting.getAz() + Math.PI));
            }
        }

        mControlMode = mode;
    }

    public void snapToAngleAuton(double angle){
        if (sideMode == SideMode.RED) {
            angle = rolloverConversion_radians(angle + (Math.PI / 2));
        } else if (RobotContainer.getInstance().getSide() == SideMode.BLUE) {
            angle = rolloverConversion_radians(angle - (Math.PI / 2));
        }
        setDriveMode(DriveMode.AIMATTARGET_AUTON);
        SmartDashboard.putNumber("snap auton angle", Math.toDegrees(rolloverConversion_radians(angle+Math.PI)));
        startSnap(Math.toDegrees(rolloverConversion_radians(angle+Math.PI)));
    }

    //#region DriveTrain methods

    public void joystickDrive_OpenLoop(double rotation) {
        if (driveOrientation == DriveOrientation.FIELD_CENTRIC) {        
            this.applyRequest(() -> driveFieldCentric
                .withVelocityX(getDriveX() * Constants.MaxSpeed) 
                .withVelocityY(getDriveY() * Constants.MaxSpeed) 
                .withRotationalRate(rotation * Constants.MaxAngularRate) 
            );
        }
        else {
            this.applyRequest(() -> driveRobotCentric
                .withVelocityX(getDriveX() * Constants.MaxSpeed) 
                .withVelocityY(getDriveY() * Constants.MaxSpeed) 
                .withRotationalRate(rotation * Constants.MaxAngularRate) 
            );
        }
    }

    // joystickDrive_holdAngle:
    // Uses PID to hold robot at its current heading, while allowing translation
    // movement.
    // The robot is able to spin by using the right joystick.
    // When no input on the right joystick, PID will hold latest bearing.
    private double joystickDrive_holdAngle = 0;
    public void setJoystickDrive_holdAngle(double radians) {
        joystickDrive_holdAngle = rolloverConversion_radians(radians);
    }

    public void joystickDrive() {
        //Set rotation to right joystick rotation value
        //If snapping is occuring, check if joystick is being moved...
            //If not, then continue following the snaps profile..
            //If so, force stop the snap profile
        //Then ... 
        //If not snapping...
            //AND the right joystick is NOT being used...
                //Use the gyro's current angle and joystickDrive_holdAngle in a PID to hold the current angle
            //else, joystick is moving, so update the hold angle with the current bot angle
        //Then send rotation command to the joystickDrive_OpenLoop.
        double rotation = getDriveRotation();
        if (isSnapping) {
            // if (Math.abs(getDriveRotation()) == 0.0) {
                maybeStopSnap(false);
                rotation = -calculateSnapValue();
            // } else {
            //     maybeStopSnap(true);
            // }
        } else {
            // if (rotation == 0) {
            //     drivetrain_state = "JOYSTICKDRIVE_PID";
            //     double offset = -(getBotAz_FieldRelative() - joystickDrive_holdAngle);
            //     rotation = joystickController.calculate(offset, 0.005) * Constants.MaxAngularRate;
            //     SmartDashboard.putNumber("PID Error:", offset);
            //     SmartDashboard.putNumber("PID Output:", rotation);
            // } else {
                drivetrain_state = "JOYSTICKDRIVE_OPENLOOP";
                joystickDrive_holdAngle = getBotAz_FieldRelative();
            // }
        }
        joystickDrive_OpenLoop(rotation);
    }

    // aimAtTarget():
    // Combines both OdometryTrack and AprilTagTrack()
    // If the targeted aprilTag is in view ... aprilTagTrack() is used.
    // Otherwise, OdometryTrack() is ran, the goal of which is to use
    // OdometryTrack() to point
    // ... the limelight at where the targeted april tag should be.
    // aimAtTarget() needs:
    // Will need everything that odometryTrack() and aprilTagTrack() need.
    // As well as the variables listed below...
    public int updateCounter = 0;
    public boolean justChanged = false;
    public boolean lockedOn = false;
    public int timeout_counter = 0;

    public void aimAtTarget() {
        // Get JSON Dump from Limelight-front
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

            boolean canSeeTarget = false;

            // Go through limelight JSON dump, and look for Target ID
            // If ID found, save TX value to offset for targeting.
            double offset = 0;
            for (var aprilTagResults : llresults.targetingResults.targets_Fiducials) {
                if (aprilTagResults.fiducialID == Targeting.getTargetID()) {
                    offset = (Math.toRadians(aprilTagResults.tx));
                    canSeeTarget = true;
                }
            }
        if(isSnapping){
            // if(canSeeTarget && odosnap){
            //     if((Math.abs(Math.toDegrees(offset)) < 5)){
            //         maybeStopSnap(true);
            //     } else {
            //         joystickDrive_OpenLoop(-calculateSnapValue());
            //     }
            // }else{
                maybeStopSnap(false);
                joystickDrive_OpenLoop(-calculateSnapValue());
            // }
        } else {
            // // If TargetID cant be seen by Limelight, use odometryTrack()
            // if ((!canSeeTarget) && (!lockedOn)){
            //     drivetrain_state = "ODOMETRYTRACK";
            //     odometryTrack();
            //     if (justChanged) {
            //         justChanged = false;
            //     }
            // }else if((!canSeeTarget) && (lockedOn)){
            //     drivetrain_state = "LIME LOST MODE";
            //     offset = -(getBotAz_FieldRelative() - joystickDrive_holdAngle);
            //     double rotation = joystickController.calculate(offset, 0.02) * Constants.MaxAngularRate;
            //     SmartDashboard.putNumber("PID Error:", offset);
            //     SmartDashboard.putNumber("PID Output:", rotation);
            //     joystickDrive_OpenLoop(rotation);
            // }else { // If TargetID can be seen, use Limelight TX tracking
                if (canSeeTarget) {
                    if (!justChanged) { // When changing modes, clear integral accumulation
                        aimAtTargetController.integralAccum = 0;
                        justChanged = true;
                        lockedOn = true;
                    }
                    drivetrain_state = "LIME MODE";

                    double currentAngle = getBotAz_FieldRelative();
                    double targetOffset = offset;
                    mLimelightVisionAlignGoal = MathUtil.inputModulus(currentAngle - targetOffset, 0.0, 2 * Math.PI);
                    
                    if (pidVisionUpdateCounter > VISON_COUNTER_MAX) {
                        aimAtTargetController.setSetpoint(mLimelightVisionAlignGoal);
                        pidVisionUpdateCounter = 0;
                    }
                    pidVisionUpdateCounter++;
                    mVisionAlignAdjustment = aimAtTargetController.calculate(currentAngle, 0.005);

                    // joystickDrive_holdAngle = getBotAz_FieldRelative();
                    // aprilTagTack(), but offset is grabbed earlier due to this drive methoed
                    // needing to determine if target is in view.
                    
                    Double rotation = mVisionAlignAdjustment;

                    SmartDashboard.putNumber("PID Output:", rotation);
                    SmartDashboard.putNumber("PID Error:", offset);

                    joystickDrive_OpenLoop(rotation);
                }
                else {
                    joystickDrive();
                }
            // }
        }
    }

    public void trapQueen(){
        Orchestra trapQUEEN = new Orchestra();
        for(SwerveModule m:Modules){
            trapQUEEN.addInstrument(m.getDriveMotor());
            trapQUEEN.addInstrument(m.getSteerMotor());
        }

        trapQUEEN.loadMusic("TrapQueen.chrp");
        trapQUEEN.play();
    }

    public void aimAtTargetAuton() {
        if(isSnapping){
            maybeStopSnap(false);

            Double request = -calculateSnapValue();

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                    0.0,
                    0.0,
                    request * Constants.MaxAngularRate,
                    m_odometry.getEstimatedPosition()
                            .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()),
                    0.005);

            var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < this.Modules.length; i++) {
                this.Modules[i].apply(states[i],
                        SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
            }
        } else { // If TargetID can be seen, use Limelight TX tracking
            {}; //stop moving
        }
    }

     public void aimAtNote() {
        // Get JSON Dump from Limelight-fron
        if(noteLimelight.hasTarget()){
            double targetOffset = Math.toRadians(noteLimelight.getTargetHorizOffset());
 
            drivetrain_state = "NOTE MODE";

            double currentAngle = getBotAz_FieldRelative();
            double adjustAngle = MathUtil.inputModulus(currentAngle - targetOffset, 0.0, 2 * Math.PI);

            if (pidNoteUpdateCounter > NOTE_COUNTER_MAX) {
                noteTrackController.setSetpoint(adjustAngle);
                pidNoteUpdateCounter = 0;
            }
            pidNoteUpdateCounter++;
            double pidOutput = noteTrackController.calculate(currentAngle, 0.005);
                
            setDriveOrientation(DriveOrientation.ROBOT_CENTRIC);
            joystickDrive_OpenLoop(pidOutput);         
        }
        else {
            setDriveOrientation(DriveOrientation.FIELD_CENTRIC);
            joystickDrive();
        }
    }

    public boolean isTrackingNote = false;
    private void autonDrive(){
        ChassisSpeeds speeds = pathFollower.update();

        if(isTrackingNote){
            double offset = noteLimelight.getTargetHorizOffset();
            double request = noteTrackController.calculate(offset, 0.005);
            speeds.omegaRadiansPerSecond = request*Constants.MaxAngularRate;
        }

        var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());
        if (!pathDone()) {
            for (int i = 0; i < this.Modules.length; i++) {
                this.Modules[i].apply(states[i],
                        SwerveModule.DriveRequestType.OpenLoopVoltage,
                        SwerveModule.SteerRequestType.MotionMagic);
            }
        } else {
            // rotationHold();
        }
    }
    
    //#endregion

    //#region getters
    private double getDriveX() {
        // return ((Math.abs(joystick.getLeftY()) > 0.1) ? -Math.copySign(Math.pow(joystick.getLeftY(), 2.0), joystick.getLeftY()) : 0.0);
        return -Math.copySign(Math.pow(joystick.getLeftY(), 2.0), joystick.getLeftY());
    }

    private double getDriveY() {
        // return ((Math.abs(joystick.getLeftX()) > 0.1) ? -Math.copySign(Math.pow(joystick.getLeftX(), 2.0), joystick.getLeftX()) : 0.0);
        return -Math.copySign(Math.pow(joystick.getLeftX(), 2.0), joystick.getLeftX());
   }

    private double getDriveRotation() {
        // return ((Math.abs(joystick.getRightX()) > 0.1) ? -Math.copySign(Math.pow(joystick.getRightX(), 2.0), joystick.getRightX()) : 0.0);
        return -Math.copySign(Math.pow(joystick.getRightX(), 2.0), joystick.getRightX());
    }

    public Pose2d getOdoPose() {
        return this.m_odometry.getEstimatedPosition();
    }

    public SideMode getSideMode() {
        return sideMode;
    }

    public double getBotAz_FieldRelative() {
        // Note from James to Zac, I'm changing this back to the 'old' one to see if it
        // works...
        return rolloverConversion_radians(
                this.m_fieldRelativeOffset.getRadians() - getPose().getRotation().getRadians());
        // return
        // rolloverConversion_radians(this.m_fieldRelativeOffset.getRadians()-getOdoPose().getRotation().getRadians());
    }

    public ChassisSpeeds getFieldRelativeVelocites() {
        ChassisSpeeds robotChassisSpeeds = m_kinematics.toChassisSpeeds(m_cachedState.ModuleStates);
        Double velocity = Math.hypot(robotChassisSpeeds.vxMetersPerSecond, robotChassisSpeeds.vyMetersPerSecond);
        Double angle = getBotAz_FieldRelative();
        ChassisSpeeds fieldRelativeVelocites = new ChassisSpeeds(velocity * Math.cos(angle), velocity * Math.sin(angle),
                robotChassisSpeeds.omegaRadiansPerSecond);
        return fieldRelativeVelocites;
    }

    public Pose2d getPose() {
        // Note from James to Zac, I'm changing this back to the 'old' one to see if it
        // works...
        return m_odometry.getEstimatedPosition();
        // return new Pose2d(this.m_odometry.getEstimatedPosition().getTranslation(),
        // Rotation2d.fromRadians(getBotAz_FieldRelative()));
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(rolloverConversion_radians(
                getPose().getRotation().getRadians() - this.m_fieldRelativeOffset.getRadians()));
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }

    public Targeting getOdoTargeting() {
        return odometryTargeting;
    }

    public Targeting getLimelightTargeting() {
        return frontCamera;
    }

    public DriveMode getDriveMode() {
        return mControlMode;
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    
    //#endregion getters

    //#region auton stuff
    
    public boolean pathDone() {
        return pathFollower.pathDone();
    }

    public double getPathTime() {
        return pathFollower.getPathTime();
    }

    public void setPath(PathPlannerPath path, boolean resetPose) {
        pathFollower.setPath(path);
        pathFollower.initialize(this, resetPose);
    }

    public void stopPath() {
        pathFollower.stopPath();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> fromChassisSpeed(speeds), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig(true, true, 1.0, 0.25)),
                () -> false, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command getAutoPath(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        // return new PathPlannerAuto(pathName);
    }

    public boolean canSeeTargetTag(){
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

        for (var aprilTagResults : llresults.targetingResults.targets_Fiducials) {
            if (aprilTagResults.fiducialID == Targeting.getTargetID()) {
                return true;
            }
        }
        return false;
    }
    //#endregion auto stuff

    public enum DriveMode {
        JOYSTICK,
        // JOYSTICK_BOTREL,
        AUTON,
        AIMATTARGET, 
        AIMATTARGET_AUTON,
        AIMATTRAP,
        AIM_AT_NOTE
        // ODOMETRYTRACK,
        ;
    }

    @Override
    public void periodic() {
        // if (mControlMode != DriveMode.AUTON && mControlMode != DriveMode.AIMATTARGET_AUTON) {
        //     odometryTargeting.update();
        //     frontCamera.update();

        //     //#region Odometry reSeeds with Limelight BotPos periodically
        //     if (periodicCounter == 100) {
        //         odometryBotPosUpdaterMethodFlag = true;
        //     }
        //     periodicCounter++;
        //     if (odometryBotPosUpdater()) {
        //         periodicCounter = 0;
        //         odometryBotPosUpdaterMethodFlag = false;
        //     }
        //     //#endregion 

        // }
        LimelightHelpers.getLatestResults("limelight-front");
        frontCamera.update();

        if (sideMode != RobotContainer.getInstance().getSideChooser().getSelected()) {
            sideMode = RobotContainer.getInstance().getSideChooser().getSelected();
            Targeting.setTarget(sideMode == SideMode.BLUE ? Target.BLUESPEAKER : Target.REDSPEAKER);
        }

        SmartDashboard.putString("side", sideMode.toString());
        
        SmartDashboard.putNumber("notetx", noteLimelight.getTargetHorizOffset());

        if (Constants.debug) {

            SmartDashboard.putBoolean("is snapping", isSnapping);

            SmartDashboard.putString("DriveTrain State", drivetrain_state);
            SmartDashboard.putNumber("joystickDrive_holdAngle", joystickDrive_holdAngle);
            SmartDashboard.putNumber("getBotAz_FieldRelative()", getBotAz_FieldRelative());
            // SmartDashboard.putNumber("odometryTargeting.getAz()", odometryTargeting.getAz());
            // SmartDashboard.putNumber("odometryTargeting.getEl()", odometryTargeting.getEl());
            SmartDashboard.putNumber("limelightTargeting.getEl()", frontCamera.getEl());
            SmartDashboard.putString("", mControlMode.toString());

            // SmartDashboard.putNumber("getPose().getX()", getPose().getX());
            // SmartDashboard.putNumber("getPose().getY()", getPose().getY());
            // SmartDashboard.putNumber("odometryTargeting.getBotPosX()", odometryTargeting.getBotPosX());
            // SmartDashboard.putNumber("odometryTargeting.getBotPosY()", odometryTargeting.getBotPosY());

            SmartDashboard.putString("Set Target:", Targeting.getTarget().toString());
            SmartDashboard.putNumber("Bot Azimuth:", rolloverConversion_radians(
                    getPose().getRotation().getRadians() - this.m_fieldRelativeOffset.getRadians()));

            // if (SmartDashboard.getNumber("P", 1.0) != aimAtTargetController.getPidConstants().p) {
            //     aimAtTargetController.setP(SmartDashboard.getNumber("P", 1.0));
            // }
            // if (SmartDashboard.getNumber("I", 1.0) != aimAtTargetController.getPidConstants().i) {
            //     aimAtTargetController.setI(SmartDashboard.getNumber("I", 1.0));
            // }
            // if (SmartDashboard.getNumber("D", 1.0) != aimAtTargetController.getPidConstants().d) {
            //     aimAtTargetController.setD(SmartDashboard.getNumber("D", 1.0));
            // }

            SmartDashboard.putString("field velocites", getFieldRelativeVelocites().toString());
            SmartDashboard.putNumber("mVisionAlignAdjustment", mVisionAlignAdjustment);
        }
    }

    @Override
    public void update(double time, double dt) {
        chooseVisionAlignGoal();
        switch (mControlMode) {
            case AIMATTARGET:
                aimAtTarget();
                break;
            case AIMATTARGET_AUTON:
                aimAtTargetAuton();
                break;
            case JOYSTICK:
                joystickDrive();
                break;
            case AIM_AT_NOTE:
                aimAtNote();
                break;
            // case JOYSTICK_BOTREL:
            //     joystickDrive_RobotRelative();
            //     break;
            // case ODOMETRYTRACK:
            //     odometryTrack();
            //     break;
            case AUTON:
                autonDrive();
                break;
        }
    }

    public static double rolloverConversion_radians(double angleRadians) {
        // Converts input angle to keep within range -pi to pi
        if (angleRadians > Math.PI) {
            return (((angleRadians + Math.PI) % (2 * Math.PI)) - Math.PI);
        } else if ((angleRadians < -Math.PI)) {
            return (((angleRadians + Math.PI) % (2 * Math.PI)) - Math.PI);
        } else {
            return angleRadians;
        }
    }

    //#region 1678 Snap/Camera Code    
    public void visionAlignDrive(Translation2d translation2d, boolean fieldRelative) {
        // drive(translation2d, mVisionAlignAdjustment, fieldRelative, false);
    }

    public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative) {
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
        double angleAdjustment = snapPIDController.calculate(getBotAz_FieldRelative());
        // drive(translation2d, angleAdjustment, fieldRelative, false);
    }

    public void acceptLatestGoalTrackVisionAlignGoal(double vision_goal) {
        mGoalTrackVisionAlignGoal = vision_goal;
    }

    public void chooseVisionAlignGoal() {
        double currentAngle = getBotAz_FieldRelative();
        if (limelight.hasTarget()) {
            double targetOffset = frontCamera.getAz();
            mLimelightVisionAlignGoal = MathUtil.inputModulus(currentAngle - targetOffset, 0.0, 2 * Math.PI);
            visionPIDController.setSetpoint(mLimelightVisionAlignGoal);
        } else {
            visionPIDController.setSetpoint(mGoalTrackVisionAlignGoal);
        }

        mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
    }
    

    public double calculateSnapValue() {
        if(mControlMode == DriveMode.AIMATTARGET_AUTON){
            return snapPIDControllerAuton.calculate(getBotAz_FieldRelative());
        }
        return snapPIDController.calculate(getBotAz_FieldRelative());
    }

    public void startSnap(double snapAngle) {
        if(mControlMode == DriveMode.AIMATTARGET_AUTON){
            joystickDrive_holdAngle = Math.toRadians(snapAngle);
            snapPIDControllerAuton.reset(getBotAz_FieldRelative());
            snapPIDControllerAuton.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
            isSnapping = true;
        }else{
            joystickDrive_holdAngle = Math.toRadians(snapAngle);
            snapPIDController.reset(getBotAz_FieldRelative());
            snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
            isSnapping = true;
        }
    }

    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    public boolean snapComplete() {
        if(mControlMode == DriveMode.AIMATTARGET_AUTON){
            double error = snapPIDControllerAuton.getGoal().position - getBotAz_FieldRelative();
            return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapAutonConstants.kEpsilon),
                Constants.SnapAutonConstants.kTimeout);
        }
        else{
            double error = snapPIDController.getGoal().position - getBotAz_FieldRelative();
            return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon),
                    Constants.SnapConstants.kTimeout);
        }        
    }

    public void maybeStopSnap(boolean force) {
        if (!isSnapping) {
            return;
        }
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(getBotAz_FieldRelative());
        }
    }

    public boolean isSnapping() {
        return this.isSnapping;
    }

    public void setDriveOrientation(DriveOrientation orientation) {
        driveOrientation = orientation;
    }

    public DriveOrientation getDriveOrientation() {
        return driveOrientation;
    }
    
    // private void startSimThread() {
    //     m_lastSimTime = Utils.getCurrentTimeSeconds();

    //     /* Run simulation at a faster rate so PID gains behave more reasonably */
    //     m_simNotifier = new Notifier(() -> {
    //         final double currentTime = Utils.getCurrentTimeSeconds();
    //         double deltaTime = currentTime - m_lastSimTime;
    //         m_lastSimTime = currentTime;

    //         /* use the measured time delta, get battery voltage from WPILib */
    //         updateSimState(deltaTime, RobotController.getBatteryVoltage());
    //     });
    //     m_simNotifier.startPeriodic(kSimLoopPeriod);
    // }

    //Odometry re-seeding with limelight stuff
    // private boolean odometryBotPosUpdaterMethodFlag = false;
    // private boolean odometryBotPosUpdater() {
    //     if (odometryBotPosUpdaterMethodFlag) {
    //         frontCamera.updateBotPos();
    //         if (frontCamera.getBotPosX() != 0 && frontCamera.getBotPosY() != 0) {
    //             seedFieldRelative(new Pose2d(new Translation2d(frontCamera.getBotPosX(), frontCamera.getBotPosY()),
    //                     new Rotation2d(getPose().getRotation().getRadians())));
    //             return true;
    //         }
    //     }
    //     return false;
    // }
    // public int periodicCounter = 0;

    // odometryTrack():
    // Uses the Odometries X and Y position, relative to the target, and drives the
    // yaw of the drivetrain to face the target.
    // odometryTrack() needs:
    // Targeting.setTarget(Target.#########); needs to be ran prior, this sets the
    // targetPos that the drivetrain will orient to.
    // private Targeting odometryTargeting = new Targeting(true); must be added.
    // PID Controller "aimAtTargetController" must be created
    // public void odometryTrack() {
    //     drivetrain_state = "ODOMETRYTRACK";
    //     Double offset = getBotAz_FieldRelative() - odometryTargeting.getAz();
    //     offset = rolloverConversion_radians(offset);
    //     Double request = aimAtTargetController.calculate(offset, 0.005) * Constants.MaxAngularRate;

    //     SmartDashboard.putNumber("PID Output:", request/Constants.MaxAngularRate);
    //     SmartDashboard.putNumber("PID Error:", offset);

    //     ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
    //             getDriveX() * Constants.MaxSpeed,
    //             getDriveY() * Constants.MaxSpeed,
    //             request,
    //             m_odometry.getEstimatedPosition()
    //                     .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()),
    //             0.005);

    //     var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

    //     for (int i = 0; i < this.Modules.length; i++) {
    //         this.Modules[i].apply(states[i],
    //                 SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
    //     }
    // }

    // aprilTagTrack():
    // Uses the limelights TX value for a targeted ID (horizontal centering of the
    // april tag in the limelights video feed),
    // ... and controls the Yaw of the robot keep the drivetrain facing the target,
    // with the targeted april tag centered.
    // ... if the april tag cannot be seen, joystickDrive() is enabled.
    // aprilTagTrack() needs:
    // Targeting.setTarget(Target.#########); needs to be ran prior, this sets which
    // april tag ID to lock onto.
    // A limelight with the host name 'limelight-front' must be connected to the
    // RIO's network
    // The limelight must be mounted facing in the direction you would like the
    // drivetrain to aim at the target
    // PID Controller "aimAtTargetController" must be created
    // public void aprilTagTrack() {
    //     // Get JSON Dump from Limelight-front
    //     LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

    //     // Go through limelight JSON dump, and look for Target ID
    //     // If ID found, save TX value to offset for targeting.
    //     boolean canSeeTarget = false;
    //     double offset = 0;
    //     for (var aprilTagResults : llresults.targetingResults.targets_Fiducials) {
    //         if (aprilTagResults.fiducialID == Targeting.getTargetID()) {
    //             offset = -(Math.toRadians(aprilTagResults.tx));
    //             canSeeTarget = true;
    //         }
    //     }

    //     if (canSeeTarget) {
    //         drivetrain_state = "APRILTAGTRACK";
    //         Double request = aimAtTargetController.calculate(offset, 0.005) * Constants.MaxAngularRate;
    //         ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
    //                 getDriveX() * Constants.MaxSpeed,
    //                 getDriveY() * Constants.MaxSpeed,
    //                 request,
    //                 m_odometry.getEstimatedPosition()
    //                         .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()),
    //                 0.005);
    //         var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());
    //         for (int i = 0; i < this.Modules.length; i++) {
    //             this.Modules[i].apply(states[i],
    //                     SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
    //         }
    //     } else {
    //         drivetrain_state = "JOYSTICK";
    //         joystickDrive();
    //     }
    // }


    //TODO This is not working
    // public void joystickDrive_RobotRelative_OpenLoop(double rotation) { 

    //     ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromRobotRelativeSpeeds(
    //             getDriveX() * Constants.MaxSpeed,
    //             getDriveY() * Constants.MaxSpeed,
    //             rotation * Constants.MaxAngularRate,
    //             m_odometry.getEstimatedPosition()
    //                     .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()),
    //             0.2);

    //     var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

    //     for (int i = 0; i < this.Modules.length; i++) {
    //         this.Modules[i].apply(states[i],
    //                 SwerveModule.DriveRequestType.OpenLoopVoltage,
    //                 SwerveModule.SteerRequestType.MotionMagic);
    //     }
    // }

    // private void rotationHold() {
    //     Double offset = getBotAz_FieldRelative() - pathFollower.lastAngle().getRadians();
    //     offset = rolloverConversion_radians(offset);
    //     Double request = aimAtTargetController.calculate(offset, 0.005) * Constants.MaxAngularRate;
    //     SmartDashboard.putNumber("PID Output:", request / Constants.MaxAngularRate);
    //     SmartDashboard.putNumber("PID Error:", offset);

    //     ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
    //             0.0,
    //             0.0,
    //             request,
    //             m_odometry.getEstimatedPosition()
    //                     .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()),
    //             0.005);

    //     var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

    //     for (int i = 0; i < this.Modules.length; i++) {
    //         this.Modules[i].apply(states[i],
    //                 SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
    //     }
    // }



   // public void joystickDrive_RobotRelative() {
    //     double rotation = getDriveRotation();
    //     if (isSnapping) {
    //         if (Math.abs(getDriveRotation()) == 0.0) {
    //             maybeStopSnap(false);
    //             rotation = -calculateSnapValue();
    //         } else {
    //             maybeStopSnap(true);
    //         }
    //     } else {
    //         if (rotation == 0) {
    //             drivetrain_state = "JOYSTICKDRIVE_PID";
    //             double offset = -(getBotAz_FieldRelative() - joystickDrive_holdAngle);
    //             rotation = joystickController.calculate(offset, 0.005) * Constants.MaxAngularRate;
    //             SmartDashboard.putNumber("PID Error:", offset);
    //             SmartDashboard.putNumber("PID Output:", rotation);
    //         } else {
    //             drivetrain_state = "JOYSTICKDRIVE_OPENLOOP";
    //             joystickDrive_holdAngle = getBotAz_FieldRelative();
    //         }
    //     }
    //     joystickDrive_RobotRelative_OpenLoop(rotation);
    // }


}   //#endregion

