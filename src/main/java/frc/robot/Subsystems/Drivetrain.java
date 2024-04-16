package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.UpdateManager;
import frc.robot.util.Camera.Limelight;
import frc.robot.util.Camera.LimelightHelpers;
import frc.robot.util.Camera.LimelightHelpers.PoseEstimate;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.TargetSimple;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Control.PidConstants;
import frc.robot.util.Control.PidController;
import frc.robot.util.Control.TimeDelayedBoolean;
import frc.robot.util.Interpolable.InterpolatingDouble;
import frc.robot.util.PathFollowing.FollowPathCommand;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem, UpdateManager.Updatable {
    private DriveMode mControlMode = DriveMode.JOYSTICK;
    private SideMode sideMode = SideMode.RED;
    private String drivetrain_state = "INIT";

    private PidController holdAngleController = new PidController(new PidConstants(0.5, 0.0, 0.00));
    private PidController aimAtTargetController = new PidController(new PidConstants(0.7, 0.0, 0.02));
    private PidController noteTrackController = new PidController(new PidConstants(0.5, 0.001, 0.02));  // 1.0 strafe, 0.4 rotate
    private PidController joystickController = new PidController(new PidConstants(1.0, 0, 0.0));
    private PidController strafeTxController = new PidController(new PidConstants(1.0, 0.00, 0.02));
    private PidController strafeRotateController = new PidController(new PidConstants(1.0, 0.00, 0.02));

    private final SwerveDrivePoseEstimator targetingOdo;	

    private Limelight limelight = new Limelight("front");
    private Limelight noteLimelight = new Limelight("note");
    private Targeting frontCamera = new Targeting("front", false);
    private Targeting odometryTargeting = new Targeting(true);

    private int pidVisionUpdateCounter = 0;
    private final int VISON_COUNTER_MAX = 4;
    private int pidNoteUpdateCounter = 0;
    private final int NOTE_COUNTER_MAX = 4;
    
    public boolean isTrackingNote = false;
    private boolean isHoldingAngle = false;
    private double joystickDriveHoldAngleRadians = 0;
    private double passFieldRelativeAngleDeg = 0;
    protected Rotation2d m_blueFieldRelativeOffset = new Rotation2d();

    private FollowPathCommand pathFollower;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final double STICK_DEADBAND = 0.1;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    
    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle driveFieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.MaxSpeed * STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentric driveFieldCentricNoDeadband = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.MaxSpeed * STICK_DEADBAND) 
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.RobotCentric driveRobotCentricNoDeadband = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    public int updateCounter = 0;
    public boolean firstTimeInAimAtTarget = true;
    public boolean hasPreviouslyLockedOnTarget = false;
    public int timeout_counter = 0;

    private boolean isSnapping;
    private double mLimelightVisionAlignGoal;
    // private double mGoalTrackVisionAlignGoal;
    // private double mVisionAlignAdjustment;

    private ProfiledPIDController snapPIDController;
    private ProfiledPIDController snapPIDControllerAuton;
    private PIDController visionPIDController;

    public enum DriveOrientation {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
    private DriveOrientation driveOrientation = DriveOrientation.FIELD_CENTRIC;

    // private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    // private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        this.targetingOdo = 
            new SwerveDrivePoseEstimator(
                m_kinematics, new Rotation2d(), m_modulePositions, new Pose2d(), 
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, 0.9));

        holdAngleController.setContinuous(true);
        holdAngleController.setOutputRange(-1.0, 1.0);
 
        aimAtTargetController.setContinuous(true);
        aimAtTargetController.setInputRange(-Math.PI,  Math.PI);
        aimAtTargetController.setOutputRange(-1.0, 1.0);

        noteTrackController.setContinuous(true);
        noteTrackController.setInputRange(-Math.PI, Math.PI);
        noteTrackController.setOutputRange(-1.0, 1.0);

        strafeRotateController.setContinuous(true);
        strafeRotateController.setInputRange(-Math.PI, Math.PI);
        strafeRotateController.setOutputRange(-1.0, 1.0);

        // strafeTxController.setContinuous(true);
        // strafeTxController.setInputRange(-Math.PI, Math.PI);
        strafeTxController.setOutputRange(-1.0, 1.0);

        joystickController.setContinuous(true);
        joystickController.setInputRange(-Math.PI, Math.PI);
        joystickController.setOutputRange(-1.0, 1.0);
        joystickController.setSetpoint(0.0);

        driveFieldCentricFacingAngle.HeadingController.setPID(10.0, 0, 0);
        driveFieldCentricFacingAngle.ForwardReference = ForwardReference.RedAlliance;

        // Targeting.setTargetSimple(TargetSimple.SPEAKER);

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

        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = TunerConstants.kStatorCurrentA;
        currentConfigs.StatorCurrentLimitEnable = true;
        currentConfigs.SupplyCurrentLimit = TunerConstants.kSupplyCurrentA;
        currentConfigs.SupplyCurrentLimitEnable = true;

        // Torque current limits are applied in SwerveModule with kSlipCurrent.

        ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs();
        rampConfigs.VoltageClosedLoopRampPeriod = TunerConstants.kVelocityClosedLoopRampPeriod;
        for (int i = 0; i < this.Modules.length; i++) {
            StatusCode response = this.Modules[i].getDriveMotor().getConfigurator().apply(rampConfigs);
            if (!response.isOK()) {
                System.out.println("TalonFX ID " + this.Modules[i].getDriveMotor().getDeviceID() + " failed config ramp configs with error " + response.toString());
            }
            response = this.Modules[i].getDriveMotor().getConfigurator().apply(currentConfigs);
            if (!response.isOK()) {
                System.out.println("TalonFX ID " + this.Modules[i].getDriveMotor().getDeviceID() + " failed config current configs with error " + response.toString());
            }
        }

        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        ReplanningConfig replan = new ReplanningConfig();

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                new PIDConstants(10, 0, 0),
                new PIDConstants(5.0, 0, 0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                //TODO where to change the error thresholds, defualts are 1.0 dynamic and 0.25 spike in meters
                //@freddytums
                new ReplanningConfig(true, true, 1.0, 0.25)
            );

        //TODO try changing this to the targeting odometry to fix
        //the reseeding freakout @freddytums
        pathFollower = new FollowPathCommand(
                () -> this.getPose(), //getTargetingOdoPose()
                this::getCurrentRobotChassisSpeeds,
                new PPHolonomicDriveController(
                        config.translationConstants, config.rotationConstants, config.period, config.maxModuleSpeed,
                        config.driveBaseRadius),
                config.replanningConfig,
                () -> false);

        configurePathPlanner();
    }

    public void setTeleopCurrentLimits(){
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = TunerConstants.kTeleStatorCurrentA;
        currentConfigs.StatorCurrentLimitEnable = true;
        currentConfigs.SupplyCurrentLimit = TunerConstants.kTeleSupplyCurrentA;
        currentConfigs.SupplyCurrentLimitEnable = true;

        // Torque current limits are applied in SwerveModule with kSlipCurrent.
        for (int i = 0; i < this.Modules.length; i++) {
            StatusCode response = this.Modules[i].getDriveMotor().getConfigurator().apply(currentConfigs);
            if (!response.isOK()) {
                System.out.println("TalonFX ID " + this.Modules[i].getDriveMotor().getDeviceID() + " failed config ramp configs with error " + response.toString());
            }
        }
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

    public boolean odosnap = false;
    public void setDriveMode(DriveMode mode) {
        // odosnap = false;
        if (mode == mControlMode){
            return;
        }

        holdAngleController.integralAccum = 0;
        aimAtTargetController.integralAccum = 0;
        noteTrackController.integralAccum = 0;
        joystickController.integralAccum = 0;
        isHoldingAngle = false;
        joystickDriveHoldAngleRadians = getGyroAngleRadians();

        firstTimeInAimAtTarget = true;
        hasPreviouslyLockedOnTarget = false;
        // noteLimelight.setLedMode(LedMode.OFF);
        isTrackingNote = false;

        // Runs once on mode change to JOYSTICK, to set the current field-relative yaw
        // of the robot to the hold angle.
        if (mode == DriveMode.JOYSTICK) {
            maybeStopSnap(true);
        } else if (mode == DriveMode.AIM_AT_NOTE) {
            pidNoteUpdateCounter = NOTE_COUNTER_MAX + 1;
            // noteLimelight.setLedMode(LedMode.ON);
        } else if (mode == DriveMode.AIM_AT_NOTE) {
             isTrackingNote = false;
        } else if (mode == DriveMode.AIMATTARGET) {
            Targeting.setTargetSimple(Targeting.getTargetSimple());
            if (Targeting.getTargetSimple() == TargetSimple.CENTERPASS) {
                    drivetrain_state = "CENTER SNAP";
                    passFieldRelativeAngleDeg = getSideMode() == SideMode.RED ? -Constants.CENTER_FIELD_RELATIVE_ANGLE_DEG : Constants.CENTER_FIELD_RELATIVE_ANGLE_DEG;
                    startSnap(passFieldRelativeAngleDeg);
            } else if (Targeting.getTargetSimple() == TargetSimple.CORNERPASS) {
                    drivetrain_state = "CORNER SNAP";
                    passFieldRelativeAngleDeg = getSideMode() == SideMode.RED ? -Constants.CORNER_FIELD_RELATIVE_ANGLE_DEG : Constants.CORNER_FIELD_RELATIVE_ANGLE_DEG;
                    startSnap(passFieldRelativeAngleDeg);
            } else {
                // pidVisionUpdateCounter = VISON_COUNTER_MAX + 1;
                boolean canSeeTarget = false;
                double offset = 0;
                double distance_XY_Average = odometryTargeting.getDistance_XY_average();
                // double aimOffset = getSideMode()==SideMode.BLUE?0.0:Constants.kAutoAimOffset.getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
                double aimOffset = Constants.kAutoAimOffset.getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
                aimOffset+= yawOffset;
                //TODO is this the right spot? @freddytums
                PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
                for (int i = 0; i < botPoseEstimate.rawFiducials.length; i++) {
                    if (botPoseEstimate.rawFiducials[i].id == Targeting.getTargetID()) {
                        offset = -(Math.toRadians(botPoseEstimate.rawFiducials[i].txnc + aimOffset));
                        // SmartDashboard.putNumber("TX Limelight", botPoseEstimate.rawFiducials[i].txnc);
                        canSeeTarget = true;
                        break;
                    }
                }

                if (canSeeTarget) {
                    if (Math.abs(Math.toDegrees(offset)) > 3.0) {
                        drivetrain_state = "LIME SNAP";
                        // SmartDashboard.putNumber("Snap offset", Math.toDegrees(offset));
                        // SmartDashboard.putNumber("Snap gyro", Math.toDegrees(getBotAz_FieldRelative()));
                        // SmartDashboard.putNumber("Snap gyro modulus", Math.toDegrees(MathUtil.inputModulus(getBotAz_FieldRelative(), -Math.PI,  Math.PI)));
                        // SmartDashboard.putNumber("Snap input", Math.toDegrees(getBotAz_FieldRelative() - offset));
                        // SmartDashboard.putNumber("Snap input modulus", Math.toDegrees(MathUtil.inputModulus(getBotAz_FieldRelative() - offset, -Math.PI,  Math.PI)));
                        startSnap(Math.toDegrees(getBotAz_FieldRelative() - offset));
                    }
                } 
                // else {
                //     drivetrain_state = "ODO SNAP";
                //     SmartDashboard.putNumber("ODO Snap Angle", Math.toDegrees(odometryTargeting.getAz() + Math.PI) - aimOffset);
                //     startSnap(Math.toDegrees(odometryTargeting.getAz() + Math.PI) - aimOffset);
                // }
            }
        }

        mControlMode = mode;
    }

    public void snapToAngleAuton(double angle){
        if (sideMode == SideMode.RED) {
            angle = rolloverConversion_radians(angle + (Math.PI / 2));
        } else if (getSideMode() == SideMode.BLUE) {
            angle = rolloverConversion_radians(angle - (Math.PI / 2));
        }
        setDriveMode(DriveMode.AIMATTARGET_AUTON);
        if(Constants.debug){
            SmartDashboard.putNumber("snap auton angle", Math.toDegrees(rolloverConversion_radians(angle+Math.PI)));
        }
        startSnap(Math.toDegrees(rolloverConversion_radians(angle+Math.PI)));
    }

    //#region DriveTrain methods

    public void updateDrive(double rotation) {
        if (driveOrientation == DriveOrientation.FIELD_CENTRIC) {
            if(soFine){
                this.applyRequest(() -> driveFieldCentricNoDeadband
                .withVelocityX(MathUtil.applyDeadband(getDriveXWithoutDeadband(), 0.05, 1.0) * Constants.MaxSpeed) 
                .withVelocityY(MathUtil.applyDeadband(getDriveYWithoutDeadband(), 0.05, 1.0) * Constants.MaxSpeed) 
                .withRotationalRate(MathUtil.applyDeadband(rotation, 0.05, 1.0) * Constants.MaxAngularRate)
            );
            } else {  
                this.applyRequest(() -> driveFieldCentric
                    .withVelocityX(getDriveXWithoutDeadband() * Constants.MaxSpeed) 
                    .withVelocityY(getDriveYWithoutDeadband() * Constants.MaxSpeed) 
                    .withRotationalRate(rotation * Constants.MaxAngularRate)
                );
            }
        }
        else {
            if(soFine){
                this.applyRequest(() -> driveRobotCentricNoDeadband
                    .withVelocityX(MathUtil.applyDeadband(getDriveXWithoutDeadband(), 0.05, 1.0) * Constants.MaxSpeed) 
                    .withVelocityY(MathUtil.applyDeadband(getDriveYWithoutDeadband(), 0.05, 1.0) * Constants.MaxSpeed) 
                    .withRotationalRate(MathUtil.applyDeadband(rotation, 0.05, 1.0) * Constants.MaxAngularRate)
                );
            }else{
                this.applyRequest(() -> driveRobotCentric
                    .withVelocityX(getDriveXWithoutDeadband() * Constants.MaxSpeed) 
                    .withVelocityY(getDriveYWithoutDeadband() * Constants.MaxSpeed) 
                    .withRotationalRate(rotation * Constants.MaxAngularRate) 
                );
            }
        }
    }

    public void updateDrive(double x, double y, double rotation) {
        if (driveOrientation == DriveOrientation.FIELD_CENTRIC) {        
            this.applyRequest(() -> driveFieldCentric
                .withVelocityX(x * Constants.MaxSpeed) 
                .withVelocityY(y * Constants.MaxSpeed) 
                .withRotationalRate(rotation * Constants.MaxAngularRate) 
            );
        }
        else {
            this.applyRequest(() -> driveRobotCentric
                .withVelocityX(x * Constants.MaxSpeed) 
                .withVelocityY(y * Constants.MaxSpeed) 
                .withRotationalRate(rotation * Constants.MaxAngularRate) 
            );
        }
    }

    public void updateDriveFacingAngle(double targetAngleRadians) {
        this.applyRequest(() -> driveFieldCentricFacingAngle
            .withVelocityX(getDriveXWithoutDeadband() * Constants.MaxSpeed) 
            .withVelocityY(getDriveYWithoutDeadband() * Constants.MaxSpeed) 
            .withTargetDirection(Rotation2d.fromRadians(targetAngleRadians))
        );
    }

    public void updateDriveStrafe(double yInput) {
        if (driveOrientation == DriveOrientation.FIELD_CENTRIC) {        
            this.applyRequest(() -> driveFieldCentricNoDeadband
                .withVelocityX(getDriveXWithDeadband() * Constants.MaxSpeed) 
                .withVelocityY(yInput * Constants.MaxSpeed) 
                .withRotationalRate(getDriveRotationWithDeadband() * Constants.MaxAngularRate) 
            );
        }
        else {
            this.applyRequest(() -> driveRobotCentricNoDeadband
                .withVelocityX(getDriveXWithDeadband() * Constants.MaxSpeed) 
                .withVelocityY(yInput * Constants.MaxSpeed) 
                .withRotationalRate(getDriveRotationWithDeadband() * Constants.MaxAngularRate) 
            );
        }
    }

    public void updateDriveStrafeRotate(double yInput, double rotationInput){
        if (driveOrientation == DriveOrientation.FIELD_CENTRIC) {        
            this.applyRequest(() -> driveFieldCentric
                .withVelocityX(getDriveXWithDeadband() * Constants.MaxSpeed) 
                .withVelocityY(yInput * Constants.MaxSpeed) 
                .withRotationalRate(rotationInput * Constants.MaxAngularRate) 
            );
        }
        else {
            this.applyRequest(() -> driveRobotCentric
                .withVelocityX(getDriveXWithDeadband() * Constants.MaxSpeed) 
                .withVelocityY(yInput * Constants.MaxSpeed) 
                .withRotationalRate(rotationInput * Constants.MaxAngularRate) 
            );
        }
    }

    public void joystickDrive() {
        setDriveOrientation(DriveOrientation.FIELD_CENTRIC);
        double rotation = getDriveRotationWithDeadband();
        if (isSnapping) {
            // if (Math.abs(getDriveRotation()) == 0.0) {
                maybeStopSnap(false);
                rotation = -calculateSnapValue();
                isHoldingAngle = false;
            // } else {
            //     maybeStopSnap(true);
            // }
        } 
        else {   
            if ((Math.abs(rotation) < 0.001 && Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 1.0 && isHoldingAngle == false) ||
                (Math.abs(rotation) < 0.001 && isHoldingAngle == true)) {
                drivetrain_state = "JOYSTICKDRIVE_HOLD_ANGLE";
                if (isHoldingAngle == false) {
                    joystickDriveHoldAngleRadians = getGyroAngleRadians();
                    holdAngleController.setSetpoint(joystickDriveHoldAngleRadians);
                    isHoldingAngle = true;
                }
                // updateDriveFacingAngle(joystickDriveHoldAngleRadians);
                // return;
                rotation = holdAngleController.calculate(getGyroAngleRadians(), 0.005);
                // SmartDashboard.putNumber("Hold Error", Math.toDegrees(getGyroAngleRadians() - joystickDriveHoldAngleRadians));
                // SmartDashboard.putNumber("Hold Output", rotation);
            }
            else {
                drivetrain_state = "JOYSTICKDRIVE_OPENLOOP";
                isHoldingAngle = false;
            }
        }

        updateDrive(rotation);
    }

    private double getGyroAngleRadians() {
//        return m_odometry.getEstimatedPosition().relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation().getRadians();
//        return m_odometry.getEstimatedPosition().getRotation().getRadians();
        return getPigeon2().getRotation2d().getRadians();
    }

    private Pose2d getGyroAngle() {
//        return m_odometry.getEstimatedPosition().relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation().getRadians();
//        return m_odometry.getEstimatedPosition().getRotation().getRadians();
        return new Pose2d(0,0,getPigeon2().getRotation2d());
    }

    private final double ROTATE_THRESH = 0.01;
    private final double TX_THRESH = 0.1;
    public void strafeToAprilTag() {
        if(frontCamera.hasTarget()){
            double rotate_targetOffset = Math.toRadians(limelight.getTargetHorizOffset());
            double[] tp_rs = limelight.getTable().getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            double tx_targetOffset = tp_rs[0];
            drivetrain_state = "STRAFE2APRILTAG";

            double tx_pidOutput = strafeTxController.calculate(tx_targetOffset, 0.005);
            double rotate_pidOutput = strafeRotateController.calculate(rotate_targetOffset, 0.005);
            
            if(Constants.debug){
                SmartDashboard.putNumber("ROT PID Error:", rotate_targetOffset);
                SmartDashboard.putNumber("ROT PID Output:", rotate_pidOutput);

                SmartDashboard.putNumber("TX PID Error:", tx_targetOffset);
                SmartDashboard.putNumber("TX PID Output:", tx_pidOutput);
            }

            setDriveOrientation(DriveOrientation.ROBOT_CENTRIC);
            updateDriveStrafeRotate(tx_pidOutput, rotate_pidOutput);   
            
            if(Math.abs(rotate_targetOffset) < ROTATE_THRESH && Math.abs(tx_targetOffset) < TX_THRESH){
                setDriveOrientation(DriveOrientation.FIELD_CENTRIC);
                setDriveMode(DriveMode.JOYSTICK);
            }
        }
        else {
            setDriveOrientation(DriveOrientation.FIELD_CENTRIC);
            joystickDrive();
        }
    }

    private double yawOffset = 0.0;

    public void increaseYawoffset(){
        yawOffset += 0.50;
    }

    public void decreaseYawoffset(){
        yawOffset -= 0.50;
    }

    public void resetYawoffset(){
        yawOffset = 0.0;
    }

    public void aimAtTarget() {
        if (isSnapping) {
            maybeStopSnap(false);
            updateDrive(-calculateSnapValue());
        } else if (Targeting.getTargetSimple() == TargetSimple.CENTERPASS || Targeting.getTargetSimple() == TargetSimple.CORNERPASS) {
            drivetrain_state = "CENTER-CORNER MODE";
            aimAtTargetController.setSetpoint(Math.toRadians(passFieldRelativeAngleDeg));
            double rotation = aimAtTargetController.calculate(getBotAz_FieldRelative(), 0.005);
            updateDrive(rotation);
        } else {
            boolean canSeeTarget = false;
            double offset = 0;
            double distance_XY_Average = odometryTargeting.getDistance_XY_average();

            PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
            for (int i = 0; i < botPoseEstimate.rawFiducials.length; i++) {
                if (botPoseEstimate.rawFiducials[i].id == Targeting.getTargetID()) {
                    distance_XY_Average = odometryTargeting.getDistance_XY_average();
                    double aimOffset = Constants.kAutoAimOffset.getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
                    aimOffset = Constants.kAutoAimOffset.getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
                    //TODO is this the right spot? @freddytums
                    aimOffset += yawOffset;
                    offset = (Math.toRadians(botPoseEstimate.rawFiducials[i].txnc + aimOffset));
                    // SmartDashboard.putNumber("TX Limelight", botPoseEstimate.rawFiducials[i].txnc);
                    canSeeTarget = true;
                    break;
                }
            }

            if (firstTimeInAimAtTarget) { // When changing modes, clear integral accumulation
                aimAtTargetController.integralAccum = 0;
                firstTimeInAimAtTarget = false;
            }

            double currentAngleRadians =  getBotAz_FieldRelative();
            double alignAngleRadians = MathUtil.inputModulus(currentAngleRadians - offset, -Math.PI,  Math.PI);
           
            if (canSeeTarget) {
                // if (Math.abs(Math.toDegrees(offset)) > 3.0) {
                //     drivetrain_state = "LIME SNAP";
                //     startSnap(Math.toDegrees(getBotAz_FieldRelative() - offset));
                // } else {
                    drivetrain_state = "LIME MODE";
                    aimAtTargetController.setSetpoint(alignAngleRadians);
                    // SmartDashboard.putNumber("Align setpoint", Math.toDegrees(alignAngleRadians));
                    hasPreviouslyLockedOnTarget = true;
                    double rotation = aimAtTargetController.calculate(currentAngleRadians, 0.005);
                    // SmartDashboard.putNumber("Align current angle", Math.toDegrees(currentAngleRadians));
                    // SmartDashboard.putNumber("Align rotation", rotation);
                    updateDrive(rotation);
                // }
            }
            else {
                if (!hasPreviouslyLockedOnTarget) {
                    isHoldingAngle = false;
                    joystickDrive();
                }
                else {
                    drivetrain_state = "LIME MODE NO TARGET";
                    double rotation = aimAtTargetController.calculate(currentAngleRadians, 0.005);
                    updateDrive(rotation);
                }

                // // If you have already locked onto a target don't go back to odo mode
                // if (Targeting.getTargetSimple() == TargetSimple.SPEAKER || hasPreviouslyLockedOnTarget) {
                //     isHoldingAngle = false;
                //     joystickDrive();
                //     return;
                // }

                // drivetrain_state = "ODO MODE";

                // double currentAngleRadians = getBotAz_FieldRelative();
                // double angleCalc = odometryTargeting.getAz() + Math.PI - Math.toRadians(aimOffset);
                // double alignAngleRadians = MathUtil.inputModulus(angleCalc, -Math.PI,  Math.PI);
    
                // aimAtTargetController.setSetpoint(alignAngleRadians);

                // double rotation = aimAtTargetController.calculate(currentAngleRadians, 0.005);
     
                // // SmartDashboard.putNumber("ODO Angle:", Math.toDegrees(angleCalc));
                // // SmartDashboard.putNumber("ODO Angle with Modulus:", Math.toDegrees(alignAngleRadians));

                // // SmartDashboard.putNumber("PID Output:", rotation);
                // // SmartDashboard.putNumber("PID Error:", offset);

                // updateDrive(-rotation);
            }
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
        Targeting.setTargetSimple(Targeting.getTargetSimple());
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
        if (noteLimelight.hasTarget() /*&& Math.abs(noteLimelight.getTargetHorizOffset()) < 27.0*/) {
            drivetrain_state = "NOTE MODE";
            double targetOffset = Math.toRadians(noteLimelight.getTargetHorizOffset());

            double currentNoteTrackAngle = getBotAz_FieldRelative();
            double adjustAngle = MathUtil.inputModulus(currentNoteTrackAngle - targetOffset, -Math.PI, Math.PI);

        //    if (pidNoteUpdateCounter > NOTE_COUNTER_MAX) {
            noteTrackController.setSetpoint(adjustAngle);
            pidNoteUpdateCounter = 0;
        //    }
        //    pidNoteUpdateCounter++;
            double pidRotationOutput = noteTrackController.calculate(currentNoteTrackAngle, 0.005);

            if(Constants.debug){
                SmartDashboard.putNumber("pidRotationOutput", pidRotationOutput);
                SmartDashboard.putNumber("noteAdjustAngle", adjustAngle);
            }
            double xForward = Math.sqrt(getDriveXWithDeadband() * getDriveXWithDeadband() + getDriveYWithDeadband() * getDriveYWithDeadband());
             
            isTrackingNote = true;
            setDriveOrientation(DriveOrientation.ROBOT_CENTRIC);
            updateDrive(xForward, 0.0, pidRotationOutput);    
        }
        else {
            if (isTrackingNote) {
                drivetrain_state = "NOTE MODE";
                // double pidRotationOutput = noteTrackController.calculate(getBotAz_FieldRelative(), 0.005);
                double xForward = Math.sqrt(getDriveXWithDeadband() * getDriveXWithDeadband() + getDriveYWithDeadband() * getDriveYWithDeadband());
                setDriveOrientation(DriveOrientation.ROBOT_CENTRIC);
                updateDrive(xForward, 0.0, 0.0);         
            }
            else {
                isTrackingNote = false;
                joystickDrive();
            }
        }
    }

    private double lastCommandedSpeed = 0.0;
    private boolean setTrackSpeed = false;

    //timer approach
    private double StopTime = 1.0;
    private double speed = 0.0;
    private double slowAccel = 0.0;
    private boolean setSlowAccel = false;

    private boolean towardsEnd = false;

    //stopping distance
    private double stopDist = 0.0;
    private boolean setStopDist = false;
    //just use path max accel?
    private double MAX_ACCEL = 5.5;

    private void autonDrive(){
        ChassisSpeeds speeds = pathFollower.update();

        if(Constants.debug){
            SmartDashboard.putBoolean("is tracking note", isTrackingNote);
        }

        //TODO test new ways of tracking note speed @freddytums
        //ie maybe command overall speed of path so we slow down towards end
        //however we will have to see how the replanning, if we get off (probably), affects that
        //if it does we could just try having a flag that once we are towards the end we tell it don't
        //replan the path anymore so we can just get speeds idk
        //or we could try using the path timer to "choose" when to slow down
        //or use the path end pose to calculate a stopping distance from the last commanded speed
        //that once we are withing that stopping distance we slow down like a mini motion profile
        //we could even like let it get within that distance then let the path replan and let it
        //do the motion profile its self idk whatever works best and is the simplest given our time
        //ultimately this is all to solve the issue of drive over the center line when we tracked
        //we could all solve this with command timing but I think doing it here will be more
        //applicable to all situations as I'd likely try tuning the commands for the center line
        //then when we go for the closer ones it'll be different

        //timer approach
        /*
        if(pathFollower.getPathTime()-pathFollower.getPathTimer()<StopTime){
            //method 1: using path speed from that point on
            speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            //method 2: making our own motion profile
            //from V²=V₀²+2aΔx we can solve for the acceleration needed
                //submethod 1: only updating the acceleration once
                if(!setSlowAccel){
                    double distance = 
                        pathFollower.getPathEnd().getTranslation().getDistance(
                            targetingOdo.getEstimatedPosition().getTranslation()
                            // m_odometry.getEstimatedPosition().getTranslation()
                        );
                    slowAccel = (-(lastCommandedSpeed*lastCommandedSpeed))/(2.0*distance);
                    setSlowAccel = true;
                }
                //submethod 2: updating constantly
                double distance = 
                        pathFollower.getPathEnd().getTranslation().getDistance(
                            targetingOdo.getEstimatedPosition().getTranslation()
                            // m_odometry.getEstimatedPosition().getTranslation()
                        );
                double curSpeed = Math.hypot(getCurrentRobotChassisSpeeds().vxMetersPerSecond, getCurrentRobotChassisSpeeds().vyMetersPerSecond);
                slowAccel = (-Math.pow(distance, distance))/(2.0*distance);
            speed -= slowAccel;

            //method 3: don't replan the path anymore
            //this will get used in FollowPathCommand to cancel replanning
            pathFollower.setReplanning(false); 
            then use path speed
            speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }else{
            setSlowAccel = false;
            speed = lastCommandedSpeed;
            pathFollower.setReplanning(true); 
        }
        */

        //stopping distance
        /*
        if(setTrackSpeed){
            stopDist = (lastCommandedSpeed*lastCommandedSpeed)/(2.0*MAX_ACCEL);
            setStopDist = true;
        }else{
            setStopDist = false;
        }
        if(setStopDist){
            double distance = 
                            pathFollower.getPathEnd().getTranslation().getDistance(
                                targetingOdo.getEstimatedPosition().getTranslation()
                                // m_odometry.getEstimatedPosition().getTranslation()
                            );
            if(distance<=stopDist){
                speed = 0.0;
            }
        }
        */

        //apply these speeds, from any method
        /*
        applyRequest(()->driveRobotCentricNoDeadband
                .withVelocityX(speed)
                .withVelocityY(0.0)
                .withRotationalRate(pidRotationOutput*Constants.MaxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity)
            );
        */


        if(isTrackingNote && noteLimelight.hasTarget()){
            if(!setTrackSpeed){
                lastCommandedSpeed = speeds.vxMetersPerSecond;
                setTrackSpeed = true;
            }
            System.out.println("TRACKING!!!!");
            double targetOffset = Math.toRadians(noteLimelight.getTargetHorizOffset());

            double currentNoteTrackAngle = getBotAz_FieldRelative();
            double adjustAngle = MathUtil.inputModulus(currentNoteTrackAngle - targetOffset, -Math.PI, Math.PI);

        //    if (pidNoteUpdateCounter > NOTE_COUNTER_MAX) {
            noteTrackController.setSetpoint(adjustAngle);

            double pidRotationOutput = noteTrackController.calculate(currentNoteTrackAngle, 0.005);
            if(Constants.debug){
                SmartDashboard.putNumber("PID Output Note", pidRotationOutput);
            }
            applyRequest(()->driveRobotCentricNoDeadband
                .withVelocityX(lastCommandedSpeed)
                .withVelocityY(0.0)
                .withRotationalRate(pidRotationOutput*Constants.MaxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity)
            );
            return;
        }

        if (!pathDone()) {
            setTrackSpeed = false;
            // var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());
            // for (int i = 0; i < this.Modules.length; i++) {
            //     this.Modules[i].apply(states[i],
            //             SwerveModule.DriveRequestType.OpenLoopVoltage,
            //             SwerveModule.SteerRequestType.MotionMagic);
            // }
            // SmartDashboard.putBoolean("path done", false);
            applyRequest(()->driveRobotCentricNoDeadband
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity)
            );
        } else {
            // rotationHold();
            // SmartDashboard.putBoolean("path done", true);
            applyRequest(()->new SwerveDriveBrake());
            //TODO put this back in?? or just use targetingOdo as poseSupplier for paths
            //prefer the latter to avoid the error spike shenanigans
            // seedFieldRelative(targetingOdo.getEstimatedPosition());
            // seedFieldRelativeWithOffset(targetingOdo.getEstimatedPosition().getRotation());
        }
    }

    private void testDrive(){
        applyRequest(()->driveFieldCentricNoDeadband
            .withVelocityX(0.35)
            .withVelocityY(0.0)
            .withRotationalRate(0.0)
        );
    }
    
    //#endregion

    //#region getters
    private boolean soFine = false;

    public void setSoFine(boolean sosoFine){
        this.soFine = sosoFine;
    }
    public Pose2d getTargetingOdoPose(){
        return this.targetingOdo.getEstimatedPosition();
    }
    private double getDriveXWithoutDeadband() {
        return -Math.copySign(Math.pow(joystick.getLeftY(), 2.0), joystick.getLeftY())*(soFine?0.25:1.0);
    }

    private double getDriveXWithDeadband() {
        return addDeadband(getDriveXWithoutDeadband());
    }

    private double getDriveYWithoutDeadband() {
        return -Math.copySign(Math.pow(joystick.getLeftX(), 2.0), joystick.getLeftX())*(soFine?0.25:1.0);
    }

    private double getDriveYWithDeadband() {
        return addDeadband(getDriveYWithoutDeadband());
   }

    private double getDriveRotationWithoutDeadband() {
        return -Math.copySign(Math.pow(joystick.getRightX(), 2.0), joystick.getRightX());
    }

    private double getDriveRotationWithDeadband() {
        return addDeadband(getDriveRotationWithoutDeadband())*(soFine?0.25:1.0);
    }

    private double addDeadband(double input) {
        return Math.abs(input) > STICK_DEADBAND ? input : 0.0;
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
        // return rolloverConversion_radians(
        return this.m_fieldRelativeOffset.getRadians() - getPose().getRotation().getRadians();
        // return
        // rolloverConversion_radians(this.m_fieldRelativeOffset.getRadians()-getOdoPose().getRotation().getRadians());
    }

    public double getFieldRelativeGyro() {
        return m_odometry.getEstimatedPosition().relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation().getRadians();
    }

    public double getBlueRelativeGyroDegrees() {
        return getGyroAngle().relativeTo(new Pose2d(0, 0, m_blueFieldRelativeOffset)).getRotation().getDegrees();
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
                        new ReplanningConfig(false, false, 1.0, 0.25)),
                () -> false, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command getAutoPath(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        // return new PathPlannerAuto(pathName);
    }

    // public Targeting getFrontTargeting(){
    //     return frontCamera;
    // }

    public boolean canSeeTargetTag(){
        PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        for (int i = 0; i < botPoseEstimate.rawFiducials.length; i++) {
            if (botPoseEstimate.rawFiducials[i].id == Targeting.getTargetID()) {
                return true;
            }
        }
        return false;
    }

    public Pose2d getCurrPathEnd(){
        return pathFollower.getPathEnd();
    }
    //#endregion auto stuff

    public enum DriveMode {
        JOYSTICK,
        // JOYSTICK_BOTREL,
        AUTON,
        AIMATTARGET, 
        AIMATTARGET_AUTON,
        AIMATTRAP,
        AIM_AT_NOTE,
        RESET_GYRO,
        // ODOMETRYTRACK,
        STRAFE2APRILTAG,
        TEST
        ;
    }

    @Override
    public void periodic() {
        // LimelightHelpers.getLatestResults("limelight-front");
        frontCamera.update();
        odometryTargeting.update();
        
        // if (sideMode != RobotContainer.getInstance().getSideChooser().getSelected()) {
        //     sideMode = RobotContainer.getInstance().getSideChooser().getSelected();
        //     Targeting.setTarget(sideMode == SideMode.BLUE ? Target.BLUESPEAKER : Target.REDSPEAKER);
        // }
    
        SmartDashboard.putString("side", sideMode.toString());

        if (Constants.debug) {
            SmartDashboard.putBoolean("can see note", noteLimelight.hasTarget());
            SmartDashboard.putNumber("odo x", getPose().getX());
            SmartDashboard.putNumber("odo y", getPose().getY());

            SmartDashboard.putNumber("notetx", noteLimelight.getTargetHorizOffset());
            SmartDashboard.putString("DriveTrain State", drivetrain_state);

            SmartDashboard.putBoolean("is snapping", isSnapping);

            SmartDashboard.putNumber("joystickDriveHoldAngle", joystickDriveHoldAngleRadians);
            SmartDashboard.putNumber("gyroAngle", getGyroAngleRadians());
            SmartDashboard.putNumber("getBotAz_FieldRelative()", Math.toDegrees(getBotAz_FieldRelative()));
            SmartDashboard.putNumber("odometryTargeting.getAz()", odometryTargeting.getAz());
            SmartDashboard.putNumber("odometryTargeting.getEl()", odometryTargeting.getEl());
            SmartDashboard.putNumber("odometryTargeting.getDistance_XY", (odometryTargeting.getDistance_XY() / 0.0254) / 12.0);
            
            
            SmartDashboard.putNumber("frontCamera.getAz()", frontCamera.getAz());
            SmartDashboard.putNumber("frontCamera.getEl()", frontCamera.getEl());
            SmartDashboard.putNumber("frontCamera.getDistance_XY", (frontCamera.getDistance_XY() / 0.0254) / 12.0);

            SmartDashboard.putString("", mControlMode.toString());
            // SmartDashboard.putNumber("getPose().getX()", getEstimatedPosition().getX());
            // SmartDashboard.putNumber("getPose().getY()", getEstimatedPosition().getY());
            SmartDashboard.putNumber("getPose().getX()", getPose().getX());
            SmartDashboard.putNumber("getPose().getY()", getPose().getY());
            SmartDashboard.putNumber("odometryTargeting.getBotPosX()", odometryTargeting.getBotPosX());
            SmartDashboard.putNumber("odometryTargeting.getBotPosY()", odometryTargeting.getBotPosY());
            SmartDashboard.putNumber("odometryTargeting.getAz()", odometryTargeting.getAz());
            SmartDashboard.putNumber("frontCamera.getBotPosX()", frontCamera.getBotPosX());
            SmartDashboard.putNumber("frontCamera.getBotPosY()", frontCamera.getBotPosY());
            SmartDashboard.putNumber("frontCamera.getAz()", frontCamera.getAz());
            SmartDashboard.putNumber("frontCamera.getDistance_XY_average()", (frontCamera.getDistance_XY_average() / 0.0254) / 12.0);
            SmartDashboard.putNumber("odometry.getDistance_XY_average()", (odometryTargeting.getDistance_XY_average() / 0.0254) / 12.0);

            // SmartDashboard.putNumber("frontCamera.getDistanceToTargetInches()", frontCamera.getDistanceToTargetInches() / 12);

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
            SmartDashboard.putNumber("m_offset", m_fieldRelativeOffset.getDegrees());

            SmartDashboard.putNumber("yaw offset", yawOffset);
        }
        SmartDashboard.putBoolean("will autos work", !Utils.isSimulation());
        // SmartDashboard.putNumber("limelight gyro", getOdoPose().getRotation().getDegrees());
        // PPLibTelemetry.setTargetPose(limelight.getBotPosePose());

        if(mControlMode != DriveMode.AUTON){
            PPLibTelemetry.setCurrentPose(getPose());
            PPLibTelemetry.setTargetPose(targetingOdo.getEstimatedPosition());
        }
    }

    public void seedFieldRelativeWithOffset(Rotation2d offset) {
        try {
            m_stateLock.writeLock().lock();
            m_blueFieldRelativeOffset = getGyroAngle().getRotation().minus(offset);
            if(getSideMode()==SideMode.RED){
                offset = new Rotation2d(Math.PI).plus(offset);
            }
            m_fieldRelativeOffset = getState().Pose.getRotation().minus(offset);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void tareEverything() {
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, new Pose2d());
            targetingOdo.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, new Pose2d());
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void seedFieldRelative(Pose2d location) {
        try {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, location);
            targetingOdo.resetPosition(Rotation2d.fromDegrees(m_yawGetter.getValue()), m_modulePositions, location);
            /* We need to update our cached pose immediately so that race conditions don't happen */
            m_cachedState.Pose = location;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds) {
        try {
            m_stateLock.writeLock().lock();
            targetingOdo.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            targetingOdo.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    @Override
    public void update(double time, double dt) {
        double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(m_yawGetter, m_angularVelocity);
        try {
            m_stateLock.writeLock().lock();
            /* Keep track of previous and current pose to account for the carpet vector */
            targetingOdo.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
        }finally {
            m_stateLock.writeLock().unlock();
        }
        // chooseVisionAlignGoal();
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
            case RESET_GYRO:
                // Do nothing
                break;
            case STRAFE2APRILTAG:
                strafeToAprilTag();
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
            case TEST:
                testDrive();
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
    // public void visionAlignDrive(Translation2d translation2d, boolean fieldRelative) {
    //     // drive(translation2d, mVisionAlignAdjustment, fieldRelative, false);
    // }

    // public void angleAlignDrive(Translation2d translation2d, double targetHeading, boolean fieldRelative) {
    //     snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(targetHeading), 0.0));
    //     double angleAdjustment = snapPIDController.calculate(getBotAz_FieldRelative());
    //     // drive(translation2d, angleAdjustment, fieldRelative, false);
    // }

    // public void acceptLatestGoalTrackVisionAlignGoal(double vision_goal) {
    //     mGoalTrackVisionAlignGoal = vision_goal;
    // }

    // public void chooseVisionAlignGoal() {
    //     double currentAngle = getBotAz_FieldRelative();
    //     if (limelight.hasTarget()) {
    //         double targetOffset = frontCamera.getAz();
    //         mLimelightVisionAlignGoal = MathUtil.inputModulus(currentAngle - targetOffset, 0.0, 2 * Math.PI);
    //         visionPIDController.setSetpoint(mLimelightVisionAlignGoal);
    //     } else {
    //         visionPIDController.setSetpoint(mGoalTrackVisionAlignGoal);
    //     }

    //     mVisionAlignAdjustment = visionPIDController.calculate(currentAngle);
    // }
    
    public double calculateSnapValue() {
        return snapPIDController.calculate(MathUtil.inputModulus(getBotAz_FieldRelative(), -Math.PI,  Math.PI));
    }

    public void startSnap(double snapAngle) {
        snapPIDController.reset(MathUtil.inputModulus(getBotAz_FieldRelative(), -Math.PI,  Math.PI));
        double alignAngleRadians = MathUtil.inputModulus(Math.toRadians(snapAngle), -Math.PI,  Math.PI);
        snapPIDController.setGoal(new TrapezoidProfile.State(alignAngleRadians, 0.0));
        isSnapping = true;
    }

    TimeDelayedBoolean delayedBoolean = new TimeDelayedBoolean();

    public boolean snapComplete() {
        double error = MathUtil.inputModulus(snapPIDController.getGoal().position, -Math.PI,  Math.PI) - MathUtil.inputModulus(getBotAz_FieldRelative(), -Math.PI,  Math.PI);
        // SmartDashboard.putNumber("Snap error", Math.toDegrees(error));
        return delayedBoolean.update(Math.abs(error) < Math.toRadians(Constants.SnapConstants.kEpsilon),
                Constants.SnapConstants.kTimeout);     
    }

    public void maybeStopSnap(boolean force) {
        if (!isSnapping) {
            // SmartDashboard.putNumber("Snapcomplete = ", -1);
            return;
        }
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(getBotAz_FieldRelative());
            // SmartDashboard.putNumber("Snapcomplete = ", 1);
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

        /* Use one of these sysidroutines for your particular test */
    // private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Volts.of(4),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
    //                 null,
    //                 this));

    // private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Volts.of(4),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(RotationCharacterization.withVolts(volts)),
    //                 null,
    //                 this));
    // private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //                 null,
    //                 Volts.of(7),
    //                 null,
    //                 (state) -> SignalLogger.writeString("state", state.toString())),
    //         new SysIdRoutine.Mechanism(
    //                 (volts) -> setControl(SteerCharacterization.withVolts(volts)),
    //                 null,
    //                 this));

    // /* Change this to the sysid routine you want to test */
    // private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    // /*
    //  * Both the sysid commands are specific to one particular sysid routine, change
    //  * which one you're trying to characterize
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return RoutineToApply.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return RoutineToApply.dynamic(direction);
    // }
    
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
    public void odometryTrack() {
        drivetrain_state = "ODOMETRYTRACK";
        Double offset = getBotAz_FieldRelative() - odometryTargeting.getAz();
        offset = rolloverConversion_radians(offset);
        Double request = aimAtTargetController.calculate(offset, 0.005);

        
        setDriveOrientation(DriveOrientation.FIELD_CENTRIC);
        updateDrive(request);

        if(Constants.debug){
            SmartDashboard.putNumber("PID Output:", request/Constants.MaxAngularRate);
            SmartDashboard.putNumber("PID Error:", offset);  
        }
    }

    public void setSideMode(Alliance side) {
        switch (side) {
            case Red:
                this.sideMode = SideMode.RED;
                break;
            case Blue:
                this.sideMode = SideMode.BLUE;
                break;
            default:
                 System.out.println("ERROR setting side");
        }
    }

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

