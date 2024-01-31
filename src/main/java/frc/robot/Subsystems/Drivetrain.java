package frc.robot.Subsystems;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Limelight;
import frc.robot.util.Targeting;
import frc.robot.util.Control.PidConstants;
import frc.robot.util.Control.PidController;
import frc.robot.util.Math.Rotation2;
import frc.robot.util.PathFollowing.FollowPathCommand;
import frc.robot.util.Targeting.Target;
import frc.robot.util.UpdateManager;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem, UpdateManager.Updatable{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private DriveMode mControlMode = DriveMode.APRIL_TAG;

    private PidController limelightController = new PidController(new PidConstants(1.0, 0.002, 0.0));
    private PidController aprilTagController = new PidController(new PidConstants(1.0, 0.0, 0.0));


    private Limelight limelight = Limelight.getInstance();
    private Targeting frontCamera = new Targeting("front", false);

    private boolean withOdo = false;

    private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;

    private FollowPathCommand pathFollower;

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);

        limelightController.setContinuous(false);
        limelightController.setInputRange(-Math.toRadians(45.0), Math.toRadians(45.0));
        limelightController.setOutputRange(-1.0, 1.0);
        limelightController.setSetpoint(0.0);

        aprilTagController.setContinuous(true);
        aprilTagController.setInputRange(-Math.PI, Math.PI);
        aprilTagController.setOutputRange(-1.0, 1.0);
        aprilTagController.setSetpoint(0.0);

        Targeting.setTarget(Target.REDSPEAKER);

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        // this.m_pigeon2.setYaw(0);

        limelightController.setContinuous(true);
        limelightController.setInputRange(0.0, Math.PI*2);
        limelightController.setOutputRange(-1.0, 1.0);
        limelightController.setSetpoint(0.0);

        aprilTagController.setContinuous(true);
        aprilTagController.setInputRange(-Math.PI, Math.PI);
        aprilTagController.setOutputRange(-1.0, 1.0);
        aprilTagController.setSetpoint(0.0);

        Targeting.setTarget(Target.REDSPEAKER);

        limelight.getBotPose();

        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        ReplanningConfig replan = new ReplanningConfig();

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig());

        pathFollower = new FollowPathCommand(
            ()->this.getState().Pose,
            this::getCurrentRobotChassisSpeeds,
            new PPHolonomicDriveController(
                config.translationConstants, config.rotationConstants, config.period, config.maxModuleSpeed, config.driveBaseRadius),
            config.replanningConfig,
            ()->false
        );

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public static double rolloverConversion_radians(double angleRadians){
        //Converts input angle to keep within range -pi to pi
        if(angleRadians > Math.PI){
            return (angleRadians %Math.PI - Math.PI);
        }else if (angleRadians < -Math.PI){
            return (angleRadians % Math.PI - Math.PI);
        }else{
            // System.err.println("Conversion Error");
            return angleRadians;
        }
    }

    public void setPath(PathPlannerPath path, boolean resetPose){
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
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->fromChassisSpeed(speeds), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->false, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    private void fromChassisSpeed(ChassisSpeeds speeds) {
        var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for(int i=0; i<this.Modules.length; i++){
            this.Modules[i].apply(states[i], 
            SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        // return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setDriveMode(DriveMode mode){
        limelightController.integralAccum = 0;
        aprilTagController.integralAccum = 0;
        mControlMode = mode;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void limelightDrive() {
        if(limelight.hasTarget()){
            Double offset = Math.toRadians(limelight.getTargetHorizOffset());
            Double request = limelightController.calculate(offset, 0.02) * Constants.MaxAngularRate;
            SmartDashboard.putNumber("PID Turn Rate", request);

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
                getDriveX() * Constants.MaxSpeed, 
                getDriveY() * Constants.MaxSpeed, 
                request,
                m_odometry.getEstimatedPosition()
                        .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()
            ),0.2);
            
            var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for(int i=0; i<this.Modules.length; i++){
                this.Modules[i].apply(states[i], 
                SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
            }
        }else{
            SmartDashboard.putNumber("PID Turn Rate", 0.0);
            joystickDrive();
        }
    }

    public void setSnapToTarget(boolean snap){
        mControlMode = DriveMode.APRIL_TAG;
        withOdo = snap;
    }

    public Pose2d getOdoPose(){
        return this.m_odometry.getEstimatedPosition();
    }

    public void snapToTarget(){
        Targeting.updateAll();
        Double offset = rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians())-Targeting.getMovingAverageAz();
        offset = rolloverConversion_radians(offset);
        Double request = aprilTagController.calculate(offset, 0.02)*Constants.MaxAngularRate;
        SmartDashboard.putNumber("PID Turn Rate", request/Constants.MaxAngularRate);
        SmartDashboard.putNumber("target az offset", offset);

        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            getDriveX() * Constants.MaxSpeed, 
            getDriveY() * Constants.MaxSpeed, 
            request,
            m_odometry.getEstimatedPosition()
                    .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()
        ),0.2);
        
        var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for(int i=0; i<this.Modules.length; i++){
            this.Modules[i].apply(states[i], 
            SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        }

        if(limelight.hasTarget()){
            withOdo=false;
        }
    }

    public void aprilTagTrack() {
        if(withOdo){
            snapToTarget();
            return;
        }
        Targeting.updateAll();
        Double offset = Targeting.getMovingAverageAz()-rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians());
        offset = rolloverConversion_radians(offset);
        Double request = aprilTagController.calculate(offset, 0.02)*Constants.MaxAngularRate;
        SmartDashboard.putNumber("PID Output:", request/Constants.MaxAngularRate);
        SmartDashboard.putNumber("PID Error:", offset);

        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            getDriveX() * Constants.MaxSpeed, 
            getDriveY() * Constants.MaxSpeed, 
            request,
            m_odometry.getEstimatedPosition()
                    .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()
        ),0.2);
        
        var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for(int i=0; i<this.Modules.length; i++){
            this.Modules[i].apply(states[i], 
            SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        }
    }

    public void joystickDrive(){

        ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            getDriveX() * Constants.MaxSpeed, 
            getDriveY() * Constants.MaxSpeed, 
            getDriveRotation() * Constants.MaxAngularRate,
            m_odometry.getEstimatedPosition()
                    .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset)).getRotation()
        ),0.2);

        var states = m_kinematics.toSwerveModuleStates(speeds, new Translation2d());

        for(int i=0; i<this.Modules.length; i++){
            this.Modules[i].apply(states[i], 
            SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
        }
    }

    private double getDriveX(){
        return ((Math.abs(joystick.getLeftY())>0.1)?-joystick.getLeftY():0.0);
    }

    private double getDriveY(){
        return ((Math.abs(joystick.getLeftX())>0.1)?-joystick.getLeftX():0.0);
    }

    private double getDriveRotation(){
        return ((Math.abs(joystick.getRightX())>0.1)?joystick.getRightX():0.0);
    }

    public boolean pathDone(){
        return pathFollower.pathDone();
    }

    public double getPathTime(){
        return pathFollower.getPathTime();
    }

    // public void setJoystick(CommandXboxController joystick){
    //     this.joystick = joystick;
    // }

    public enum DriveMode{
        JOYSTICK,
        LIMELIGHT,
        AUTON,
        APRIL_TAG,
        ;
    }

    @Override
    public void periodic(){
        // targeting.update();
        frontCamera.update();
        //Troubeshooting if Swerve Robot Azimuth Output matches Targeting Class Azimuth Output
        SmartDashboard.putString("", mControlMode.toString());
        SmartDashboard.putNumber("Targeting az", frontCamera.getAz());
        SmartDashboard.putNumber("Targeting.getMovingAverageAz()", Targeting.getMovingAverageAz());
        SmartDashboard.putString("target:",Targeting.getTarget().toString());
        SmartDashboard.putNumber("Bot Azimuth:", (getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians()));
        SmartDashboard.putNumber("Bot Azimuth [CONVERTED]:", rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians()));
    }


    @Override
    public void update(double time, double dt) {
        // System.out.println("ah");
        switch(mControlMode){
            case APRIL_TAG:
                aprilTagTrack(); break;
            case LIMELIGHT:
                limelightDrive(); break;
            case JOYSTICK:
                joystickDrive(); break;
            case AUTON:
                var states = m_kinematics.toSwerveModuleStates(pathFollower.update(), new Translation2d());

                for(int i=0; i<this.Modules.length; i++){
                    this.Modules[i].apply(states[i], 
                    SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
                };  break;

        }
    }
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }
    public boolean hasTarget() {
        return limelight.hasTarget();
    }
    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians()));
    }
}