package frc.robot.Subsystems;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import frc.robot.Swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import frc.robot.Swerve.SwerveRequest;
import frc.robot.Swerve.SwerveModule.DriveRequestType;
import frc.robot.Swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Control.PidConstants;
import frc.robot.util.Control.PidController;
import frc.robot.util.PathFollowing.FollowPathCommand;
import frc.robot.util.UpdateManager;
import frc.robot.util.Camera.Limelight;
import frc.robot.util.Camera.LimelightHelpers;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.Target;
import frc.robot.util.Choosers.SideChooser.SideMode;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem, UpdateManager.Updatable{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private DriveMode mControlMode = DriveMode.JOYSTICK;
    private SideMode sideMode = SideMode.RED;

    // private PidController limelightController = new PidController(new PidConstants(1.0, 0.002, 0.0));
    // private PidController aprilTagController = new PidController(new PidConstants(1.0, 0.0, 0.0));
    // private PidController aimAtSpeaker = new PidController(new PidConstants(1, 0.2, 0));

    private PidController aimAtTargetController = new PidController(new PidConstants(1.0, 0.002, 0.0));

    private Limelight limelight = new Limelight("front");
    private Targeting frontCamera = new Targeting("front", false);
    private Targeting odometryTargeting = new Targeting(true);
    
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

        aimAtTargetController.setContinuous(true);
        aimAtTargetController.setInputRange(-Math.PI, Math.PI);
        aimAtTargetController.setOutputRange(-1.0, 1.0);
        aimAtTargetController.setSetpoint(0.0);

        Targeting.setTarget(Target.REDSPEAKER);

        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        aimAtTargetController.setContinuous(true);
        aimAtTargetController.setInputRange(-Math.PI, Math.PI);
        aimAtTargetController.setOutputRange(-1.0, 1.0);
        aimAtTargetController.setSetpoint(0.0);

        Targeting.setTarget(Target.REDSPEAKER);

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
            ()->this.getPose(),
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

    public void setPath(PathPlannerPath path, boolean resetPose){
        pathFollower.setPath(path);
        pathFollower.initialize(this, resetPose);
    }

    public void stopPath() {
        pathFollower.stopPath();
    }

    public SideMode getSideMode(){
        return sideMode;
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
        aimAtTargetController.integralAccum = 0;
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

    public Pose2d getOdoPose(){
        return this.m_odometry.getEstimatedPosition();
    }

    public double getBotAz_FieldRelative(){
        //Note from James to Zac, I'm changing this back to the 'old' one to see if it works...
        return rolloverConversion_radians(this.m_fieldRelativeOffset.getRadians()-getPose().getRotation().getRadians());
        // return rolloverConversion_radians(this.m_fieldRelativeOffset.getRadians()-getOdoPose().getRotation().getRadians());
    }

    //odometryTrack():
    //Uses the Odometries X and Y position, relative to the target, and drives the yaw of the drivetrain to face the target.

    //odometryTrack() needs:
    //Targeting.setTarget(Target.#########); needs to be ran prior, this sets the targetPos that the drivetrain will orient to.
    //private Targeting odometryTargeting = new Targeting(true); must be added.
    //PID Controller "aimAtTargetController" must be created
    public void odometryTrack(){
        Double offset = getBotAz_FieldRelative() - odometryTargeting.getAz();
        offset = rolloverConversion_radians(offset);
        Double request = aimAtTargetController.calculate(offset, 0.02)*Constants.MaxAngularRate;

        
        // SmartDashboard.putNumber("odometryTargetng", ModuleCount)
        // SmartDashboard.putNumber("PID Output:", request/Constants.MaxAngularRate);
        // SmartDashboard.putNumber("PID Error:", offset);

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

    //aprilTagTrack():
    //Uses the limelights TX value for a targeted ID (horizontal centering of the april tag in the limelights video feed),
    //... and controls the Yaw of the robot keep the drivetrain facing the target, with the targeted april tag centered.
    //... if the april tag cannot be seen, joystickDrive() is enabled.

    //aprilTagTrack() needs:
    //Targeting.setTarget(Target.#########); needs to be ran prior, this sets which april tag ID to lock onto.
    //A limelight with the host name 'limelight-front' must be connected to the RIO's network
    //The limelight must be mounted facing in the direction you would like the drivetrain to aim at the target
    //PID Controller "aimAtTargetController" must be created
    public void aprilTagTrack(){
        // Get JSON Dump from Limelight-front
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

        // Go through limelight JSON dump, and look for Target ID
        // If ID found, save TX value to offset for targeting.
        boolean canSeeTarget = false;
        double offset = 0;
        for(var aprilTagResults : llresults.targetingResults.targets_Fiducials){
            if (aprilTagResults.fiducialID == Targeting.getTargetID()){
                offset = -(Math.toRadians(aprilTagResults.tx));
                canSeeTarget = true;
            }
        }

        if(canSeeTarget){
            Double request = aimAtTargetController.calculate(offset, 0.02) * Constants.MaxAngularRate;
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
            joystickDrive();
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

    //aimAtTarget():
    //Combines both OdometryTrack and AprilTagTrack()
    //If the targeted aprilTag is in view ... aprilTagTrack() is used.
    //Otherwise, OdometryTrack() is ran, the goal of which is to use OdometryTrack() to point 
    //... the limelight at where the targeted april tag should be.

    //aimAtTarget() needs:
    //Will need everything that odometryTrack() and aprilTagTrack() need.
    //As well as the variables listed below...
    public int updateCounter = 0;
    public String aimAtSpeakerState = "INIT";
    public boolean justChanged = false;
    public void aimAtTarget(){
        // Get JSON Dump from Limelight-front
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

        boolean canSeeTarget = false;

        // Go through limelight JSON dump, and look for Target ID
        // If ID found, save TX value to offset for targeting.
        double offset = 0;
        for(var aprilTagResults : llresults.targetingResults.targets_Fiducials){
            if (aprilTagResults.fiducialID == Targeting.getTargetID()){
                offset = -(Math.toRadians(aprilTagResults.tx));
                canSeeTarget = true;
            }
        }

        //Update Odometry Targeting
        odometryTargeting.update();
        odometryTargeting.getAz();

        //If TargetID cant be seen by Limelight, use odometryTrack()
        if(!canSeeTarget){
            aimAtSpeakerState = "LOOKING";
            odometryTrack();
            if(justChanged){
                justChanged = false;
            }
        }else{ //If TargetID can be seen, use Limelight TX tracking
            if(!justChanged){ //When changing modes, clear integral accumulation
                aimAtTargetController.integralAccum=0;
                justChanged = true;
            }
            aimAtSpeakerState = "LOCKING";

            //aprilTagTack(), but offset is grabbed earlier due to this drive methoed needing to determine if target is in view.
            Double request = aimAtTargetController.calculate(offset, 0.02) * Constants.MaxAngularRate;
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

    public enum DriveMode{
        JOYSTICK,
        AUTON,
        AIMATTARGET,
        AIMATTRAP,
        ;
    }

private boolean odometryBotPosUpdaterMethodFlag = false;
    private boolean odometryBotPosUpdater(){
        if(odometryBotPosUpdaterMethodFlag){
            frontCamera.updateBotPos();
            if (frontCamera.getBotPosX() != 0 && frontCamera.getBotPosY() != 0){ 
                seedFieldRelative(new Pose2d(new Translation2d(frontCamera.getBotPosX(), frontCamera.getBotPosY()), new Rotation2d(getPose().getRotation().getRadians())));
                return true;
            }
        }
        return false;
    }

    public int periodicCounter = 0;
    @Override
    public void periodic(){
        if(mControlMode!=DriveMode.AUTON){
            odometryTargeting.update();
            frontCamera.update();
            if(periodicCounter == 500){
                odometryBotPosUpdaterMethodFlag = true;
            }
            periodicCounter++;

        if(odometryBotPosUpdater()){
            periodicCounter = 0;
            odometryBotPosUpdaterMethodFlag = false;
        }

        sideMode = RobotContainer.getInstance().getSideChooser().getSelected();

        SmartDashboard.putString("side", sideMode.toString());

        SmartDashboard.putString("aimAtSpeakerState",aimAtSpeakerState);
        SmartDashboard.putNumber("getBotAz_FieldRelative()", getBotAz_FieldRelative());
        SmartDashboard.putNumber("odometryTargeting.getAz()", odometryTargeting.getAz());
        SmartDashboard.putNumber("odometryTargeting.getEl()", odometryTargeting.getEl());
        SmartDashboard.putNumber("limelightTargeting.getEl()", frontCamera.getEl());
        SmartDashboard.putString("", mControlMode.toString());
        
        SmartDashboard.putNumber("getPose().getX()", getPose().getX());
        SmartDashboard.putNumber("getPose().getY()", getPose().getY());
        SmartDashboard.putNumber("odometryTargeting.getBotPosX()", odometryTargeting.getBotPosX());
        SmartDashboard.putNumber("odometryTargeting.getBotPosY()", odometryTargeting.getBotPosY());

        SmartDashboard.putString("Set Target:",Targeting.getTarget().toString());
        SmartDashboard.putNumber("Bot Azimuth:", rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians()));
        }
    }


    @Override
    public void update(double time, double dt) {
        // System.out.println("ah");
        switch(mControlMode){
            case AIMATTARGET:
                aimAtTarget(); break;
            case JOYSTICK:
                joystickDrive(); break;
            case AUTON:
                var states = m_kinematics.toSwerveModuleStates(pathFollower.update(), new Translation2d());
                if(!pathDone()){
                    for(int i=0; i<this.Modules.length; i++){
                        this.Modules[i].apply(states[i], 
                        SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
                    };  break;
                }else{
                    rotationHold();
                }
        }
    }

    private void rotationHold() {
        Double offset = pathFollower.lastAngle().getRadians()-rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians());
        offset = rolloverConversion_radians(offset);
        Double request = aimAtTargetController.calculate(offset, 0.02)*Constants.MaxAngularRate;
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

    public static double rolloverConversion_radians(double angleRadians){
        //Converts input angle to keep within range -pi to pi
        if((angleRadians > Math.PI) || (angleRadians < -Math.PI)){
            return (((angleRadians + Math.PI) % (2*Math.PI))-Math.PI);
        }else{
            return angleRadians;
        }
    }

    public Pose2d getPose() {
        //Note from James to Zac, I'm changing this back to the 'old' one to see if it works...
        return m_odometry.getEstimatedPosition();
        // return new Pose2d(this.m_odometry.getEstimatedPosition().getTranslation(), Rotation2d.fromRadians(getBotAz_FieldRelative()));
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(rolloverConversion_radians(getPose().getRotation().getRadians()-this.m_fieldRelativeOffset.getRadians()));
    }

    public boolean hasTarget() {
        return limelight.hasTarget();
    }
    public Targeting getOdoTargeting() {
        return odometryTargeting;
    }
    public Targeting getLimelightTargeting(){
        return frontCamera;
    }
    public DriveMode getDriveMode() {
        return mControlMode;
    }
}