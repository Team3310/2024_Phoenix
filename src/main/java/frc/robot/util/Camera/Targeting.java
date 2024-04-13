package frc.robot.util.Camera;

import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.LimelightHelpers.PoseEstimate;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Interpolable.InterpolatingDouble;

public class Targeting {
    private LinearFilter distanceFilter;
    //#region Static Constants
    // Cartesian Coordinates of Targets
    // Origin is BOTTOM LEFT OF FIELD
    // X, Y, Z (Inches)
    private static final double[] blueSpeaker =     { -0.2000000, 5.5478680, 2.0432395 };
    private static final double[] blueAmp =         { 14.7007580, 8.2042000, 0.8891270 };
    private static final double[] blueTrap1 =       {  4.6413420, 4.4983400, 1.6414750 };
    private static final double[] blueTrap2 =       {  4.6413420, 3.7132260, 1.6414750 };
    private static final double[] blueTrap3 =       {  5.3207920, 4.1051480, 1.6414750 };
    private static final double[] redSpeaker =      { 16.700000,  5.5478680, 2.0432395 };
    private static final double[] redAmp =          {  1.8415000, 8.2042000, 0.8891270 };
    private static final double[] redTrap1 =        { 11.9047260, 3.7132260, 1.6414750 };
    private static final double[] redTrap2 =        { 11.9047260, 4.4983400, 1.6414750 };
    private static final double[] redTrap3 =        { 11.2201960, 4.1051480, 1.6414750 };
    private static final double[] centerOfField =   {  5.3207920, 4.1067482, 0.0000000 };
    private static final double[] blueCenterPass =  {  6.6300000, 10.0000000, 0.0000000 }; 
    private static final double[] redCenterPass =   {  9.8600000, 10.0000000, 0.0000000 };
    private static final double[] blueCornerPass =  {  1.0000000, 10.0000000, 0.0000000 };
    private static final double[] redCornerPass =   { 15.5000000, 10.0000000, 0.0000000 };

    private static final double blueSpeakerID = 7;
    private static final double blueAmpID = 6;
    private static final double blueTrap1ID = 15;
    private static final double blueTrap2ID = 16;
    private static final double blueTrap3ID = 14;
    private static final double redSpeakerID = 4;
    private static final double redAmpID = 5;
    private static final double redTrap1ID = 11;
    private static final double redTrap2ID = 12;
    private static final double redTrap3ID = 13;
    private static final double nullID = 0;

    private static final double speakerHeightInches = 57.0;
    private static final double cameraMountAngleDegrees = 15.0;
    private static final double caneraMountHeightInches = 23.25;

    private static final int NUMBER_TAPS_MOVING_AVERAGE = 10;
    // private static final double[] distanceFilterInputs = new double[NUMBER_TAPS_MOVING_AVERAGE];
    // private static final double[] distanceFilterOutputs = new double[NUMBER_TAPS_MOVING_AVERAGE];
    private static boolean distanceFilterNeedsReset = false;

    //#endregion

    public enum Target {
        BLUESPEAKER(blueSpeaker, blueSpeakerID), BLUAMP(blueAmp, blueAmpID), BLUTRAP1(blueTrap1, blueTrap1ID),
        BLUTRAP2(blueTrap2, blueTrap2ID), BLUTRAP3(blueTrap3, blueTrap3ID), REDSPEAKER(redSpeaker, redSpeakerID),
        REDAMP(redAmp, redAmpID), REDTRAP1(redTrap1, redTrap1ID), REDTRAP2(redTrap2, redTrap2ID),
        REDTRAP3(redTrap3, redTrap3ID), NONE(centerOfField, nullID), BLUECENTERPASS(blueCenterPass, nullID),
        REDCENTERPASS(redCenterPass, nullID), BLUECORNERPASS(blueCornerPass, nullID), REDCORNERPASS(redCornerPass, nullID);

        private double[] pos;
        private double id;

        private Target(double[] toTargetPos, double toTargetID) {
            this.pos = toTargetPos;
            this.id = toTargetID;
        }

        public double[] getPos() {
            return pos;
        }

        public double getID() {
            return id;
        }
    }

    public enum TargetSimple{
        SPEAKER,
        CORNERPASS,
        CENTERPASS, NONE;
    }
    private final Drivetrain drivetrain; 
    //#region Constructors
    // Targeting Constructor, Limelight
    public Targeting(String limelightHostname, boolean isOdometry) {
        this.drivetrain = TunerConstants.DriveTrain;
        this.limelightHostname = "-" + limelightHostname;
        this.limelight = new Limelight(limelightHostname);
        this.isOdometry = isOdometry;
        this.distanceFilter = LinearFilter.movingAverage(NUMBER_TAPS_MOVING_AVERAGE);
    }

    
    // Targeting Constructor, Odometry
    public Targeting(boolean isOdometry, Drivetrain drivetrain) { 
        // this("junk", isOdometry);
        this.drivetrain = drivetrain;
        this.isOdometry = isOdometry;
        this.limelight = new Limelight();
        this.distanceFilter = LinearFilter.movingAverage(NUMBER_TAPS_MOVING_AVERAGE);
    }
    //#endregion

    //#region Static
    private static double[] targetPos = centerOfField;
    private static double targetID = nullID;
    private static Target target = Target.NONE;
    private static TargetSimple targetSimple = TargetSimple.SPEAKER;

    public static void setTarget(Target toTarget) {
        target = toTarget;
        targetPos = target.getPos();
        targetID = target.getID();
    }

    public static void setTargetSimple(TargetSimple toTargetSimple) {
        targetSimple = toTargetSimple;
        try{
            if(DriverStation.getAlliance().get() == SideMode.BLUE.getAlliance()){
                switch (targetSimple){
                    case SPEAKER:
                        target = target.BLUESPEAKER;
                        break;
                    case CENTERPASS:
                        target = target.BLUECENTERPASS;
                        break;
                    case CORNERPASS:
                        target = target.BLUECORNERPASS;
                        break;
                    case NONE:
                        target = target.NONE;
                        break;
                }
            } else {
                switch (targetSimple) {
                    case SPEAKER:
                        target = target.REDSPEAKER;
                        break;
                    case CENTERPASS:
                        target = target.REDCENTERPASS;
                        break;
                    case CORNERPASS:
                        target = target.REDCORNERPASS;
                        break;
                    case NONE:
                        target = target.NONE;
                        break;
                }
            }
            setTarget(target);
        }catch(Exception e){
            // e.printStackTrace();
            System.err.println("simple target set failed");
        }
    }

    //#region Static Getters
    public static Target getTarget() {
        return target;
    }

    public static TargetSimple getTargetSimple() {
        return targetSimple;
    }

    public static double getTargetID() {
        return targetID;
    }
    //#endregion Static Getters

    public static double[] getTargetAzElFromPoint(double x, double y) {
        // AZIMUTH
        // -----------------------------------------------------------------------------------------------------------------------
        // 0 degrees is in the fields Y Positive Direction
        // pi/2 (90 degrees) is in the fields X Positive Direction
        // pi (180 degrees) is in the fields Y Negative Direction
        // -pi/2 (-90 degrees) is in the fields X Negative Direction
        // Range is from pi to ~-pi (180 to -179.999)

        // ELEVATION
        // -----------------------------------------------------------------------------------------------------------------------
        // 0 degrees is along the XY plane
        // 90 degrees is in the fields Z Positive Direction
        // -90 degrees is in the fields Z Negative Direction
        // Range is from pi/2 to -pi/2 (180 to -180)

        // Compute the X, Y, and Z distances between coordinates
        double delta_X = targetPos[0] - x;
        double delta_Y = targetPos[1] - y;
        double delta_Z = targetPos[2] - 0.0;

        double targetAz = 0.0;
        double targetEl = 0.0;

        double leftShooterSpeed = 0.0;
        double rightShooterSpeed = 0.0;

        // Azimuth Trig
        if (delta_Y > 0) { // (pi/2 to -pi/2), (90 deg to -90 deg))
            targetAz = Math.atan(delta_X / delta_Y);
        } else if ((delta_X > 0) && (delta_Y <= 0)) { // [pi/2 to pi)
            targetAz = -(Math.atan(delta_Y / delta_X)) + (Math.PI / 2);
        } else if ((delta_X < 0) && (delta_Y <= 0)) { // [-pi/2 to -pi)
            targetAz = -(Math.atan(delta_Y / delta_X)) - (Math.PI / 2);
        } else if ((delta_X == 0) && (delta_Y < 0)) { // Edge case: pointing along Y-Axis, Y-: pi (180 deg)
            targetAz = Math.PI;
        } else if ((delta_X == 0) && (delta_Y == 0)) { // This should never occur in a realistic scenario, will not
                                                       // update AZ.
            {}; // Nothing is done if both delta_X and delta_Y are 0, targetAz is not updated.
        } else { // No cases ran, should never occur
            {};
        }

        // Elevation Trig
        double distance_XY = Math.hypot(delta_X, delta_Y);
        // SmartDashboard.putNumber("Distance2Target", (distance_XY / 0.0254) / 12.0);
        if (distance_XY != 0) {
            targetEl = Constants.kLiftAngleMap
                    .getInterpolated(new InterpolatingDouble((distance_XY / 0.0254) / 12.0)).value;
            leftShooterSpeed = Constants.kLeftShooterMap
                    .getInterpolated(new InterpolatingDouble((distance_XY / 0.0254) / 12.0)).value;
            rightShooterSpeed = Constants.kRightShooterMap
                    .getInterpolated(new InterpolatingDouble((distance_XY / 0.0254) / 12.0)).value;
        } else {
            {}; // Nothing is done if the distance is 0 somehow,
        }

        return new double[]{targetAz, targetEl, leftShooterSpeed, rightShooterSpeed};
    }
    //#endregion Static

    //#region Non-Static
    

    private double[] botPos = centerOfField;
    private double targetAz = 0;
    private double targetEl = 0;
    private double leftShooterSpeed = 0;
    private double rightShooterSpeed = 0;
    private double distance_XY = 0.0;
    private double distance_XY_Average = 0.0;
    private double distance_XY_pos = 0.0;
    private String limelightHostname;
    private boolean isOdometry = false;
    private final Limelight limelight;

    public void updateBotPos() {
        if (isOdometry) {
            botPos[0] = drivetrain.getPose().getX();
            botPos[1] = drivetrain.getPose().getY();
            botPos[2] = 0.0;
        } else {
            try {
                botPos = limelight.getBotPose();

                if (botPos.length < 3) {
                    // System.err.println(botPos.length);
                    // System.err.println("Targeting Error: " + limelightHostname);
                    // System.err.println("Targeting Error: updateBotPos failed, Limelight gave BAD data, zeroed botPos");
                    botPos = new double[] { 0, 0, 0, 0, 0, 0 };
                }

            } catch (Exception e) {
                // System.err.println("Targeting Error: " + limelightHostname);
                // System.err.println("Targeting Error: updateBotPos failed to get Limelight Data");
                botPos = new double[] { 0, 0, 0, 0, 0, 0 };
            }
        }
    }

    public double getDistanceToTargetInches() {
        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");

        boolean canSeeTarget = false;
        double offset = 0;
        for (var aprilTagResults : llresults.targetingResults.targets_Fiducials) {
            if (aprilTagResults.fiducialID == Targeting.getTargetID()) {
                offset = aprilTagResults.ty;
                canSeeTarget = true;
            }
        }

        double angleToSpeakerDegrees = cameraMountAngleDegrees + offset; //limelight.getTargetVertOffset();
        double heightDelta = speakerHeightInches - caneraMountHeightInches;
        double distance = heightDelta / Math.tan(Math.toRadians(angleToSpeakerDegrees));

        return distance;
    }

    public void updateTargetAzEl() {
        // AZIMUTH
        // -----------------------------------------------------------------------------------------------------------------------
        // 0 degrees is in the fields Y Positive Direction
        // pi/2 (90 degrees) is in the fields X Positive Direction
        // pi (180 degrees) is in the fields Y Negative Direction
        // -pi/2 (-90 degrees) is in the fields X Negative Direction
        // Range is from pi to ~-pi (180 to -179.999)

        // ELEVATION
        // -----------------------------------------------------------------------------------------------------------------------
        // 0 degrees is along the XY plane
        // 90 degrees is in the fields Z Positive Direction
        // -90 degrees is in the fields Z Negative Direction
        // Range is from pi/2 to -pi/2 (180 to -180)

        // Compute the X, Y, and Z distances between coordinates
        double delta_X = targetPos[0] - this.botPos[0];
        double delta_Y = targetPos[1] - this.botPos[1];
        double delta_Z = targetPos[2] - this.botPos[2];

        // If Limelight cannot see any targets... don't calculate, and change latestUpdateSuccesfull to FALSE
        if (!isOdometry && !this.hasTarget()) {
            distanceFilterNeedsReset = true;
            return;
        }

        // Azimuth Trig
        if (delta_Y > 0) { // (pi/2 to -pi/2), (90 deg to -90 deg))
            this.targetAz = Math.atan(delta_X / delta_Y);
        } else if ((delta_X > 0) && (delta_Y <= 0)) { // [pi/2 to pi)
            this.targetAz = -(Math.atan(delta_Y / delta_X)) + (Math.PI / 2);
        } else if ((delta_X < 0) && (delta_Y <= 0)) { // [-pi/2 to -pi)
            this.targetAz = -(Math.atan(delta_Y / delta_X)) - (Math.PI / 2);
        } else if ((delta_X == 0) && (delta_Y < 0)) { // Edge case: pointing along Y-Axis, Y-: pi (180 deg)
            this.targetAz = Math.PI;
        } else if ((delta_X == 0) && (delta_Y == 0)) { // This should never occur in a realistic scenario, will not
                                                       // update AZ.
            {}; // Nothing is done if both delta_X and delta_Y are 0, targetAz is not updated.
        } else { // No cases ran, should never occur
            {};
        }

        // Elevation Trig
        // delta_X += TunerConstants.DriveTrain.getFieldRelativeVelocites().vxMetersPerSecond * Constants.SHOOT_TIME;
        // delta_Y += TunerConstants.DriveTrain.getFieldRelativeVelocites().vyMetersPerSecond * Constants.SHOOT_TIME;
        distance_XY = Math.hypot(delta_X, delta_Y);

        if (distanceFilterNeedsReset) {
            distanceFilter.reset();
            for (int i = 0; i < NUMBER_TAPS_MOVING_AVERAGE; i++) {
                distanceFilter.calculate(distance_XY);
            }
            distanceFilterNeedsReset = false;
        }
        distance_XY_Average = distanceFilter.calculate(distance_XY);
        
        // SmartDashboard.putNumber("Distance2Target", ((distance_XY / 0.0254) / 12.0));
        // distance_XY = getDistanceToTargetInches();
        // SmartDashboard.putNumber("Distance2Target", (distance_XY / 12.0));
 //       SmartDashboard.putNumber("Distance2Target pos", (distance_XY_pos / 0.0254) / 12.0);
        if (distance_XY != 0) {
            this.targetEl = Constants.kLiftAngleMap
                    .getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
            this.leftShooterSpeed = Constants.kLeftShooterMap
                    .getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
            this.rightShooterSpeed = Constants.kRightShooterMap
                    .getInterpolated(new InterpolatingDouble((distance_XY_Average / 0.0254) / 12.0)).value;
            // this.targetEl = Constants.kLiftAngleMap
            //         .getInterpolated(new InterpolatingDouble(distance_XY / 12.0)).value;
            // this.leftShooterSpeed = Constants.kLeftShooterMap
            //         .getInterpolated(new InterpolatingDouble(distance_XY / 12.0)).value;
            // this.rightShooterSpeed = Constants.kRightShooterMap
            //         .getInterpolated(new InterpolatingDouble(distance_XY / 12.0)).value;
        } else {
            {
            }
            ; // Nothing is done if the distance is 0 somehow,
        }
    }

    public boolean hasTarget() {
        if (isOdometry) {
            return true;
        } else {
            return limelight.hasTarget();
        }
    }

    public static double rolloverConversion_radians(double angleRadians) {
        // Converts input angle to keep within range -pi to pi
        if ((angleRadians > Math.PI) || (angleRadians < -Math.PI)) {
            return (((angleRadians + Math.PI) % (2 * Math.PI)) - Math.PI);
        } else {
            return angleRadians;
        }
    }

    public void update() {
        updateBotPos();
        updateTargetAzEl();
    }

    // Drivetrain drivetrain = TunerConstants.DriveTrain;
    double KALMAN_ROTATION_MAX_RATE = 2;    //radians per second
    double KALMAN_MAX_SPEED = 2;            //meters per second
    double KALMAN_APRILTAG_MAX_RANGE = 4.5; //meters
    public void updateKalmanFilter(){
        if (isOdometry) {
            return; //No Kalman Filter updating with odometry!
        }

        PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight" + limelightHostname);

        if(Constants.debug){
            SmartDashboard.putBoolean("LimelightHasTarget", limelight.hasTarget());
        }
        if (limelight.hasTarget()){
            try {
                botPos = limelight.getBotPose();//botpose_wpiblue
                if (botPos.length < 3) {
                    return;
                }
            } catch (Exception e) {
                return;
            }

            double[] targetpose_robotspace = limelight.getTable().getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            double distanceToTarget = Math.hypot(targetpose_robotspace[0], targetpose_robotspace[1]);
            Rotation2d botRotation2d = new Rotation2d(Math.toRadians(botPos[5])); //Believe botPos TZ is in degreess... could be wrong...
            Pose2d botPose2d = new Pose2d(botPos[0], botPos[1], botRotation2d);

            ChassisSpeeds ChassisSpeeds = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds();
            double rotationSpeed = ChassisSpeeds.omegaRadiansPerSecond;
            double xSpeed = ChassisSpeeds.vxMetersPerSecond;
            double ySpeed = ChassisSpeeds.vyMetersPerSecond;
            double translationalSpeed = Math.hypot(xSpeed, ySpeed);

            if((Math.abs(rotationSpeed) < KALMAN_ROTATION_MAX_RATE) && (Math.abs(translationalSpeed) < KALMAN_MAX_SPEED) && (distanceToTarget < KALMAN_APRILTAG_MAX_RANGE)){
                TunerConstants.TargetingDrivetrain.addVisionMeasurement(botPose2d, Timer.getFPGATimestamp());
                if(Constants.debug){
                    SmartDashboard.putString("KALMAN", "Valid");
                }
                PPLibTelemetry.setTargetPose(botPose2d)
                ;
            } else {
                if(Constants.debug){
                    SmartDashboard.putString("KALMAN", "Invalid");
                }
            }

            if(Constants.debug){
                SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
                SmartDashboard.putNumber("rotationSpeed", rotationSpeed);
                SmartDashboard.putNumber("translationSpeed", translationalSpeed);
            }
        }
    }
    
    public void updatePoseEstimatorWithVisionBotPose() {
        if(isOdometry){
            return;
        }
        LimelightHelpers.SetRobotOrientation("limelight-front", TunerConstants.DriveTrain.getOdoPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight" + limelightHostname);
       
        // invalid LL data
        if (botPoseEstimate.pose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = TunerConstants.DriveTrain.getPose().getTranslation()
                .getDistance(botPoseEstimate.pose.getTranslation());

        ChassisSpeeds ChassisSpeeds = TunerConstants.DriveTrain.getCurrentRobotChassisSpeeds();
        double rotationSpeed = ChassisSpeeds.omegaRadiansPerSecond;
        double xSpeed = ChassisSpeeds.vxMetersPerSecond;
        double ySpeed = ChassisSpeeds.vyMetersPerSecond;
        double translationalSpeed = Math.hypot(xSpeed, ySpeed);
        
        if ((Math.abs(rotationSpeed) < KALMAN_ROTATION_MAX_RATE) && 
            (Math.abs(translationalSpeed) < KALMAN_MAX_SPEED) && 
            (botPoseEstimate.avgTagDist < KALMAN_APRILTAG_MAX_RANGE || botPoseEstimate.tagCount > 2)
        ) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (botPoseEstimate.tagCount >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (botPoseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (botPoseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            TunerConstants.TargetingDrivetrain.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Math.toRadians(degStds)));
            TunerConstants.TargetingDrivetrain.addVisionMeasurement(botPoseEstimate.pose,
                    Timer.getFPGATimestamp() - botPoseEstimate.latency / 1000.0);
            PPLibTelemetry.setTargetPose(botPoseEstimate.pose);
        }
    }


    //#region Getters
    public double getAz() {
        if (TunerConstants.DriveTrain.getSideMode() == SideMode.RED) {
            return rolloverConversion_radians(this.targetAz + (Math.PI / 2));
        } else if (TunerConstants.DriveTrain.getSideMode() == SideMode.BLUE) {
            return rolloverConversion_radians(this.targetAz - (Math.PI / 2));
        } else {
            return this.targetAz;
        }
    }

    public double getDistance_XY(){
        return this.distance_XY;
    }

    public double getDistance_XY_average(){
        return this.distance_XY_Average;
    }

    public double getEl() {
        return this.targetEl;
    }

    public double getLeftShooterSpeed(){
        return this.leftShooterSpeed;
    }

    public double getRightShooterSpeed(){
        return this.rightShooterSpeed;
    }

    //getTrapAz will use botPos to determine which 'region' the bot is in and return the angle needed to face the trap in radians
    //The angle returned is already calculated with the red/blue gyro zero in mind.
    public double getTrapAz() {
        if (TunerConstants.DriveTrain.getSideMode() == SideMode.BLUE) { //+PI/2
            if (botPos[0] < 5.7706260) {
                if (botPos[1] > 4.1056560) {
                    return ((2 / 6) * Math.PI); //60 degrees
                } else {
                    return ((-2 / 6) * Math.PI); //-60 degrees
                }
            } else {
                return (Math.PI); //180 degrees
            }
        } else { // SideMode.RED //-PI/2
            if (botPos[0] > 11.7196870) {
                if (botPos[1] > 4.1056560)
                    return ((-2 / 6) * Math.PI); //-60 degrees
                else {
                    return ((2 / 6) * Math.PI); //60 degrees
                }
            } else {
                return (Math.PI); //180 degrees
            }
        }
    }

    public double getBotPosX() {
        return botPos[0];
    }

    public double getBotPosY() {
        return botPos[1];
    }
    //#endregion
    //#endregion Non-Static
}
