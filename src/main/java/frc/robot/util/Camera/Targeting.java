package frc.robot.util.Camera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Interpolable.InterpolatingDouble;

public class Targeting {
    //#region Static Constants
    // Cartesian Coordinates of Targets
    // Origin is BOTTOM LEFT OF FIELD
    // X, Y, Z (Inches)
    private static final double[] blueSpeaker =     { -0.2000000, 5.5478680, 2.0432395 };
    private static final double[] blueAmp =         { 14.7007580, 8.2042000, 0.8891270 };
    private static final double[] blueTrap1 =       { 4.6413420, 4.4983400, 1.6414750 };
    private static final double[] blueTrap2 =       { 4.6413420, 3.7132260, 1.6414750 };
    private static final double[] blueTrap3 =       { 5.3207920, 4.1051480, 1.6414750 };
    private static final double[] redSpeaker =      { 16.700000, 5.5478680, 2.0432395 };
    private static final double[] redAmp =          { 1.8415000, 8.2042000, 0.8891270 };
    private static final double[] redTrap1 =        { 11.9047260, 3.7132260, 1.6414750 };
    private static final double[] redTrap2 =        { 11.9047260, 4.4983400, 1.6414750 };
    private static final double[] redTrap3 =        { 11.2201960, 4.1051480, 1.6414750 };
    private static final double[] centerOfField =   { 5.3207920, 4.1067482, 0.0000000 };

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
    //#endregion

    public enum Target {
        BLUESPEAKER(blueSpeaker, blueSpeakerID), BLUAMP(blueAmp, blueAmpID), BLUTRAP1(blueTrap1, blueTrap1ID),
        BLUTRAP2(blueTrap2, blueTrap2ID), BLUTRAP3(blueTrap3, blueTrap3ID), REDSPEAKER(redSpeaker, redSpeakerID),
        REDAMP(redAmp, redAmpID), REDTRAP1(redTrap1, redTrap1ID), REDTRAP2(redTrap2, redTrap2ID),
        REDTRAP3(redTrap3, redTrap3ID), NONE(centerOfField, nullID);

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

    //#region Constructors
    // Targeting Constructor, Limelight
    public Targeting(String limelightHostname, boolean isOdometry) {
        this.limelightHostname = "-" + limelightHostname;
        this.limelight = new Limelight(limelightHostname);
        this.isOdometry = isOdometry;
    }

    // Targeting Constructor, Odometry
    public Targeting(boolean isOdometry) {
        this.isOdometry = isOdometry;
        this.limelight = new Limelight();
    }
    //#endregion

    //#region Static
    private static double[] targetPos = centerOfField;
    private static double targetID = nullID;
    private static Target target = Target.NONE;

    public static void setTarget(Target toTarget) {
        target = toTarget;
        targetPos = target.getPos();
        targetID = target.getID();
    }
    //#region Static Getters
    public static Target getTarget() {
        return target;
    }

    public static double getTargetID() {
        return targetID;
    }
    //#endregion Static Getters
    //#endregion Static

    //#region Non-Static
    private double[] botPos = centerOfField;
    private double targetAz = 0;
    private double targetEl = 0;
    private double leftShooterSpeed = 0;
    private double rightShooterSpeed = 0;
    private double distance_XY = 0.0;
    private String limelightHostname;
    private boolean isOdometry = false;
    private final Limelight limelight;

    public void updateBotPos() {
        if (isOdometry) {
            botPos[0] = TunerConstants.DriveTrain.getPose().getX();
            botPos[1] = TunerConstants.DriveTrain.getPose().getY();
            botPos[2] = 0.0;
        } else {
            try {
                botPos = limelight.getBotPose();

                if (botPos.length < 3) {
                    System.err.println(botPos.length);
                    System.err.println("Targeting Error: " + limelightHostname);
                    System.err.println("Targeting Error: updateBotPos failed, Limelight gave BAD data, zeroed botPos");
                    botPos = new double[] { 0, 0, 0, 0, 0, 0 };
                }

            } catch (Exception e) {
                System.err.println("Targeting Error: " + limelightHostname);
                System.err.println("Targeting Error: updateBotPos failed to get Limelight Data");
                botPos = new double[] { 0, 0, 0, 0, 0, 0 };
            }
        }
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
        if (!this.hasTarget()) {
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
            {
            }
            ; // Nothing is done if both delta_X and delta_Y are 0, targetAz is not updated.
        } else { // No cases ran, should never occur
            {
            }
            ;
        }

        // Elevation Trig
        delta_X += TunerConstants.DriveTrain.getFieldRelativeVelocites().vxMetersPerSecond * Constants.SHOOT_TIME;
        delta_Y += TunerConstants.DriveTrain.getFieldRelativeVelocites().vyMetersPerSecond * Constants.SHOOT_TIME;
        distance_XY = Math.hypot(delta_X, delta_Y);
        SmartDashboard.putNumber("Distance2Target", (distance_XY / 0.0254) / 12.0);
        if (distance_XY != 0) {
            this.targetEl = Constants.kLiftAngleMap
                    .getInterpolated(new InterpolatingDouble((distance_XY / 0.0254) / 12.0)).value;
            this.leftShooterSpeed = Constants.kLeftShooterMap
                    .getInterpolated(new InterpolatingDouble((distance_XY / 0.0254) / 12.0)).value;
            this.rightShooterSpeed = Constants.kRightShooterMap
                    .getInterpolated(new InterpolatingDouble((distance_XY / 0.0254) / 12.0)).value;
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

    //#region Getters
    public double getAz() {
        if (RobotContainer.getInstance().getSide() == SideMode.RED) {
            return rolloverConversion_radians(this.targetAz + (Math.PI / 2));
        } else if (RobotContainer.getInstance().getSide() == SideMode.BLUE) {
            return rolloverConversion_radians(this.targetAz - (Math.PI / 2));
        } else {
            return this.targetAz;
        }
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
        if (RobotContainer.getInstance().getSide() == SideMode.BLUE) { //+PI/2
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
