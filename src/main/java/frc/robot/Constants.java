package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.Interpolable.InterpolatingDouble;
import frc.robot.util.Interpolable.InterpolatingTreeMap;

public class Constants {

    public static final boolean debug = false;

    //#region Intake
        public static final double INTAKE_GEAR_RATIO = 12.0/30.0;

        public static final int FRONT_INTAKE_ID = 10;
        public static final int TOP_INTAKE_ID = 11;
        public static final int BOTTOM_INTAKE_ID = 12;

        public static final double UNDER_INTAKE_RPM = 3000.0;
        public static final double UNDER_INTAKE_EJECT_RPM = -3000.0;
        public static final double FRONT_IN_INTAKE_RPM = 2000.0;
        public static final double FRONT_EJECT_INTAKE_RPM = -1000.0;
        public static final double BACK_IN_INTAKE_RPM = 1500.0;
        public static final double BACK_EJECT_INTAKE_RPM = -1000.0;
        public static final double UP_INTAKE_RPM = 1500.0;
        public static final double UP_INTAKE_EJECT_RPM = -1000.0;
        public static final double SPIT_RPM = 1500.0;
        public static final double SLURP_INTAKE_RPM = 1000.0;
    //#endregion

    //#region Shooter
        public static final double SHOOTER_GEAR_RATIO = 1.0;
        public static final double KICKER_GEAR_RATIO = 2.5;

        public static final int SHOOTER_RIGHT_ID = 20;
        public static final int SHOOTER_LEFT_ID = 21;

        public static final int SHOOTER_KICKER_ID = 22;

        public static final double KICKER_INTAKE_RPM = 800.0;
        public static final double KICKER_EJECT_RPM = -800.0;
        public static final double KICKER_SCORE_RPM = 1000.0;

        public static final double LEFT_SCORE_RPM = 5000.0;
        public static final double RIGHT_SCORE_RPM = 3000.0;

        public static final double SHOOT_TIME = 0.25;
    //#endregion    

    //#region Lift
        public static final double LIFT_GEAR_RATIO = (60.0/16.0)*(72.0/16.0)*(70.0/18.0);

        public static final int LIFT_MOTOR_ID = 30;
        public static final int LIFT_CANCODER_ID = 5;

        public static final double LIFT_MAX_DEGREES = 70.0;
        public static final double LIFT_MIN_DEGREES = 18.6;
        public static final double LIFT_INTAKE_DEGREES = 30.0;
        public static final double LIFT_START_DEGREES = 18.0;

        public static final double CORNER_FIELD_RELATIVE_ANGLE_DEG = 43.0;
        public static double kCornerLiftAngleFixed = 60.0;
        public static double kCornerRightShooterFixed = 2000.0;
        public static double kCornerLeftShooterFixed = 3000.0;

        public static final double CENTER_FIELD_RELATIVE_ANGLE_DEG = 43.0;
        public static double kCenterLiftAngleFixed = 50.0;
        public static double kCenterRightShooterFixed = 2200.0-200.0;
        public static double kCenterLeftShooterFixed = 3400.0-200.0;

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLiftAngleMap = new InterpolatingTreeMap<>();
        static {
            kLiftAngleMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(62.0));
            kLiftAngleMap.put(new InterpolatingDouble(4.7), new InterpolatingDouble(62.0));
            kLiftAngleMap.put(new InterpolatingDouble(8.0), new InterpolatingDouble(46.0));
            kLiftAngleMap.put(new InterpolatingDouble(12.0), new InterpolatingDouble(33.0));
            kLiftAngleMap.put(new InterpolatingDouble(16.0), new InterpolatingDouble(26.0));
            kLiftAngleMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(23.0+0.5));
            kLiftAngleMap.put(new InterpolatingDouble(24.0), new InterpolatingDouble(20.25));
            kLiftAngleMap.put(new InterpolatingDouble(28.0), new InterpolatingDouble(18.75));
            kLiftAngleMap.put(new InterpolatingDouble(32.0), new InterpolatingDouble(18.5));
            kLiftAngleMap.put(new InterpolatingDouble(40.0), new InterpolatingDouble(19.0));
        }

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kAutoAimOffset = new InterpolatingTreeMap<>();
        static {
            kAutoAimOffset.put(new InterpolatingDouble(3.0), new InterpolatingDouble(0.0));
            kAutoAimOffset.put(new InterpolatingDouble(4.7), new InterpolatingDouble(0.0));
            kAutoAimOffset.put(new InterpolatingDouble(12.0), new InterpolatingDouble(-3.0)); // -5.0
            kAutoAimOffset.put(new InterpolatingDouble(16.0), new InterpolatingDouble(-1.0));//0.0
            kAutoAimOffset.put(new InterpolatingDouble(20.0), new InterpolatingDouble(-1.0)); // -2.0
            kAutoAimOffset.put(new InterpolatingDouble(24.0), new InterpolatingDouble(-1.0)); // -2.0
            kAutoAimOffset.put(new InterpolatingDouble(28.0), new InterpolatingDouble(-1.0)); // -2.0
            kAutoAimOffset.put(new InterpolatingDouble(32.0), new InterpolatingDouble(-1.0));
            kAutoAimOffset.put(new InterpolatingDouble(40.0), new InterpolatingDouble(-1.0));
        }
 
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kRightShooterMap = new InterpolatingTreeMap<>();
        static {
            kRightShooterMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(2200.0));
            kRightShooterMap.put(new InterpolatingDouble(4.7), new InterpolatingDouble(2200.0));
            kRightShooterMap.put(new InterpolatingDouble(8.0), new InterpolatingDouble(2200.0));
            // kRightShooterMap.put(new InterpolatingDouble(12.0), new InterpolatingDouble(2700.0));
            kRightShooterMap.put(new InterpolatingDouble(16.0), new InterpolatingDouble(3000.0));
            kRightShooterMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(3000.0));
            kRightShooterMap.put(new InterpolatingDouble(24.0), new InterpolatingDouble(3500.0));
            kRightShooterMap.put(new InterpolatingDouble(28.0), new InterpolatingDouble(3500.0));
            kRightShooterMap.put(new InterpolatingDouble(40.0), new InterpolatingDouble(3500.0));
        }

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLeftShooterMap = new InterpolatingTreeMap<>();
        static {
            kLeftShooterMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(2700.0));
            kLeftShooterMap.put(new InterpolatingDouble(4.7), new InterpolatingDouble(2700.0));
            kLeftShooterMap.put(new InterpolatingDouble(8.0), new InterpolatingDouble(2700.0));
            // kLeftShooterMap.put(new InterpolatingDouble(12.0), new InterpolatingDouble(3700.0));
            kLeftShooterMap.put(new InterpolatingDouble(16.0), new InterpolatingDouble(5000.0));
            kLeftShooterMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(5000.0));
            kLeftShooterMap.put(new InterpolatingDouble(24.0), new InterpolatingDouble(5800.0));
            kLeftShooterMap.put(new InterpolatingDouble(28.0), new InterpolatingDouble(5800.0));
            kLeftShooterMap.put(new InterpolatingDouble(40.0), new InterpolatingDouble(5800.0));
        }

        // public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPassLiftAngleMap = new InterpolatingTreeMap<>();
        // static{
        //     kPassLiftAngleMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(60.0));
        //     kPassLiftAngleMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(60.0));
        //     kPassLiftAngleMap.put(new InterpolatingDouble(35.0), new InterpolatingDouble(60.0)); // 42.0
        //     kPassLiftAngleMap.put(new InterpolatingDouble(35.0), new InterpolatingDouble(60.0)); // 42.0
        // }

        // public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPassRightShooterMap = new InterpolatingTreeMap<>();
        // static{
        //     kPassRightShooterMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(2000.0));
        //     kPassRightShooterMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(2000.0));
        //     kPassRightShooterMap.put(new InterpolatingDouble(35.0), new InterpolatingDouble(2000.0)); // 2100.0
        //     kPassRightShooterMap.put(new InterpolatingDouble(35.0), new InterpolatingDouble(2000.0)); // 2100.0
        // }

        // public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPassLeftShooterMap = new InterpolatingTreeMap<>();
        // static{
        //     kPassLeftShooterMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(3000.0));
        //     kPassLeftShooterMap.put(new InterpolatingDouble(20.0), new InterpolatingDouble(3000.0));
        //     kPassLeftShooterMap.put(new InterpolatingDouble(36.0), new InterpolatingDouble(3000.0)); // 3800.0
        //     kPassLeftShooterMap.put(new InterpolatingDouble(36.0), new InterpolatingDouble(3000.0)); // 38000.0
        // }

        // public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kCenterLiftAngleMap = new InterpolatingTreeMap<>();
        // static{
        //     kCenterLiftAngleMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(50.0));
        //     kCenterLiftAngleMap.put(new InterpolatingDouble(50.0), new InterpolatingDouble(50.0));
        // }

        // public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kCenterRightShooterMap = new InterpolatingTreeMap<>();
        // static{
        //     kCenterRightShooterMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(2200.0));
        //     kCenterRightShooterMap.put(new InterpolatingDouble(50.0), new InterpolatingDouble(2200.0)); 
        // }

        // public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kCenterLeftShooterMap = new InterpolatingTreeMap<>();
        // static{
        //     kCenterLeftShooterMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(3200.0));
        //     kCenterLeftShooterMap.put(new InterpolatingDouble(50.0), new InterpolatingDouble(3200.0));
        // }

        public static final double FENDER_SHOT_ANGLE = 60.0;
    //#endregion    

    //#region Elevator
        public static final double ELEVATOR_GEAR_RATIO = (54.0/34.0)*(54.0/11.0);
        public static final double ELEVATOR_PULLEY_DIAMETER = 1.25;

        public static final int ELEVATOR_MOTOR_ID = 40;

        public static final double ELEVATOR_MAX_INCHES = 20.625;//+1
        public static final double ELEVATOR_MIN_INCHES = 0.0;

        public static final double ELEVATOR_AUTO_ZERO_SPEED = -0.1;
        public static final double ELEVATOR_AUTO_ZERO_MOTOR_CURRENT = 1.0;

        public static final double AMP_SCORE_INCHES = 12.0;
        public static final double TRAP_SCORE_INCHES = ELEVATOR_MAX_INCHES;
    //#endregion

    //#region Amp
        public static final double AMP_GEAR_RATIO = (30.0/12.0);

        public static final int AMP_MOTOR_ID = 50;
        public static final int AMP_SENSOR_PORT = 1;

        public static final double AMP_SCORE_RPM = -3000.0;
        public static final double AMP_LOAD_RPM = 400.0;
        public static final double AMP_INTAKE_RPM = 800.0;
        public static final double AMP_EJECT_RPM = -800.0;
        public static final double TRAP_SCORE_RPM = -800.0;
    //#endregion

    //#region Climber
        public static final double CLIMBER_GEAR_RATIO = (70.0/18.0)*(60.0/11.0);
        public static final double CLIMBER_PULLEY_DIAMETER = 1.25;

        public static final int CHAIN_SENSOR_PORT = 2;

        public static final int CLIMBER_LEFT_ID = 60;
        public static final int CLIMBER_RIGHT_ID = 61;

        public static final double CLIMBER_MAX_INCHES = 4.875
        ; // 19.0
        public static final double CLIMBER_MIN_INCHES = -14.25;
        public static final double CLIMBER_AUTO_ZERO_SPEED = -0.1;
        public static final double CLIMBER_AUTO_ZERO_MOTOR_CURRENT = 5.0;

        public static final double CLIMBER_MIN_PERCENT_BUS = 0.1;
   //#endregion

    //drivetrain
    public static final double MaxSpeed = 6.21; // 6 meters per second desired top speed
    public static final double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    //camera
    public static final double MAX_NOTE_DISTANCE = 5.0/3.281; //feet to meters

    public static final class SnapConstants {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kTimeout = 0.25;

        public static final double kEpsilon = 5.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond,
                2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class SnapAutonConstants {
        public static final double kP = 1.0;
        public static final double kI = 0.05;
        public static final double kD = 0.0;
        public static final double kTimeout = 0.25;

        public static final double kEpsilon = 1.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond,
                2);

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionAlignConstants {
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.10;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 5.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public enum SwerveCardinal {
        NONE(0),

        AMP(90),
        SOURCE(-60);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

}
