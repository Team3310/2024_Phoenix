package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {

    public static final String rioCANbusName = "rio";

    //#region Intake
        public static final double INTAKE_GEAR_RATIO = 12.0/30.0;

        public static final int FRONT_INTAKE_ID = 10;
        public static final int TOP_INTAKE_ID = 11;
        public static final int BOTTOM_INTAKE_ID = 12;

        public static final double UNDER_INTAKE_RPM = 3000.0;
        public static final double IN_INTAKE_RPM = 1000.0;
        public static final double UP_INTAKE_RPM = 1000.0;
        public static final double SPIT_RPM = 1500.0;
        public static final double SLURP_INTAKE_RPM = 1000.0;
    //#endregion

    //#region Shooter
        public static final double SHOOTER_GEAR_RATIO = 1.0;
        public static final double KICKER_GEAR_RATIO = 2.5;

        public static final int SHOOTER_RIGHT_MASTER_ID = 20;
        public static final int SHOOTER_RIGHT_SLAVE_ID = 1000;

        public static final int SHOOTER_LEFT_MASTER_ID = 21;
        public static final int SHOOTER_LEFT_SLAVE_ID = 1001;

        public static final int SHOOTER_KICKER_ID = 22;

        public static final double KICKER_INTAKE_RPM = 1000.0;
        public static final double KICKER_SCORE_RPM = 1000.0;

        public static final double LEFT_SCORE_RPM = 5000.0;
        public static final double RIGHT_SCORE_RPM = 3000.0;
    //#endregion    

    //#region Lift
        public static final double LIFT_GEAR_RATIO = (60.0/16.0)*(72.0/16.0)*(70.0/18.0);

        public static final int LIFT_MOTOR_ID = 30;
        public static final int LIFT_CANCODER_ID = 5;

        public static final double LIFT_MAX_DEGREES = 75.0;
        public static final double LIFT_MIN_DEGREES = 18.0;
        public static final double LIFT_START_DEGREES = 18.0;
    //#endregion    

    //#region Elevator
        public static final double ELEVATOR_GEAR_RATIO = (54.0/34.0)*(54.0/11.0);
        public static final double ELEVATOR_PULLEY_DIAMETER = 1.25;

        public static final int ELEVATOR_MOTOR_ID = 40;

        public static final double ELEVATOR_MAX_INCHES = 20.625;
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

        public static final double AMP_SCORE_RPM = -1500.0;
        public static final double AMP_LOAD_RPM = 200.0;
        public static final double AMP_INTAKE_RPM = 800.0;
    //#endregion

    //#region Climber
        public static final double CLIMBER_GEAR_RATIO = (70.0/18.0)*(60.0/11.0);
        public static final double CLIMBER_PULLEY_DIAMETER = 1.25;

        public static final int CLIMBER_LEFT_ID = 60;
        public static final int CLIMBER_RIGHT_ID = 61;

        public static final double CLIMBER_MAX_INCHES = 22.0;
        public static final double CLIMBER_MIN_INCHES = 0.0;
        public static final double CLIMBER_AUTO_ZERO_SPEED = -0.1;
        public static final double CLIMBER_AUTO_ZERO_MOTOR_CURRENT = 5.0;
   //#endregion

    //drivetrain
    public static final double MaxSpeed = 6; // 6 meters per second desired top speed
    public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    //camera
    public static final double MAX_NOTE_DISTANCE = 5.0/3.281; //feet to meters

    public static final class SnapConstants {
        public static final double kP = 5.0;
        public static final double kI = 0;
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
        public static final double kP = 6.37;
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


}
