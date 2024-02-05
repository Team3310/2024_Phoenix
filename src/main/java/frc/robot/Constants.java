package frc.robot;

public class Constants {
    //#region Intake
        public static final double INTAKE_GEAR_RATIO = 12.0/30.0;

        public static final int FRONT_INTAKE_ID = 0;
        public static final int TOP_INTAKE_ID = 3;
        public static final int BOTTOM_INTAKE_ID = 5;

        public static final double UNDER_INTAKE_RPM = 3000.0;
        public static final double IN_INTAKE_RPM = 2000.0;
        public static final double UP_INTAKE_RPM = 2000.0;
        public static final double SPIT_RPM = 1500.0;
        public static final double SLURP_INTAKE_RPM = 1000.0;
    //#endregion

    //#region Shooter
        public static final double SHOOTER_GEAR_RATIO = 1.0;

        public static final int SHOOTER_RIGHT_MASTER_ID = 40;
        public static final int SHOOTER_RIGHT_SLAVE_ID = 1000;

        public static final int SHOOTER_LEFT_MASTER_ID = 41;
        public static final int SHOOTER_LEFT_SLAVE_ID = 1001;

        public static final int SHOOTER_KICKER_ID = 42;

        public static final double KICKER_INTAKE_RPM = 1500.0;
    //#endregion    

    //#region Hood
        public static final double HOOD_GEAR_RATIO = (60.0/16.0)*(66.0/20.0)*(72.0/16.0);

        public static final int HOOD_ID = 50;

        public static final double HOOD_MAX_DEGREES = 75.0;
        public static final double HOOD_MIN_DEGREES = 20.0;
        public static final double HOOD_START_DEGREES = 20.0;
    //#endregion    

    //#region Elevator
        public static final double ELEVATOR_GEAR_RATIO = 1.0;
        public static final double ELEVATOR_PULLY_DIAMTER = 1.0;

        public static final int ELEVATOR_MASTER_ID = 61;
        public static final int ELEVATOR_SLAVE_ID = 62;

        public static final double ELEVATOR_MAX_INCHES = 45.0;
        public static final double ELEVATOR_MIN_INCHES = 0.0;
    //#endregion

    //drivetrain
    public static final double MaxSpeed = 6; // 6 meters per second desired top speed
    public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    
    //camera
    public static final double MAX_NOTE_DISTANCE = 5.0/3.281; //feet to meters
}
