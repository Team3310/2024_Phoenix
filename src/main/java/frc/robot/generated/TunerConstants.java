// package frc.robot.generated;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

// import edu.wpi.first.math.util.Units;
// import frc.robot.Subsystems.Drivetrain;

// public class TunerConstants {
//     // Both sets of gains need to be tuned to your individual robot.

//     // The steer motor uses any SwerveModule.SteerRequestType control request with the
//     // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
//     private static final Slot0Configs steerGains = new Slot0Configs()
//         .withKP(100).withKI(0).withKD(0.2)
//         .withKS(0).withKV(1.5).withKA(0);
//     // When using closed-loop control, the drive motor uses the control
//     // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
//     private static final Slot0Configs driveGains = new Slot0Configs()
//         .withKP(3).withKI(0).withKD(0)
//         .withKS(0).withKV(0).withKA(0);

//     // The closed-loop output type to use for the steer motors;
//     // This affects the PID/FF gains for the steer motors
//     private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
//     // The closed-loop output type to use for the drive motors;
//     // This affects the PID/FF gains for the drive motors
//     private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

//     // The stator current at which the wheels start to slip;
//     // This needs to be tuned to your individual robot
//     private static final double kSlipCurrentA = 300.0;

//     // Theoretical free speed (m/s) at 12v applied output;
//     // This needs to be tuned to your individual robot
//     public static final double kSpeedAt12VoltsMps = 6.21;

//     // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
//     // This may need to be tuned to your individual robot
//     private static final double kCoupleRatio = 2.6666666666666665;

//     private static final double kDriveGearRatio = 5.142857142857143;
//     private static final double kSteerGearRatio = 11.314285714285715;
//     private static final double kWheelRadiusInches = 2;

//     private static final boolean kSteerMotorReversed = true;
//     private static final boolean kInvertLeftSide = false;
//     private static final boolean kInvertRightSide = true;

//     public static final String kCANbusName = "Drivetrain";
//     private static final int kPigeonId = 0;


//     // These are only used for simulation
//     private static final double kSteerInertia = 0.00001;
//     private static final double kDriveInertia = 0.001;
//     // Simulated voltage necessary to overcome friction
//     private static final double kSteerFrictionVoltage = 0.25;
//     private static final double kDriveFrictionVoltage = 0.25;

//     private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
//             .withPigeon2Id(kPigeonId)
//             .withCANbusName(kCANbusName);

//     private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
//             .withDriveMotorGearRatio(kDriveGearRatio)
//             .withSteerMotorGearRatio(kSteerGearRatio)
//             .withWheelRadius(kWheelRadiusInches)
//             .withSlipCurrent(kSlipCurrentA)
//             .withSteerMotorGains(steerGains)
//             .withDriveMotorGains(driveGains)
//             .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
//             .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
//             .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
//             .withSteerInertia(kSteerInertia)
//             .withDriveInertia(kDriveInertia)
//             .withSteerFrictionVoltage(kSteerFrictionVoltage)
//             .withDriveFrictionVoltage(kDriveFrictionVoltage)
//             .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
//             .withCouplingGearRatio(kCoupleRatio)
//             .withSteerMotorInverted(kSteerMotorReversed);


//     // Front Left
//     private static final int kFrontLeftDriveMotorId = 1;
//     private static final int kFrontLeftSteerMotorId = 8;
//     private static final int kFrontLeftEncoderId = 0;
//     private static final double kFrontLeftEncoderOffset = 0.0361328125;

//     private static final double kFrontLeftXPosInches = 11.5;
//     private static final double kFrontLeftYPosInches = 11.5;

//     // Front Right
//     private static final int kFrontRightDriveMotorId = 9;
//     private static final int kFrontRightSteerMotorId = 2;
//     private static final int kFrontRightEncoderId = 3;
//     private static final double kFrontRightEncoderOffset = -0.498046875;

//     private static final double kFrontRightXPosInches = 11.5;
//     private static final double kFrontRightYPosInches = -11.5;

//     // Back Left
//     private static final int kBackLeftDriveMotorId = 10;
//     private static final int kBackLeftSteerMotorId = 6;
//     private static final int kBackLeftEncoderId = 1;
//     private static final double kBackLeftEncoderOffset = 0.01904296875;

//     private static final double kBackLeftXPosInches = -11.5;
//     private static final double kBackLeftYPosInches = 11.5;

//     // Back Right
//     private static final int kBackRightDriveMotorId = 7;
//     private static final int kBackRightSteerMotorId = 4;
//     private static final int kBackRightEncoderId = 2;
//     private static final double kBackRightEncoderOffset = -0.401611328125;

//     private static final double kBackRightXPosInches = -11.5;
//     private static final double kBackRightYPosInches = -11.5;


//     private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
//             kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
//     private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
//             kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
//     private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
//             kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
//     private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
//             kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

//     public static final Drivetrain DriveTrain = new Drivetrain(DrivetrainConstants, FrontLeft,
//             FrontRight, BackLeft, BackRight);
// }

//Little swerve bot non-kracken
package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Drivetrain;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(10).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.73;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 13.371428571428572;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = true;
    private static final boolean kInvertRightSide = false;

    public static final String kCANbusName = "rio";
    private static final int kPigeonId = 0;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 8;
    private static final int kFrontLeftEncoderId = 0;
    private static final double kFrontLeftEncoderOffset = 0.1806640625;

    private static final double kFrontLeftXPosInches = 11.9375;
    private static final double kFrontLeftYPosInches = 12.6875;

    // Front Right
    private static final int kFrontRightDriveMotorId = 9;
    private static final int kFrontRightSteerMotorId = 2;
    private static final int kFrontRightEncoderId = 3;
    private static final double kFrontRightEncoderOffset = 0.38671875;

    private static final double kFrontRightXPosInches = 11.9375;
    private static final double kFrontRightYPosInches = -12.6875;

    // Back Left
    private static final int kBackLeftDriveMotorId = 0;
    private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 1;
    private static final double kBackLeftEncoderOffset = 0.125732421875;

    private static final double kBackLeftXPosInches = -11.9375;
    private static final double kBackLeftYPosInches = 12.6875;

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 2;
    private static final double kBackRightEncoderOffset = -0.2099609375;

    private static final double kBackRightXPosInches = -11.9375;
    private static final double kBackRightYPosInches = -12.6875;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), !kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), !kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), !kInvertRightSide);

    public static final Drivetrain DriveTrain = new Drivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
