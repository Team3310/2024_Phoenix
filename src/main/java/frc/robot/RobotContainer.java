// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Hood.SetHoodAngle;
import frc.robot.Commands.Intake.BeltSideSpit;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeSlurp;
import frc.robot.Commands.Intake.IntakeUnder;
import frc.robot.Commands.Intake.IntakeUp;
import frc.robot.Commands.Intake.StopIntake;
import frc.robot.Commands.Shooter.FeederLoadCommand;
import frc.robot.Commands.Shooter.FeederShootCommand;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.SetShooterKickerRPM;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Hood;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.DriverReadout;
import frc.robot.util.SideChooser;
import frc.robot.util.SpotChooser;
import frc.robot.util.SideChooser.SideMode;
import frc.robot.util.SpotChooser.SpotMode;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final Intake intake = Intake.getInstance();
  public final Shooter shooter = Shooter.getInstance();
  public final Hood hood = Hood.getInstance();
  // public final Flicker flicker = Flicker.getInstance();

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final AutonomousChooser autonomousChooser = new AutonomousChooser();
  private final SideChooser sideChooser = new SideChooser();
  private final SpotChooser spotChooser = new SpotChooser();

  private final DriverReadout driverReadout;

  private static RobotContainer instance;

  private void configureBindings() {
    // drivetrain.setJoystick(joystick);
    // flicker.setDefaultCommand(
    //   new FlickerCommand(flicker, operatorController)
    // );



    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverController.a().onTrue(new SetDriveMode(DriveMode.LIMELIGHT));
    driverController.b().onTrue(new SetDriveMode(DriveMode.JOYSTICK));

    // operatorController.a().onTrue(new InstantCommand(()->flicker.setPosition(0.0)));
    // operatorController.y().onTrue(new InstantCommand(()->flicker.setPosition(1.0)));


    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);


    driverController.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverController.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  
    operatorController.a().onTrue(/*all under*/ new IntakeUnder());
    operatorController.b().onTrue(/*in robot*/ new IntakeAuton());
    operatorController.y().onTrue(/*up*/ new IntakeUp());
    operatorController.x().onTrue(/*stop all*/ new StopIntake());
    operatorController.pov(0).onTrue(new IntakeSlurp());
    operatorController.pov(90).onTrue(new BeltSideSpit(false));
    operatorController.pov(270).onTrue(new BeltSideSpit(true));

    SmartDashboard.putData("Right Side RPM 0", new SetRightShooterRPM(shooter, 0.0));
    SmartDashboard.putData("Right Side RPM 50", new SetRightShooterRPM(shooter, 0.0));
    SmartDashboard.putData("Right Side RPM 500", new SetRightShooterRPM(shooter,500));
    SmartDashboard.putData("Right Side RPM 1000", new SetRightShooterRPM(shooter,1000));
    SmartDashboard.putData("Right Side RPM 2000", new SetRightShooterRPM(shooter,2000));
    SmartDashboard.putData("Right Side RPM 3000", new SetRightShooterRPM(shooter,3000));
    SmartDashboard.putData("Right Side RPM 4000", new SetRightShooterRPM(shooter,4000));
    SmartDashboard.putData("Right Side RPM 5000", new SetRightShooterRPM(shooter,5000));
    SmartDashboard.putData("Right Side RPM 6000", new SetRightShooterRPM(shooter,6000));

    SmartDashboard.putData("Left Side RPM 0", new SetLeftShooterRPM(shooter, 0.0));
    SmartDashboard.putData("Left Side RPM 50", new SetLeftShooterRPM(shooter, -50));
    SmartDashboard.putData("Left Side RPM 500", new SetLeftShooterRPM(shooter, -500));
    SmartDashboard.putData("Left Side RPM 1000", new SetLeftShooterRPM(shooter, -1000));
    SmartDashboard.putData("Left Side RPM 2000", new SetLeftShooterRPM(shooter, -2000));
    SmartDashboard.putData("Left Side RPM 3000", new SetLeftShooterRPM(shooter, -3000));
    SmartDashboard.putData("Left Side RPM 4000", new SetLeftShooterRPM(shooter, -4000));
    SmartDashboard.putData("Left Side RPM 5000", new SetLeftShooterRPM(shooter, -5000));
    SmartDashboard.putData("Left Side RPM 6000", new SetLeftShooterRPM(shooter, -6000));

    SmartDashboard.putData("Kicker RPM 0", new SetShooterKickerRPM(shooter, 0.0));
    SmartDashboard.putData("Kicker RPM 50", new SetShooterKickerRPM(shooter, 50));
    SmartDashboard.putData("Kicker RPM 500", new SetShooterKickerRPM(shooter, 500));
    SmartDashboard.putData("Kicker RPM 1000", new SetShooterKickerRPM(shooter, 1000));
    SmartDashboard.putData("Kicker RPM 2000", new SetShooterKickerRPM(shooter, 2000));
    SmartDashboard.putData("Kicker RPM 3000", new SetShooterKickerRPM(shooter, 3000));
    SmartDashboard.putData("Kicker RPM 4000", new SetShooterKickerRPM(shooter, 4000));
    SmartDashboard.putData("Kicker RPM 5000", new SetShooterKickerRPM(shooter, 5000));
    SmartDashboard.putData("Kicker RPM 6000", new SetShooterKickerRPM(shooter, 6000));

    SmartDashboard.putData("Hood Angle 15", new SetHoodAngle(hood, 15));
    SmartDashboard.putData("Hood Angle 20", new SetHoodAngle(hood, 20));
    SmartDashboard.putData("Hood Angle 22", new SetHoodAngle(hood, 22));
    SmartDashboard.putData("Hood Angle 25", new SetHoodAngle(hood, 25));
    SmartDashboard.putData("Hood Angle 30", new SetHoodAngle(hood, 30));
    SmartDashboard.putData("Hood Angle 35", new SetHoodAngle(hood, 35));
    SmartDashboard.putData("Hood Angle 40", new SetHoodAngle(hood, 40));
    SmartDashboard.putData("Hood Angle 45", new SetHoodAngle(hood, 45));
    SmartDashboard.putData("Hood Angle 70", new SetHoodAngle(hood, 70));
    SmartDashboard.putData("Zero Hood", new InstantCommand(()->hood.setHoodZero(90)));

    SmartDashboard.putData("Kicker Load", new FeederLoadCommand(shooter));
    SmartDashboard.putData("Kicker Shoot", new FeederShootCommand(shooter));
  }

  public RobotContainer() {
    instance = this;

    driverReadout = new DriverReadout(this);

    // CommandScheduler.getInstance().registerSubsystem(intake);
    CommandScheduler.getInstance().registerSubsystem(drivetrain);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autonomousChooser.getCommand(instance);
  }

  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }

  public SideMode getSide() {
    return sideChooser.getSide();
  }

  public static RobotContainer getInstance(){
    return instance;
  }

  public SpotMode getSpot(){
    return spotChooser.getSpot();
  }

  public SendableChooser<SideMode> getSideChooser() {
    return sideChooser.getSendableChooser();
  }

  public SendableChooser<SpotMode> getSpotChooser() {
    return spotChooser.getSendableChooser();
  }

  public Drivetrain getDrivetrain(){
    return drivetrain;
  }
}