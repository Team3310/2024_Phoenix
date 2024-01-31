// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.BeltSideSpit;
import frc.robot.Commands.FlickerCommand;
import frc.robot.Commands.IntakeIn;
import frc.robot.Commands.IntakeSlurp;
import frc.robot.Commands.IntakeUnder;
import frc.robot.Commands.IntakeUp;
import frc.robot.Commands.SetDriveMode;
import frc.robot.Commands.StopIntake;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;
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
  // public final Intake intake = Intake.getInstance();
  // public final Flicker flicker = Flicker.getInstance();

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("test");

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
    driverController.y().onTrue(new InstantCommand(()->drivetrain.setSnapToTarget(false)));
    driverController.x().onTrue(new InstantCommand(()->drivetrain.setSnapToTarget(true)));

    // operatorController.a().onTrue(new InstantCommand(()->flicker.setPosition(0.0)));
    // operatorController.y().onTrue(new InstantCommand(()->flicker.setPosition(1.0)));


    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);


    driverController.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    driverController.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
  
    operatorController.a().onTrue(/*all under*/ new IntakeUnder());
    operatorController.b().onTrue(/*in robot*/ new IntakeIn());
    operatorController.y().onTrue(/*up*/ new IntakeUp());
    operatorController.x().onTrue(/*stop all*/ new StopIntake());
    operatorController.pov(0).onTrue(new IntakeSlurp());
    operatorController.pov(90).onTrue(new BeltSideSpit(false));
    operatorController.pov(270).onTrue(new BeltSideSpit(true));
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