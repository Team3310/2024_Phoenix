// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.IntakeSlurp;
import frc.robot.Commands.Intake.IntakeSpit;
import frc.robot.Commands.Intake.IntakeUnder;
import frc.robot.Commands.Intake.IntakeUp;
import frc.robot.Commands.Intake.StopIntake;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederLoadCommand;
import frc.robot.Commands.Shooter.FeederShootCommand;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.SetShooterKickerRPM;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.Telemetry;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.DriverReadout;
import frc.robot.util.Choosers.AutonomousChooser;
import frc.robot.util.Choosers.SideChooser;
import frc.robot.util.Choosers.SpotChooser;
import frc.robot.util.Choosers.AutonomousChooser.AutonomousMode;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Choosers.SpotChooser.SpotMode;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public final Drivetrain drivetrain;
  public final Intake intake;
  public final Shooter shooter;
  public final Lift lift;

  private final Telemetry logger;

  private final AutonomousChooser autonomousChooser;
  private final SideChooser sideChooser;
  private final SpotChooser spotChooser;

  private static RobotContainer instance;

  public RobotContainer() {
    instance = this;

    spotChooser = new SpotChooser();
    sideChooser = new SideChooser();
    autonomousChooser = new AutonomousChooser();

    lift = Lift.getInstance();
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    drivetrain = TunerConstants.DriveTrain;

    logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    // CommandScheduler.getInstance().registerSubsystem(intake);
    CommandScheduler.getInstance().registerSubsystem(drivetrain);

    configureBindings();
    DriverReadout.addChoosers(spotChooser, sideChooser, autonomousChooser);
  }

  //#region controller buttons
  public void configureDriverController(){
    //driving related
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverController.povUp().onTrue(new SetDriveMode(DriveMode.JOYSTICK));
    driverController.povDown().onTrue(new SetDriveMode(DriveMode.AIMATTARGET));

    //intake
    driverController.rightTrigger(0.5).onTrue(new IntakeAuton());
    driverController.leftTrigger(0.5).onTrue(new IntakeSpit());
    driverController.leftTrigger(0.5).onFalse(new StopIntake());

    //shooting
    driverController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0.0); shooter.setRightMainRPM(0.0); lift.setHoodAngle(25.0);}));
    driverController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setHoodAngle(40.0);}));
    driverController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setHoodAngle(25.0);}));
    driverController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setHoodAngle(60.0);}));
    driverController.rightBumper().onTrue(new FeederShootCommand(shooter));
    driverController.povRight().onTrue(new AimLiftWithOdometry());
  }

  public void configureOperatorController(){
    //intake
    operatorController.a().onTrue(new IntakeUnder());
    operatorController.y().onTrue(new IntakeUp());
    operatorController.x().onTrue(new StopIntake());
    operatorController.pov(0).onTrue(new IntakeSlurp());

    //shooting
    operatorController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0.0); shooter.setRightMainRPM(0.0); lift.setHoodAngle(25.0);}));
    operatorController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setHoodAngle(40.0);}));
    operatorController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setHoodAngle(25.0);}));
    operatorController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setHoodAngle(60.0);}));
    operatorController.rightBumper().onTrue(new FeederShootCommand(shooter));
    operatorController.povRight().onTrue(new AimLiftWithOdometry());

    //flicker
    // flicker.setDefaultCommand(
    //   new FlickerCommand(flicker, operatorController)
    // );
    // operatorController.a().onTrue(new InstantCommand(()->flicker.setPosition(0.0)));
    // operatorController.y().onTrue(new InstantCommand(()->flicker.setPosition(1.0)));
  }

  private void configureBindings() {
    // addTestButtons();
    configureDriverController();
    configureOperatorController();

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  //#endregion

  //#region smartdashboard buttons
    public void addTestButtons(){
      addShooterTestButtons();
      addLiftTestButtons();
    }

    public void addShooterTestButtons(){
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

      SmartDashboard.putData("Kicker Load", new FeederLoadCommand(shooter));
      SmartDashboard.putData("Kicker Shoot", new FeederShootCommand(shooter));
    }

    public void addLiftTestButtons(){
    SmartDashboard.putData("Hood Angle 15", new SetLiftAngle(lift, 15));
    SmartDashboard.putData("Hood Angle 20", new SetLiftAngle(lift, 20));
    SmartDashboard.putData("Hood Angle 22", new SetLiftAngle(lift, 22));
    SmartDashboard.putData("Hood Angle 25", new SetLiftAngle(lift, 25));
    SmartDashboard.putData("Hood Angle 30", new SetLiftAngle(lift, 30));
    SmartDashboard.putData("Hood Angle 35", new SetLiftAngle(lift, 35));
    SmartDashboard.putData("Hood Angle 40", new SetLiftAngle(lift, 40));
    SmartDashboard.putData("Hood Angle 45", new SetLiftAngle(lift, 45));
    SmartDashboard.putData("Hood Angle 50", new SetLiftAngle(lift, 50));
    SmartDashboard.putData("Hood Angle 60", new SetLiftAngle(lift, 60));
    SmartDashboard.putData("Hood Angle 70", new SetLiftAngle(lift, 70));
    SmartDashboard.putData("Zero Hood", new InstantCommand(()->lift.setHoodZero(90)));
  }
  //#endregion

  //#region getters
    public static RobotContainer getInstance(){
      return instance;
    }
    //#region subsystems
      public Drivetrain getDrivetrain(){
        return drivetrain;
      }
    //#endregion
    
    //#region choosers
      public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autonomousChooser.getCommand();
      }

      public SideMode getSide() {
        return sideChooser.getSide();
      }

      public SpotMode getSpot(){
        return spotChooser.getSpot();
      }

      public SendableChooser<SideMode> getSideChooser() {
        return sideChooser.getSendable();
      }

      public SendableChooser<SpotMode> getSpotChooser() {
        return spotChooser.getSendable();
      }

      public SendableChooser<AutonomousMode> getAutonomousChooser() {
        return autonomousChooser.getSendable();
      }
    //#endregion
  //#endregion
}