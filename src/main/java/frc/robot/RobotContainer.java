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
import frc.robot.Commands.Climber.ClimberAutoZero;
import frc.robot.Commands.Climber.SetClimberInches;
import frc.robot.Commands.Climber.SetClimberSpeed;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Elevator.SetElevatorInches;
import frc.robot.Commands.Flicker.LoadAmp;
import frc.robot.Commands.Flicker.ScoreAmp;
import frc.robot.Commands.Flicker.SetFlickerRPM;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.IntakeSlurp;
import frc.robot.Commands.Intake.IntakeSpit;
import frc.robot.Commands.Intake.IntakeUnder;
import frc.robot.Commands.Intake.Outtake;
import frc.robot.Commands.Intake.IntakeAmp;
import frc.robot.Commands.Intake.StopIntake;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederLoadCommand;
import frc.robot.Commands.Shooter.FeederShootCommand;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.SetShooterKickerRPM;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Flicker;
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
  public final Flicker flicker;
  public final Elevator elevator;
  public final Climber climber;

  private final Telemetry logger;

  private final AutonomousChooser autonomousChooser;
  private final SideChooser sideChooser;
  private final SpotChooser spotChooser;

  private static RobotContainer instance;

  public RobotContainer() {
    instance = this;

    lift = Lift.getInstance();
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    elevator = Elevator.getInstance();
    climber = Climber.getInstance();
    flicker = Flicker.getInstance();
    drivetrain = TunerConstants.DriveTrain;

    spotChooser = new SpotChooser();
    sideChooser = new SideChooser();
    autonomousChooser = new AutonomousChooser();

    logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    // CommandScheduler.getInstance().registerSubsystem(intake);
    CommandScheduler.getInstance().registerSubsystem(drivetrain);

    configureBindings();
    DriverReadout.addChoosers(spotChooser, sideChooser, autonomousChooser);
  }

  //#region controller buttons
  public void configureDriverController(){
    // //driving related
    // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    //JOYSTICK MODE and TURN OFF SHOOTER
    // driverController.b().onTrue(new SetDriveMode(DriveMode.JOYSTICK).alongWith(new InstantCommand(()->{shooter.setLeftMainRPM(0.0); shooter.setRightMainRPM(0.0); lift.setLiftAngle(25.0);})));
    // AIMATTARGET and AIMLIFTWITHODOMETRY and TURN ON SHOOTER
    // driverController.a().onTrue(new SetDriveMode(DriveMode.AIMATTARGET).alongWith(new AimLiftWithOdometry()).alongWith(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000);})));

    // //intake
    driverController.rightTrigger(0.5).onTrue(new IntakeAuton()).onFalse(new StopAllIntakes());
    driverController.leftTrigger(0.5).onTrue(new IntakeAmp()).onFalse(new StopAllIntakes());

    // //shooting
    driverController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0.0); shooter.setRightMainRPM(0.0); lift.setLiftAngle(25.0);}));
    driverController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0); shooter.setRightMainRPM(0); lift.setLiftAngle(40.0);}));
    driverController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0); shooter.setRightMainRPM(0); lift.setLiftAngle(25.0);}));
    driverController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0); shooter.setRightMainRPM(0); lift.setLiftAngle(60.0);}));
    driverController.rightBumper().onTrue(new FeederShootCommand(shooter)).onFalse(new StopAllIntakes());
    driverController.leftBumper().onTrue(new ScoreAmp(flicker)).onFalse(new StopAllIntakes());//.alongWith(new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES)));
    driverController.povRight().onTrue(new AimLiftWithOdometry());

    driverController.povLeft().onTrue(new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES));
    driverController.povUp().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MAX_INCHES));
    driverController.povDown().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES));
    driverController.povRight().onTrue(new LoadAmp(flicker).alongWith(new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES)));
  }

  public void configureOperatorController(){
    //intake
    // operatorController.a().onTrue(new IntakeUnder());
    // operatorController.y().onTrue(new IntakeUp());
    // operatorController.x().onTrue(new StopIntake());
    // operatorController.pov(0).onTrue(new IntakeSlurp());

    // //shooting
    // operatorController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0.0); shooter.setRightMainRPM(0.0); lift.setLiftAngle(25.0);}));
    // operatorController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(40.0);}));
    // operatorController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(25.0);}));
    // operatorController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(60.0);}));
    // operatorController.rightBumper().onTrue(new FeederShootCommand(shooter));
    // operatorController.povRight().onTrue(new AimLiftWithOdometry());

    //flicker
    // flicker.setDefaultCommand(
    //   new FlickerCommand(flicker, operatorController)
    // );
    // operatorController.a().onTrue(new InstantCommand(()->flicker.setPosition(0.0)));
    // operatorController.y().onTrue(new InstantCommand(()->flicker.setPosition(1.0)));
  }

  private void configureBindings() {
    addTestButtons();
    configureDriverController();
    configureOperatorController();

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  //#endregion

  //#region smartdashboard buttons
    public void addTestButtons(){
      addShooterTestButtons();
      addLiftTestButtons();
      addClimberTestButtons();
      addElevatorTestButtons();
      addFlickerTestButtons();
    }

    private void addFlickerTestButtons() {
      SmartDashboard.putData("set amp rpm", new SetFlickerRPM(flicker, Constants.AMP_SCORE_RPM));
      SmartDashboard.putData("set amp 0", new SetFlickerRPM(flicker, 0.0));
    }

   private void addClimberTestButtons() {
      SmartDashboard.putData("set climber max", new SetClimberInches(climber, Constants.CLIMBER_MAX_INCHES));
      SmartDashboard.putData("set climber min", new SetClimberInches(climber, Constants.CLIMBER_MIN_INCHES));
      SmartDashboard.putData("set climber speed", new SetClimberSpeed(climber, Constants.CLIMBER_AUTO_ZERO_SPEED));
      SmartDashboard.putData("auto zero climber", new ClimberAutoZero(climber));
    }

    private void addElevatorTestButtons() {
       SmartDashboard.putData("elevator amp score", new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES));
      SmartDashboard.putData("elevator trap score", new SetElevatorInches(elevator, Constants.TRAP_SCORE_INCHES));
      SmartDashboard.putData("elevator zero", new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES));
    }

    public void addShooterTestButtons(){
    //   SmartDashboard.putData("Right Side RPM 0", new SetRightShooterRPM(shooter, 0.0));
    //   SmartDashboard.putData("Right Side RPM 50", new SetRightShooterRPM(shooter, 0.0));
    //   SmartDashboard.putData("Right Side RPM 500", new SetRightShooterRPM(shooter,500));
    //   SmartDashboard.putData("Right Side RPM 1000", new SetRightShooterRPM(shooter,1000));
    //   SmartDashboard.putData("Right Side RPM 2000", new SetRightShooterRPM(shooter,2000));
    //   SmartDashboard.putData("Right Side RPM 3000", new SetRightShooterRPM(shooter,3000));
    //   SmartDashboard.putData("Right Side RPM 4000", new SetRightShooterRPM(shooter,4000));
    //   SmartDashboard.putData("Right Side RPM 5000", new SetRightShooterRPM(shooter,5000));
    //   SmartDashboard.putData("Right Side RPM 6000", new SetRightShooterRPM(shooter,6000));

    //   SmartDashboard.putData("Left Side RPM 0", new SetLeftShooterRPM(shooter, 0.0));
    //   SmartDashboard.putData("Left Side RPM 50", new SetLeftShooterRPM(shooter, -50));
    //   SmartDashboard.putData("Left Side RPM 500", new SetLeftShooterRPM(shooter, -500));
    //   SmartDashboard.putData("Left Side RPM 1000", new SetLeftShooterRPM(shooter, -1000));
    //   SmartDashboard.putData("Left Side RPM 2000", new SetLeftShooterRPM(shooter, -2000));
    //   SmartDashboard.putData("Left Side RPM 3000", new SetLeftShooterRPM(shooter, -3000));
    //   SmartDashboard.putData("Left Side RPM 4000", new SetLeftShooterRPM(shooter, -4000));
    //   SmartDashboard.putData("Left Side RPM 5000", new SetLeftShooterRPM(shooter, -5000));
    //   SmartDashboard.putData("Left Side RPM 6000", new SetLeftShooterRPM(shooter, -6000));

    //   SmartDashboard.putData("Kicker RPM 0", new SetShooterKickerRPM(shooter, 0.0));
    //   SmartDashboard.putData("Kicker RPM 50", new SetShooterKickerRPM(shooter, 50));
    //   SmartDashboard.putData("Kicker RPM 500", new SetShooterKickerRPM(shooter, 500));
    //   SmartDashboard.putData("Kicker RPM 1000", new SetShooterKickerRPM(shooter, 1000));
    //   SmartDashboard.putData("Kicker RPM 2000", new SetShooterKickerRPM(shooter, 2000));
    //   SmartDashboard.putData("Kicker RPM 3000", new SetShooterKickerRPM(shooter, 3000));
    //   SmartDashboard.putData("Kicker RPM 4000", new SetShooterKickerRPM(shooter, 4000));
    //   SmartDashboard.putData("Kicker RPM 5000", new SetShooterKickerRPM(shooter, 5000));
    //   SmartDashboard.putData("Kicker RPM 6000", new SetShooterKickerRPM(shooter, 6000));

    //   SmartDashboard.putData("Kicker Load", new FeederLoadCommand(shooter));
    //   SmartDashboard.putData("Kicker Shoot", new FeederShootCommand(shooter));
    }

    public void addLiftTestButtons(){
      // SmartDashboard.putData("Lift Angle 15", new SetLiftAngle(lift, 15));
      // SmartDashboard.putData("Lift Angle 20", new SetLiftAngle(lift, 20));
      // SmartDashboard.putData("Lift Angle 22", new SetLiftAngle(lift, 22));
      // SmartDashboard.putData("Lift Angle 25", new SetLiftAngle(lift, 25));
      // SmartDashboard.putData("Lift Angle 30", new SetLiftAngle(lift, 30));
      // SmartDashboard.putData("Lift Angle 35", new SetLiftAngle(lift, 35));
      // SmartDashboard.putData("Lift Angle 40", new SetLiftAngle(lift, 40));
      // SmartDashboard.putData("Lift Angle 45", new SetLiftAngle(lift, 45));
      // SmartDashboard.putData("Lift Angle 50", new SetLiftAngle(lift, 50));
      // SmartDashboard.putData("Lift Angle 60", new SetLiftAngle(lift, 60));
      // SmartDashboard.putData("Lift Angle 70", new SetLiftAngle(lift, 70));
      // SmartDashboard.putData("Zero Lift", new InstantCommand(()->lift.setLiftZero(90)));
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