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
import frc.robot.Commands.Climber.ClimberPrep;
import frc.robot.Commands.Climber.SetClimberInches;
import frc.robot.Commands.Climber.SetClimberSpeed;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Drive.ZeroGyro;
import frc.robot.Commands.Elevator.ElevatorAutoZero;
import frc.robot.Commands.Elevator.SetElevatorInches;
import frc.robot.Commands.Flicker.SetFlickerRPM;
import frc.robot.Commands.Intake.IntakeAmp;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Shooter.FeederShootCommand;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.ShooterOff;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.Telemetry;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.DriverReadout;
import frc.robot.util.Choosers.AutonomousChooser;
import frc.robot.util.Choosers.AutonomousChooser.AutonomousMode;
import frc.robot.util.Choosers.SideChooser;
import frc.robot.util.Choosers.SideChooser.SideMode;
import frc.robot.util.Choosers.SpotChooser;
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
    // intake
    driverController.rightTrigger(0.5).onTrue(new IntakeAuton()).onFalse(new StopAllIntakes());
    driverController.leftTrigger(0.5).onTrue(new IntakeAmp()).onFalse(new StopAllIntakes());

    // shooting
    driverController.rightBumper().onTrue(new FeederShootCommand(shooter)).onFalse(new StopAllIntakes());
    driverController.leftBumper().onTrue(new SetDriveMode(DriveMode.AIMATTARGET)).onFalse(new SetDriveMode(DriveMode.JOYSTICK)); // auto speaker track
    driverController.a().onTrue(new ShooterOff(shooter));
    driverController.y().onTrue(new ShooterOn(shooter));

    // snap to cardinal angles
    driverController.x().onTrue(new InstantCommand(()->{drivetrain.startSnap(-60);}));
    driverController.b().onTrue(new InstantCommand(()->{drivetrain.startSnap(90);}));

    // reset buttons
    driverController.start().onTrue(new ZeroGyro());
 
    // //driving related
    // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    //JOYSTICK MODE and TURN OFF SHOOTER
    // driverController.b().onTrue(new SetDriveMode(DriveMode.JOYSTICK).alongWith(new InstantCommand(()->{shooter.setLeftMainRPM(0.0); shooter.setRightMainRPM(0.0); lift.setLiftAngle(25.0);})));
    // AIMATTARGET and AIMLIFTWITHODOMETRY and TURN ON SHOOTER
    // driverController.a().onTrue(new SetDriveMode(DriveMode.AIMATTARGET).alongWith(new AimLiftWithOdometry()));//.alongWith(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000);})));

    // driverController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000.0); shooter.setRightMainRPM(3000.0); lift.setLiftAngle(SmartDashboard.getNumber("set hood degrees", 20.0));}));
    // driverController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0); shooter.setRightMainRPM(0); lift.setLiftAngle(40.0);}));
    // driverController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0); shooter.setRightMainRPM(0); lift.setLiftAngle(25.0);}));
    // driverController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(0); shooter.setRightMainRPM(0); lift.setLiftAngle(60.0);}));
    // driverController.leftBumper().onTrue(new ScoreAmp(flicker)).onFalse(new StopAllIntakes());//.alongWith(new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES)));
    // driverController.povRight().onTrue(new AimLiftWithOdometry());
    // driverController.povLeft().onTrue(new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES));
    // driverController.povUp().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MAX_INCHES));
    // driverController.povDown().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES));
    // driverController.povRight().onTrue(new LoadAmp(flicker).alongWith(new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES)));
  }

  public void configureOperatorController(){
    // intake
    operatorController.rightTrigger(0.5).onTrue(new IntakeAuton()).onFalse(new StopAllIntakes());
    operatorController.leftTrigger(0.5).onTrue(new IntakeAmp()).onFalse(new StopAllIntakes());

    // shooting
    operatorController.rightBumper().onTrue(new FeederShootCommand(shooter)).onFalse(new StopAllIntakes());
    operatorController.leftBumper().onTrue(new FeederShootCommand(shooter)).onFalse(new StopAllIntakes()); // auto speaker track
 
    // climb
    driverController.povRight().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MAX_INCHES));
    driverController.povLeft().onTrue(new SetClimberInches(climber, Constants.CLIMBER_MAX_INCHES));

    // amp
    driverController.povUp().onTrue(new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES));
    driverController.povDown().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES));

    // shooting
    operatorController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(25.0);}));
    operatorController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(40.0);}));
    operatorController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(60.0);}));
    operatorController.b().onTrue(new ClimberPrep(this));

    //intake
    // operatorController.a().onTrue(new IntakeUnder());
    // operatorController.y().onTrue(new IntakeUp());
    // operatorController.x().onTrue(new StopIntake());
    // operatorController.pov(0).onTrue(new IntakeSlurp());

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
      if(Constants.debug){
        addIntakeTestButtons();
        addShooterTestButtons();
        addLiftTestButtons();
        addDrivemodeTestButtons();
        addClimberTestButtons();
        addElevatorTestButtons();
        addFlickerTestButtons();

        SmartDashboard.putNumber("P", 1.0);
        SmartDashboard.putNumber("I", 0);
        SmartDashboard.putNumber("D", 0);

        SmartDashboard.putNumber("set hood degrees", 20.0);

        SmartDashboard.putData("shooter wheels go", new SetLeftShooterRPM(shooter, 5000).alongWith(new SetRightShooterRPM(shooter, 3000)));
      }
    }

    private void addIntakeTestButtons() {
      SmartDashboard.putData("Intake Top +rpm", new InstantCommand(()->intake.setTopIntakeRPM(Constants.IN_INTAKE_RPM)));
      SmartDashboard.putData("Intake Top -rpm", new InstantCommand(()->intake.setTopIntakeRPM(-Constants.IN_INTAKE_RPM)));
      SmartDashboard.putData("Intake Top 0 rpm", new InstantCommand(()->intake.setTopIntakeRPM(0)));
      SmartDashboard.putData("Intake Bottom +rpm", new InstantCommand(()->intake.setBottomIntakeRPM(Constants.IN_INTAKE_RPM)));
      SmartDashboard.putData("Intake Bottom -rpm", new InstantCommand(()->intake.setBottomIntakeRPM(-Constants.IN_INTAKE_RPM)));
      SmartDashboard.putData("Intake Bottom 0 rpm", new InstantCommand(()->intake.setBottomIntakeRPM(0)));
      SmartDashboard.putData("Intake Front +rpm", new InstantCommand(()->intake.setFrontIntakeRPM(Constants.IN_INTAKE_RPM)));
      SmartDashboard.putData("Intake Front -rpm", new InstantCommand(()->intake.setFrontIntakeRPM(-Constants.IN_INTAKE_RPM)));
      SmartDashboard.putData("Intake Front 0 rpm", new InstantCommand(()->intake.setFrontIntakeRPM(0)));
      SmartDashboard.putData("set amp +rpm", new SetFlickerRPM(flicker, 1000));
      SmartDashboard.putData("set amp -rpm", new SetFlickerRPM(flicker,-1000));
      SmartDashboard.putData("set amp 0", new SetFlickerRPM(flicker, 0));
    }

    private void addFlickerTestButtons() {
      SmartDashboard.putData("set amp +rpm", new SetFlickerRPM(flicker, Constants.AMP_SCORE_RPM));
      SmartDashboard.putData("set amp -rpm", new SetFlickerRPM(flicker, -Constants.AMP_SCORE_RPM));
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
      SmartDashboard.putData("auto zero elevator", new ElevatorAutoZero(elevator));
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
      // SmartDashboard.putData("Hood Angle 15", new SetLiftAngle(lift, 15));
      // SmartDashboard.putData("Hood Angle 20", new SetLiftAngle(lift, 20));
      // SmartDashboard.putData("Hood Angle 22", new SetLiftAngle(lift, 22));
      // SmartDashboard.putData("Hood Angle 25", new SetLiftAngle(lift, 25));
      // SmartDashboard.putData("Hood Angle 30", new SetLiftAngle(lift, 30));
      // SmartDashboard.putData("Hood Angle 35", new SetLiftAngle(lift, 35));
      // SmartDashboard.putData("Hood Angle 40", new SetLiftAngle(lift, 40));
      // SmartDashboard.putData("Hood Angle 45", new SetLiftAngle(lift, 45));
      // SmartDashboard.putData("Hood Angle 50", new SetLiftAngle(lift, 50));
      // SmartDashboard.putData("Hood Angle 60", new SetLiftAngle(lift, 60));
      // SmartDashboard.putData("Hood Angle 70", new SetLiftAngle(lift, 70));
      // SmartDashboard.putData("Zero Hood", new InstantCommand(()->lift.setHoodZero(90)));
    }
  
  public void addDrivemodeTestButtons(){
    SmartDashboard.putData("JOYSTICK", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.JOYSTICK)));
    SmartDashboard.putData("AIMATTARGET", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.AIMATTARGET)));
    // SmartDashboard.putData("AIMATTRAP", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.AIMATTRAP)));
    SmartDashboard.putData("StartSnap XPOS", new InstantCommand(()->drivetrain.startSnap(90)));
    SmartDashboard.putData("StartSnap XNEG", new InstantCommand(()->drivetrain.startSnap(-90)));
    SmartDashboard.putData("StartSnap YPOS", new InstantCommand(()->drivetrain.startSnap(0)));
    SmartDashboard.putData("StartSnap YNEG", new InstantCommand(()->drivetrain.startSnap(180))); 
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