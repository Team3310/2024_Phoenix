// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auton.DriveTest;
import frc.robot.Commands.Climber.ClimbControlJoysticks;
import frc.robot.Commands.Climber.ClimberAutoZero;
import frc.robot.Commands.Climber.ClimberPrepNoAngle;
import frc.robot.Commands.Climber.SetClimberInches;
import frc.robot.Commands.Climber.SetClimberUpDown;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Drive.SetDriveOrientation;
import frc.robot.Commands.Drive.SetTarget;
import frc.robot.Commands.Elevator.SetElevatorInches;
import frc.robot.Commands.Flicker.LoadAmp;
import frc.robot.Commands.Intake.IntakeAmp;
import frc.robot.Commands.Intake.IntakeAmpToShooter;
import frc.robot.Commands.Intake.IntakeEject;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.SetLiftOff;
import frc.robot.Commands.Shooter.ScoreOffCommand;
import frc.robot.Commands.Shooter.ScoreOnCommand;
import frc.robot.Commands.Shooter.ShooterOff;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Subsystems.Drivetrain.DriveOrientation;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.Telemetry;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.DriverReadout;
import frc.robot.util.Camera.Targeting.TargetSimple;
import frc.robot.util.Choosers.AutonomousChooser;
import frc.robot.util.Choosers.AutonomousChooser.AutonomousMode;
import frc.robot.util.Interpolable.InterpolatingDouble;

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
  public final LED led;

  private final Telemetry logger;

  private final AutonomousChooser autonomousChooser;

  private static RobotContainer instance;

  public RobotContainer() {
    instance = this;

    lift = Lift.getInstance();
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    elevator = Elevator.getInstance();
    climber = Climber.getInstance();
    flicker = Flicker.getInstance();
    led = LED.getInstance();
    drivetrain = TunerConstants.DriveTrain;

    autonomousChooser = new AutonomousChooser();

    logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

    // CommandScheduler.getInstance().registerSubsystem(intake);
    CommandScheduler.getInstance().registerSubsystem(drivetrain);

    configureBindings();
    DriverReadout.addChoosers(autonomousChooser);
  }

  //#region controller buttons
  public void configureDriverController(){
    // intake
    // driverController.rightTrigger(0.5).onTrue(new IntakeAuton()).onFalse(new SetDriveMode(DriveMode.JOYSTICK).andThen(new StopAllIntakes()));
    driverController.rightTrigger(0.5).onTrue(new SetDriveMode(DriveMode.AIM_AT_NOTE)).onFalse(new SetDriveMode(DriveMode.JOYSTICK));
    // driverController.rightTrigger(0.5).onTrue(new IntakeShooter()).onFalse(new StopAllIntakes());
    // driverController.leftTrigger(0.5).onTrue(new IntakeShooter()).onFalse(new StopAllIntakes());
    driverController.leftTrigger(0.5).onTrue(new SetDriveOrientation(DriveOrientation.ROBOT_CENTRIC)).onFalse(new SetDriveOrientation(DriveOrientation.FIELD_CENTRIC));
   // driverController.rightTrigger(0.5).onTrue(new IntakeShooter()).onFalse(new StopAllIntakes());
   // driverController.leftTrigger(0.5).onTrue(new IntakeAmp()).onFalse(new StopAllIntakes());

    // shooting 
    driverController.rightBumper().onTrue(new ScoreOnCommand(shooter, flicker)).onFalse(new ScoreOffCommand(shooter, flicker).andThen(new SetLiftOff(lift)));
    driverController.leftBumper().onTrue(new SetTarget(TargetSimple.SPEAKER).andThen(new SetDriveMode(DriveMode.AIMATTARGET))).onFalse(new SetDriveMode(DriveMode.JOYSTICK)); // auto speaker track
    driverController.x().onTrue(new ShooterOff(shooter));
    driverController.b().onTrue(new ShooterOn(shooter));
    // driverController.a().onTrue(new IntakeAmpToShooter());//new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(25.0);})); // far
    driverController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(22.9);})); // platform
    // driverController.a().onTrue(new DriveTest(this)).onFalse(new SetDriveMode(DriveMode.JOYSTICK));
    //Adding target swapping controls for testing -JB
    driverController.povLeft().onTrue(new SetTarget(TargetSimple.CENTERPASS));
    driverController.povRight().onTrue(new SetTarget(TargetSimple.CORNERPASS));
    driverController.povUp().onTrue(new SetTarget(TargetSimple.SPEAKER));
    
    // driverController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(30.0);})); // fender
    // driverController.a().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5500); shooter.setRightMainRPM(3500); lift.setLiftAngle(19.0);})); // far
    // driverController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(3500); shooter.setRightMainRPM(2500); lift.setLiftAngle(60.0);})); // fender
    // driverController.y().onTrue(new SetDriveMode(DriveMode.STRAFE2APRILTAG));

    // snap to cardinal angles
    // driverController.y().onTrue(new SetSnapToCardinal(Constants.SwerveCardinal.SOURCE));
    // driverController.a().onTrue(new SetSnapToCardinal(Constants.SwerveCardinal.AMP));

    // reset buttons
    driverController.start().onTrue(new SetDriveMode(DriveMode.AIM_AT_NOTE)
      .andThen(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()))
      .andThen(new SetDriveMode(DriveMode.JOYSTICK)));

    // climber prep
    // driverController.povUp().onTrue(new ClimberPrep(this, 0.0));
    // driverController.povRight().onTrue(new ClimberPrep(this, 120.0));
    // driverController.povLeft().onTrue(new ClimberPrep(this, -120.0));
 
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
    // NetworkTableInstance.getDefault().getTable("test").getBooleanTopic("work?").publish().set(true);
    // intake
    operatorController.rightTrigger(0.5).onTrue(new IntakeShooter()).onFalse(new StopAllIntakes());
    operatorController.leftTrigger(0.5).onTrue(new IntakeAmp()).onFalse(new StopAllIntakes());
    operatorController.rightStick().onTrue(new IntakeEject()).onFalse(new StopAllIntakes());

    // shooting
    operatorController.rightBumper().onTrue(new ScoreOnCommand(shooter, flicker)).onFalse(new ScoreOffCommand(shooter, flicker).andThen(new SetLiftOff(lift)));
    operatorController.leftBumper().onTrue(new SetTarget(TargetSimple.SPEAKER).andThen(new SetDriveMode(DriveMode.AIMATTARGET)).andThen(new AimLiftWithOdometry())).onFalse(new SetDriveMode(DriveMode.JOYSTICK)); // auto speaker track
   
    // climb 
    operatorController.back().onTrue(new SetClimberInches(climber, Constants.CLIMBER_MAX_INCHES).alongWith(new SetDriveMode(DriveMode.JOYSTICK)).alongWith(new SetLiftOff(lift)).alongWith(new ShooterOff(shooter)));
    operatorController.start().onTrue(new SetClimberInches(climber, Constants.CLIMBER_MIN_INCHES).alongWith(new SetDriveMode(DriveMode.JOYSTICK)).alongWith(new SetLiftOff(lift)).alongWith(new ShooterOff(shooter)));

    // elevator
    operatorController.povUp().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MAX_INCHES).alongWith(new LoadAmp(flicker)));
    operatorController.povRight().onTrue(new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES).alongWith(new LoadAmp(flicker)));
    operatorController.povDown().onTrue(new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES));
    // operatorController.povLeft().onTrue(new SetTarget(TargetSimple.CORNERPASS).andThen(new SetDriveMode(DriveMode.AIMATTARGET)).andThen(new AimLiftWithOdometry())).onFalse(new SetDriveMode(DriveMode.JOYSTICK)); // auto speaker track
    operatorController.povLeft().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(23.0);})); // amp wing

    // shooting
    operatorController.a().onTrue(new IntakeAmpToShooter());//new InstantCommand(()->{shooter.setLeftMainRPM(5000); shooter.setRightMainRPM(3000); lift.setLiftAngle(25.0);})); // far
    operatorController.x().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(3500); shooter.setRightMainRPM(2500); lift.setLiftAngle(39.5);})); // platform
    operatorController.y().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(3000); shooter.setRightMainRPM(2000); lift.setLiftAngle(60.0);})); // fender
    operatorController.b().onTrue(new InstantCommand(()->{shooter.setLeftMainRPM(3400-200.0-100.0); shooter.setRightMainRPM(2200-200.0-100.0); lift.setLiftAngle(45.0);})); // pass
 //   operatorController.b().onTrue(new SetTarget(TargetSimple.CENTERPASS).andThen(new SetDriveMode(DriveMode.AIMATTARGET)).andThen(new AimLiftWithOdometry())).onFalse(new SetDriveMode(DriveMode.JOYSTICK)); // auto speaker track
 
    // CommandScheduler.getInstance().setDefaultCommand(climber, new ClimbControlJoysticks(climber, operatorController));

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
    if(Constants.debug){
      addTestButtons();
    }
    configureDriverController();
    configureOperatorController();
    //always want these offset buttons for comp
    addOffsetButtons();

    // SmartDashboard.putString("red start pose", PathPlannerPath.fromPathFile("2Amp").flipPath().getPreviewStartingHolonomicPose().getRotation().plus(new Rotation2d(Math.PI)).toString());
    // SmartDashboard.putString("blue start pose", PathPlannerPath.fromPathFile("2Amp").getPreviewStartingHolonomicPose().getRotation().toString());

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  //#endregion

  //#region smartdashboard buttons
    public void addTestButtons(){

      // SmartDashboard.putData("Amp To Shooter", new IntakeAmpToShooter());

      SmartDashboard.putData("test drive train", new SetDriveMode(DriveMode.TEST));
      SmartDashboard.putData("stop test drive train", new SetDriveMode(DriveMode.JOYSTICK));

      addIntakeTestButtons();
      addShooterTestButtons();
      addLiftTestButtons();
      addDrivemodeTestButtons();
      addClimberTestButtons();
      addElevatorTestButtons();
      addFlickerTestButtons();

      // SmartDashboard.putNumber("P", 1.0);
      // SmartDashboard.putNumber("I", 0);
      // SmartDashboard.putNumber("D", 0);

      // SmartDashboard.putNumber("set hood degrees", 20.0);

      // SmartDashboard.putData("shooter wheels go", new SetLeftShooterRPM(shooter, 5000).alongWith(new SetRightShooterRPM(shooter, 3000)));

      // SmartDashboard.putData("Snap 0", new InstantCommand(()->drivetrain.startSnap(0)));
      // SmartDashboard.putData("Snap 90", new InstantCommand(()->drivetrain.startSnap(90)));
      // SmartDashboard.putData("Snap -90", new InstantCommand(()->drivetrain.startSnap(-90)));

      // SmartDashboard.putData("sysIdQuasistatic Forward", drivetrain.sysIdQuasistatic(Direction.kForward));
      // SmartDashboard.putData("sysIdQuasistatic Reverse", drivetrain.sysIdQuasistatic(Direction.kReverse));
      // SmartDashboard.putData("sysIdDynamic Forward", drivetrain.sysIdDynamic(Direction.kForward));
      // SmartDashboard.putData("sysIdDynamic Reverse", drivetrain.sysIdDynamic(Direction.kReverse));
      
    }

    private void addOffsetButtons(){
      SmartDashboard.putData("increase lift offset", new InstantCommand(()->lift.adjustLiftOffset(0.25)));
      SmartDashboard.putData("decrease lift offset", new InstantCommand(()->lift.adjustLiftOffset(-0.25)));
      SmartDashboard.putData("reset lift offset", new InstantCommand(()->lift.resetLiftOffset()));

      if(Constants.debug){
        SmartDashboard.putData("increase RPM Left offset", new InstantCommand(()->shooter.adjustRPMLeftOffset(200.0)));
        SmartDashboard.putData("decrease RPM Left offset", new InstantCommand(()->shooter.adjustRPMLeftOffset(-200.0)));
        SmartDashboard.putData("reset RPM offset", new InstantCommand(()->shooter.resetRPMOffset()));
        SmartDashboard.putData("increase RPM Right offset", new InstantCommand(()->shooter.adjustRPMRightOffset(200.0)));
        SmartDashboard.putData("decrease RPM Right offset", new InstantCommand(()->shooter.adjustRPMRightOffset(-200.0)));
      }
    }

    private void addIntakeTestButtons() {
      // SmartDashboard.putData("Intake Top +rpm", new InstantCommand(()->intake.setTopIntakeRPM(Constants.IN_INTAKE_RPM)));
      // SmartDashboard.putData("Intake Top -rpm", new InstantCommand(()->intake.setTopIntakeRPM(-Constants.IN_INTAKE_RPM)));
      // SmartDashboard.putData("Intake Top 0 rpm", new InstantCommand(()->intake.setTopIntakeRPM(0)));
      // SmartDashboard.putData("Intake Bottom +rpm", new InstantCommand(()->intake.setBottomIntakeRPM(Constants.IN_INTAKE_RPM)));
      // SmartDashboard.putData("Intake Bottom -rpm", new InstantCommand(()->intake.setBottomIntakeRPM(-Constants.IN_INTAKE_RPM)));
      // SmartDashboard.putData("Intake Bottom 0 rpm", new InstantCommand(()->intake.setBottomIntakeRPM(0)));
      // SmartDashboard.putData("Intake Front +rpm", new InstantCommand(()->intake.setFrontIntakeRPM(Constants.IN_INTAKE_RPM)));
      // SmartDashboard.putData("Intake Front -rpm", new InstantCommand(()->intake.setFrontIntakeRPM(-Constants.IN_INTAKE_RPM)));
      // SmartDashboard.putData("Intake Front 0 rpm", new InstantCommand(()->intake.setFrontIntakeRPM(0)));
      // SmartDashboard.putData("set amp +rpm", new SetFlickerRPM(flicker, 1000));
      // SmartDashboard.putData("set amp -rpm", new SetFlickerRPM(flicker,-1000));
      // SmartDashboard.putData("set amp 0", new SetFlickerRPM(flicker, 0));
      
    }

    private void addFlickerTestButtons() {
      // SmartDashboard.putData("set amp +rpm", new SetFlickerRPM(flicker, Constants.AMP_SCORE_RPM));
      // SmartDashboard.putData("set amp -rpm", new SetFlickerRPM(flicker, -Constants.AMP_SCORE_RPM));
      // SmartDashboard.putData("set amp 0", new SetFlickerRPM(flicker, 0.0));
    }

   private void addClimberTestButtons() {
      // SmartDashboard.putData("set climber max", new SetClimberInches(climber, Constants.CLIMBER_MAX_INCHES));
      // SmartDashboard.putData("set climber min", new SetClimberInches(climber, Constants.CLIMBER_MIN_INCHES));
      // SmartDashboard.putData("set climber speed", new SetClimberSpeed(climber, Constants.CLIMBER_AUTO_ZERO_SPEED));
      SmartDashboard.putData("auto zero climber", new ClimberAutoZero(climber));
    }

    private void addElevatorTestButtons() {
      // SmartDashboard.putData("elevator amp score", new SetElevatorInches(elevator, Constants.AMP_SCORE_INCHES));
      // SmartDashboard.putData("elevator trap score", new SetElevatorInches(elevator, Constants.TRAP_SCORE_INCHES));
      // SmartDashboard.putData("elevator zero", new SetElevatorInches(elevator, Constants.ELEVATOR_MIN_INCHES));
      // SmartDashboard.putData("auto zero elevator", new ElevatorAutoZero(elevator));
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
    // SmartDashboard.putData("JOYSTICK_BOTREL", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.JOYSTICK_BOTREL)));
    // SmartDashboard.putData("JOYSTICK", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.JOYSTICK)));
    // SmartDashboard.putData("AIMATTARGET", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.AIMATTARGET)));
    // SmartDashboard.putData("ODOMETRYTRACK", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.ODOMETRYTRACK)));
    // SmartDashboard.putData("AIMATTRAP", new InstantCommand(()->drivetrain.setDriveMode(DriveMode.AIMATTRAP)));
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

      public SendableChooser<AutonomousMode> getAutonomousChooser() {
        return autonomousChooser.getSendable();
      }
    //#endregion
  //#endregion
}