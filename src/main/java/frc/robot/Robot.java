// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.UpdateManager;
import frc.robot.util.Camera.LimelightHelpers;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final boolean useLimelight = false;
  private SideMode lastSideMode = SideMode.RED;

  private RobotContainer m_robotContainer = new RobotContainer();

  private UpdateManager updateManager = new UpdateManager(
    TunerConstants.DriveTrain
  );


  @Override
  public void robotInit() {
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-front");
    updateManager.startLoop(0.005);

    m_robotContainer.climber.setClimberZero(0);
    m_robotContainer.elevator.setElevatorZero(0);

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    Paths.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // if (useLimelight) {
    //   var lastResult = LimelightHelpers.getLatestResults("limelight-fron").targetingResults;

    //   Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

    //   if (lastResult.valid) {
    //     m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
    //   }
    // }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.led.setOff();
  }

  @Override
  public void disabledPeriodic() {
    if(m_robotContainer.drivetrain.getSideMode()!=lastSideMode){
      Paths.getInstance().flip(m_robotContainer.drivetrain.getSideMode());
      lastSideMode = m_robotContainer.drivetrain.getSideMode();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // m_robotContainer.getDrivetrain().setDriveMode(DriveMode.JOYSTICK);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    TunerConstants.DriveTrain.setDriveMode(DriveMode.JOYSTICK);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
