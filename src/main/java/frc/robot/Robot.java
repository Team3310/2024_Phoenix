// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Auton.TestOneNote;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.UpdateManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  private UpdateManager updateManager = new UpdateManager(
    TunerConstants.DriveTrain
  );


  @Override
  public void robotInit() {
    updateManager.startLoop(0.005);

    // m_robotContainer.lift.setHoodZero(Constants.HOOD_START_DEGREES);
    m_robotContainer.climber.setClimberZero(0);
    m_robotContainer.elevator.setElevatorZero(0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_robotContainer.getDrivetrain().setDriveMode(DriveMode.AUTON);
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    new TestOneNote(m_robotContainer).schedule();
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
