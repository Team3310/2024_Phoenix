// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auton.Paths;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.UpdateManager;
import frc.robot.util.Camera.LimelightHelpers;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final boolean useLimelight = false;

  private RobotContainer m_robotContainer = new RobotContainer();

  private UpdateManager updateManager = new UpdateManager(
    TunerConstants.DriveTrain
  );

  private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[]{
    0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0xc4, 0x68
  };
  private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[]{
    0x00, (byte) 0x80, 0x2f, 0x33, (byte) 0xcf, (byte) 0x65
  };

  public static boolean competitionBot;
  public static boolean practiceBot;

  static {
    List<byte[]> macAddresses;
    try {
      macAddresses = getMacAddresses();
    } catch (IOException e) {
      // Don't crash, just log the stacktrace and continue without any mac addresses.
      // LOGGER.error(e);
      macAddresses = List.of();
    }

    for (byte[] macAddress : macAddresses) {
      // First check if we are the competition bot
      if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
        competitionBot = true;
        break;
      }

      // Next check if we are the practice bot
      if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
        practiceBot = true;
        break;
      }
    }

    if (!competitionBot && !practiceBot) {
      String[] macAddressStrings = macAddresses.stream()
        .map(Robot::macToString)
        .toArray(String[]::new);

      SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
      SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
      SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

      // If something goes terribly wrong we still want to use the competition bot stuff in competition.
      competitionBot = true;
    }

      SmartDashboard.putBoolean("Competition Bot", competitionBot);
  }

  public static boolean isCompetitionBot() {
    return competitionBot;
  }

  public static boolean isPracticeBot() {
    return practiceBot;
  }

  /**
   * Gets the MAC addresses of all present network adapters.
   *
   * @return the MAC addresses of all network adapters.
   */
  private static List<byte[]> getMacAddresses() throws IOException {
    List<byte[]> macAddresses = new ArrayList<>();

    Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

    NetworkInterface networkInterface;
    while (networkInterfaces.hasMoreElements()) {
      networkInterface = networkInterfaces.nextElement();
      byte[] address = networkInterface.getHardwareAddress();
      if (address == null) {
        continue;
      }
      macAddresses.add(address);
    }
    return macAddresses;
  }

    private static String macToString(byte[] address) {
      StringBuilder builder = new StringBuilder();
      for (int i = 0; i < address.length; i++) {
        if (i != 0) {
          builder.append(':');
        }
        builder.append(String.format("%02X", address[i]));
      }
      return builder.toString();
    }


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
    Paths.getInstance().flip();
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
    m_robotContainer.getDrivetrain().setDriveMode(DriveMode.JOYSTICK);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // TunerConstants.DriveTrain.setDriveMode(DriveMode.JOYSTICK);
//    TunerConstants.DriveTrain.trapQueen();
    TunerConstants.DriveTrain.getFrontTargeting().updatePoseEstimatorWithVisionBotPose();
  }

  @Override
  public void teleopPeriodic() {
    TunerConstants.DriveTrain.getFrontTargeting().updatePoseEstimatorWithVisionBotPose();
  }

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
