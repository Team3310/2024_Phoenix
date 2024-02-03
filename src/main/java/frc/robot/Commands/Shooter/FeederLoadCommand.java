// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class FeederLoadCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_subsystem;
  private final Timer timer = new Timer();
  private boolean isNoteDetected = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FeederLoadCommand(Shooter subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setKickerRPM(1000);
    timer.reset();
    isNoteDetected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setKickerRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isNoteDetected) {
      return timer.hasElapsed(.1);
    }
    else if (m_subsystem.isNoteLoaded()) {
      timer.reset();
      timer.start();
      isNoteDetected = true;
      return false;
    }
    return false;  
  }
}
