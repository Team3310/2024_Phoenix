// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Shooter;

public class ScoreOnCommand extends Command {
  private final Shooter shooter;
  private final Flicker flicker;
  private final Elevator elevator;

  public ScoreOnCommand(Shooter shooter, Flicker flicker) {
    this.shooter = shooter;
    this.flicker = flicker;
    this.elevator = Elevator.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
    addRequirements(this.flicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shooter.setKickerRPM(Constants.KICKER_SCORE_RPM);
      
      if(elevator.getPositionInches()>Constants.ELEVATOR_MAX_INCHES-2.0){
        flicker.setRPM(Constants.TRAP_SCORE_RPM);
      }else{
        flicker.setRPM(Constants.AMP_SCORE_RPM);
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
