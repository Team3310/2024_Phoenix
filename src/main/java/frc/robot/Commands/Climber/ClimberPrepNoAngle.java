package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Elevator.SetElevatorInches;
import frc.robot.Commands.Shooter.ShooterOff;

public class ClimberPrepNoAngle extends SequentialCommandGroup{
    public ClimberPrepNoAngle(RobotContainer robotContainer){
        addCommands(
            new ParallelDeadlineGroup(
                new ShooterOff(robotContainer.shooter),
                new SetElevatorInches(robotContainer.elevator, Constants.ELEVATOR_MIN_INCHES),
                new SetClimberInches(robotContainer.climber, Constants.CLIMBER_MAX_INCHES) 
            )
        );
    }
}