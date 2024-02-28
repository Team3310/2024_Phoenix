package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Elevator.SetElevatorInches;
import frc.robot.Commands.Lift.SetLiftOff;
import frc.robot.Commands.Shooter.ShooterOff;

public class ClimberPrep extends SequentialCommandGroup{
    public ClimberPrep(RobotContainer robotContainer, double angle){
        addCommands(
            new ParallelDeadlineGroup(
                new ShooterOff(robotContainer.shooter),
                new SetLiftOff(robotContainer.lift),
                new SetElevatorInches(robotContainer.elevator, Constants.ELEVATOR_MIN_INCHES),
                new SetClimberInches(robotContainer.climber, Constants.CLIMBER_MAX_INCHES), 
 //               new InstantCommand(()->{robotContainer.drivetrain.startSnap(Math.toDegrees(robotContainer.drivetrain.getOdoTargeting().getTrapAz()));}) 
                new InstantCommand(()->{robotContainer.drivetrain.startSnap(angle);})
            )
        );
    }
}