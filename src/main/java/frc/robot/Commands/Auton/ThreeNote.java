package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.IntakeAutoCommand;

public class ThreeNote extends AutonCommandBase{
    public ThreeNote(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoNote(robotContainer),
            new ParallelDeadlineGroup(
                follow("3"),
                new IntakeAutoCommand()
            )
        );
    }
}
