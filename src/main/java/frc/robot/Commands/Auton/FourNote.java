package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAutoCommand;

public class FourNote extends AutonCommandBase{
    public FourNote(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeNote(robotContainer),
            new ParallelDeadlineGroup(
                follow("4"),
                new IntakeAutoCommand()
            )
        );
    }
}
