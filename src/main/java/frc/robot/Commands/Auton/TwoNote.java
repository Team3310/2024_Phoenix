package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAutoCommand;

public class TwoNote extends AutonCommandBase{
    public TwoNote(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("2"));

        this.addCommands(
            new ParallelDeadlineGroup(
                follow("2"), 
                new IntakeAutoCommand()
            )
        );
    }
}
