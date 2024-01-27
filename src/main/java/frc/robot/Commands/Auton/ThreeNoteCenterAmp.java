package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAutoCommand;

public class ThreeNoteCenterAmp extends AutonCommandBase{
    public ThreeNoteCenterAmp(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoNote(robotContainer),
            new ParallelDeadlineGroup(
                follow("3Amp"),
                new IntakeAutoCommand()
            )
        );
    }
}
