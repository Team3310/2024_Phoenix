package frc.robot.Commands.Auton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.StopIntake;

public class TwoNoteAnywhere extends AutonCommandBase{
    public TwoNoteAnywhere(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ParallelDeadlineGroup(
                new PathPlannerAuto("2NoteAnywhere"),
                new IntakeIn()
            ),
            new StopIntake()
        );
    }
}
