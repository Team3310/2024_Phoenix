package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.StopIntake;

public class Test extends AutonCommandBase{
    public Test(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose("test");

        this.addCommands(
            new ParallelDeadlineGroup(
                follow("test"), 
                new SequentialCommandGroup(
                    new IntakeIn(),
                    new WaitCommand(0.75),
                    new StopIntake()
                )
            )
        );
    }
}
