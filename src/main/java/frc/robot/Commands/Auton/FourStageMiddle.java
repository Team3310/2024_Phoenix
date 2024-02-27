package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Drive.AngleRobotFromPathEnd;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftFromPathEnd;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class FourStageMiddle extends AutonCommandBase{
    public FourStageMiddle(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStageMiddle(robotContainer),
            new ParallelDeadlineGroup(
                follow("4StageMiddle"),
                new IntakeAuton()
            ),
            new ParallelDeadlineGroup(
                new AngleRobotFromPathEnd(getPath("3StageMiddle")).withTimeout(0.5),
                new AimLiftFromPathEnd(getPath("3StageMiddle"))
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
