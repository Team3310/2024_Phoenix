package frc.robot.Auton.Source;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Drive.AimRobot;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class FourStageMiddle extends AutonCommandBase{
    public FourStageMiddle(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStageMiddle(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().FOUR_STAGE_MIDDLE),
                new IntakeShooter()
            ),
            new ParallelDeadlineGroup(
                new AimRobot().andThen(new WaitCommand(0.25)),
                new AimLiftWithOdometry()
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
