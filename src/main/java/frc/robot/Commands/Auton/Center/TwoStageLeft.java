package frc.robot.Commands.Auton.Center;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class TwoStageLeft extends AutonCommandBase{
    public TwoStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().TWO_STAGE_LEFT_PRE_GRAB);

        this.addCommands(
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().TWO_STAGE_LEFT_PRE_GRAB),
                new ShooterOn(robotContainer.shooter),
                new AimLiftWithOdometryAuton()
            ),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.125),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().TWO_STAGE_LEFT_GRAB),
                new IntakeAuton(false)
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.3)
        );
    }
}
