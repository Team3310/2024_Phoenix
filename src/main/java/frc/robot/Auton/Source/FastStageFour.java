package frc.robot.Auton.Source;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class FastStageFour extends AutonCommandBase{
    public FastStageFour(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().THREE_FAST);

        this.addCommands(
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new FeederShootCommandAuton(Shooter.getInstance()).withTimeout(0.2),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().THREE_FAST),
                new IntakeShooter()
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new FeederShootCommandAuton(Shooter.getInstance()).withTimeout(0.2),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().FOUR_FAST),
                new IntakeShooter()
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new FeederShootCommandAuton(Shooter.getInstance()).withTimeout(0.2),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().FAST_END),
                new IntakeShooter()
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new FeederShootCommandAuton(Shooter.getInstance()).withTimeout(0.2)
        );
    }
}
