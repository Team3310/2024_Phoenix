package frc.robot.Auton.Amp;

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

public class FourAmp extends AutonCommandBase{
    public FourAmp(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeAmp(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().FOUR_AMP).andThen(new WaitCommand(0.5)),
                new IntakeShooter()
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            ),
            FollowToIntake(Paths.getInstance().SNEAK5),
            GoToShoot(robotContainer, Paths.getInstance().CN_S)
        );
    }
}
