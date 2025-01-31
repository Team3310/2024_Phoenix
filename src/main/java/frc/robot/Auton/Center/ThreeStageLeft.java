package frc.robot.Auton.Center;

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

public class ThreeStageLeft extends AutonCommandBase{
    public ThreeStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoStageLeft(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().THREE_STAGE_LEFT), 
                new SequentialCommandGroup(
                    new WaitCommand(0.75),
                    new IntakeShooter(false)
                )
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            // new ParallelDeadlineGroup(
            //     new WaitCommand(0.5), 
            //     new SetLiftAngle(Lift.getInstance(), 60.0),
            //     new SetLeftShooterRPM(Shooter.getInstance(), 3500),
            //     new SetRightShooterRPM(Shooter.getInstance(), 2500)
            // ),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.3)
        );
    }
}
