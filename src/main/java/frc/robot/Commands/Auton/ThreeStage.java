package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class ThreeStage extends AutonCommandBase{
    public ThreeStage(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("2Stage"));

        this.addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(0.25),
                new ShooterOn(robotContainer.shooter),
                new SetLiftAngle(Lift.getInstance(), Constants.FENDER_SHOT_ANGLE)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            ),
            new ParallelDeadlineGroup(
                follow(getPath("2Stage")), 
                new IntakeAuton()
            ),
            new ParallelDeadlineGroup(
                follow("3Stage"),
                new SequentialCommandGroup( 
                    new ParallelDeadlineGroup(
                        new AimLiftWithOdometryAuton(),
                        new SequentialCommandGroup(
                            new WaitCommand(0.1),
                            new FeederShootCommandAuton(robotContainer.shooter)
                        )
                    ),
                    new IntakeAuton(),
                    new ParallelDeadlineGroup(
                        new AimLiftWithOdometryAuton(),
                        new SequentialCommandGroup(
                            new WaitCommand(0.1),
                            new FeederShootCommandAuton(robotContainer.shooter)
                        )
                    )
                )
            )
        );
    }
}
