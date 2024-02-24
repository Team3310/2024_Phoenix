package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class TwoStage extends AutonCommandBase{
    public TwoStage(RobotContainer robotContainer){
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
            // new StopAllIntakes(),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    follow(getPath("2Stage")), 
                    follow("2StageScore")
                ),
                new SequentialCommandGroup(
                    new IntakeAuton().until(()->robotContainer.shooter.isNoteLoaded()),
                    new SetLiftAngle(Lift.getInstance(), Constants.FENDER_SHOT_ANGLE)
                )
            ),
            new FeederShootCommandAuton(robotContainer.shooter)
        );
    }
}
