package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;

public class TwoStageLeft extends AutonCommandBase{
    public TwoStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("2StageLeftPreGrab"));

        this.addCommands(
            new ParallelDeadlineGroup(
                follow("2StageLeftPreGrab"),
                new ShooterOn(robotContainer.shooter),
                new AimLiftWithOdometryAuton()
            ),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.125),
            new ParallelDeadlineGroup(
                follow("2StageLeftGrab"),
                new IntakeAuton(false)
            ),
            new AimLiftWithOdometryAuton().withTimeout(0.125),
            new FeederShootCommandAuton(robotContainer.shooter)
        );
    }
}
