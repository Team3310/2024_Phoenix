package frc.robot.Commands.Auton.Center;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftFromPathEnd;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class ThreeStageLeft extends AutonCommandBase{
    public ThreeStageLeft(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoStageLeft(robotContainer),
            new ParallelDeadlineGroup(
                follow(Paths.getInstance().THREE_STAGE_LEFT), 
                new SequentialCommandGroup(
                    new WaitCommand(0.75),
                    new IntakeAuton(false)
                )
            ),
            // new AimLiftWithOdometryAuton().until(()->Lift.getInstance().isFinished()),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                new SetLiftAngle(Lift.getInstance(), 60.0),
                new SetLeftShooterRPM(Shooter.getInstance(), 3500),
                new SetRightShooterRPM(Shooter.getInstance(), 2500)
            ),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.3)
        );
    }
}
