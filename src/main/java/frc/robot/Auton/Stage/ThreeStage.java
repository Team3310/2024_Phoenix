package frc.robot.Auton.Stage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.FullIntakeGo;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.AimLiftFromPathEnd;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class ThreeStage extends AutonCommandBase{
    public ThreeStage(RobotContainer robotContainer){
        super(robotContainer);

 //       resetRobotPose(Paths.getInstance().TWO_STAGE);
//        resetRobotPose(getPath("2Stage"));

        this.addCommands(
            new TwoStage(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().THREE_STAGE),
                new SequentialCommandGroup(
                    new IntakeAuton(),
                    new WaitCommand(0.1),
                    new AimLiftWithOdometry()
                )
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
            )
        );
    }
}
