package frc.robot.Auton.Center;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
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

public class FourStageLeftCounter extends AutonCommandBase{
    public FourStageLeftCounter(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStageLeftCounter(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().THREE_STAGE), 
                new IntakeAuton()
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
