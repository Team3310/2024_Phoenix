package frc.robot.Commands.Auton.Stage;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class TwoStage extends AutonCommandBase{
    public TwoStage(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().TWO_STAGE);

        this.addCommands(
            new AimLiftWithOdometryAuton(),
            new ParallelDeadlineGroup(
                new WaitCommand(0.2), 
                new FeederShootCommandAuton(robotContainer.shooter)
            ),
            new ParallelDeadlineGroup(
                follow(Paths.getInstance().TWO_STAGE).andThen(new WaitCommand(0.25)), 
                new IntakeAuton()
            ),
            new ParallelRaceGroup(
                new AimLiftWithOdometryAuton(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    new WaitUntilCommand(()->Lift.getInstance().isFinished())
                )
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
