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
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Lift;

public class ThreeSource extends AutonCommandBase{
    public ThreeSource(RobotContainer robotContainer){
        super(robotContainer);

 //       resetRobotPose(Paths.getInstance().TWO_STAGE);
//        resetRobotPose(getPath("2Stage"));

        this.addCommands(
            new TwoSource(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().THREE_STAGE),
                new SequentialCommandGroup(
                    new IntakeShooter(),
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
