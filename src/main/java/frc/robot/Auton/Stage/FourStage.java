package frc.robot.Auton.Stage;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Elevator.AmpPrep;
import frc.robot.Commands.Intake.IntakeAmp;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Subsystems.Lift;

public class FourStage extends AutonCommandBase{
    public FourStage(RobotContainer robotContainer){
        super(robotContainer);

//        resetRobotPose(Paths.getInstance().TWO_STAGE);

        this.addCommands(
            new ThreeStage(robotContainer),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().FOUR_STAGE).andThen(new WaitCommand(0.5)),
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
            ),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().CLOSE_STAGE_END),
                new SequentialCommandGroup(
                    new IntakeAmp(),
                    new AmpPrep(robotContainer)
                )
            )
        );
    }
}
