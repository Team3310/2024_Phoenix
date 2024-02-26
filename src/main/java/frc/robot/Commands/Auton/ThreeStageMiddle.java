package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Intake.FullIntakeGo;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Intake.StopAllIntakes;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Drivetrain.DriveMode;

public class ThreeStageMiddle extends AutonCommandBase{
    public ThreeStageMiddle(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("2Stage"));

        this.addCommands(
            new TwoStage(robotContainer),
            new ParallelDeadlineGroup(
                follow("3StageMiddle"),
                new IntakeAuton()
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.75),
                new AimLiftWithOdometryAuton(),
                new SetDriveMode(DriveMode.AIMATTARGET_AUTON)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
