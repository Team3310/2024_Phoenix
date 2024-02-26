package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Drivetrain.DriveMode;

public class FourStageMiddle extends AutonCommandBase{
    public FourStageMiddle(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStageMiddle(robotContainer),
            new ParallelDeadlineGroup(
                follow("4StageMiddle"),
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
