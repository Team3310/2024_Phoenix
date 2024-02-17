package frc.robot.Commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.SetRightShooterRPM;
import frc.robot.Commands.Shooter.SetShooterKickerRPM;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drivetrain.DriveMode;

public class TestOneNote extends AutonCommandBase{
    public TestOneNote(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("OneBlue"));

        this.addCommands(
            new ParallelDeadlineGroup(
                follow(getPath("OneBlue")), 
                new IntakeAuton(),
                new SetLeftShooterRPM(Shooter.getInstance(), 6000),
                new SetRightShooterRPM(Shooter.getInstance(), 3000)
            ),
            new SetLiftAngle(Lift.getInstance(), 27.0),
            new ParallelDeadlineGroup(
                new WaitCommand(2.5),
                new SetDriveMode(DriveMode.AIMATTARGET_AUTON),
                new AimLiftWithOdometry()
            ),
            new SetShooterKickerRPM(Shooter.getInstance(), 1500)
        );
    }
}
