package frc.robot.Commands.Auton.Stage;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Drivetrain.DriveMode;

public class FourStage extends AutonCommandBase{
    public FourStage(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeStage(robotContainer),
            new ParallelDeadlineGroup(
                follow("4Stage").andThen(new WaitCommand(0.5)),
                new IntakeAuton()
            ),
            new AimLiftWithOdometryAuton().withTimeout(0.15),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
