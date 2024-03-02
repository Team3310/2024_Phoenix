package frc.robot.Commands.Auton.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Subsystems.Lift;

public class FourAmp extends AutonCommandBase{
    public FourAmp(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeAmp(robotContainer),
            new ParallelDeadlineGroup(
                follow(Paths.getInstance().FOUR_AMP).andThen(new WaitCommand(0.5)),
                new IntakeAuton()
            ),
            new AimLiftWithOdometryAuton().until(()->Lift.getInstance().isFinished()),
            new ParallelDeadlineGroup(
                new WaitCommand(0.25), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
