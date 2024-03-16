package frc.robot.Commands.Auton.Middle.Stage.Base;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Drive.SetDriveMode;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Drivetrain.DriveMode;

public class StageON extends AutonCommandBase{
    public StageON(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().SOURCE_ON);

        this.addCommands(
            new OneAuton(robotContainer),
            FollowToIntake(Paths.getInstance().SOURCE_ON),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().ON_SM),
                new SequentialCommandGroup(
                    new SetLiftAngle(Lift.getInstance(), Constants.LIFT_MIN_DEGREES).withTimeout(0.3),
                    new AimLiftWithOdometryAuton()
                )
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                new SetDriveMode(DriveMode.AIMATTARGET).andThen(new WaitUntilCommand(()->robotContainer.getDrivetrain().snapComplete()))
            ),
            new WaitCommand(0.1),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.2)
        );
    }
}