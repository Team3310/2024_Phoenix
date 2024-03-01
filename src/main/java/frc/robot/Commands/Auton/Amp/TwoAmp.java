package frc.robot.Commands.Auton.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometry;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Lift.SetLiftAngle;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;
import frc.robot.Commands.Shooter.SetLeftShooterRPM;
import frc.robot.Commands.Shooter.ShooterOn;
import frc.robot.Subsystems.Lift;

public class TwoAmp extends AutonCommandBase{
    public TwoAmp(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(getPath("2AmpPreGrab"));

        this.addCommands(
            new ParallelDeadlineGroup(
                follow("2AmpPreGrab").andThen(new WaitCommand(0.3)),
                new AimLiftWithOdometryAuton()
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.3), 
                new FeederShootCommandAuton(robotContainer.shooter)
            ),
            new ParallelDeadlineGroup(
                follow("2AmpGrab").andThen(new WaitCommand(0.25)), 
                new IntakeAuton()
            ),
            new AimLiftWithOdometryAuton().withTimeout(0.5),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                new FeederShootCommandAuton(robotContainer.shooter)
            )
        );
    }
}
