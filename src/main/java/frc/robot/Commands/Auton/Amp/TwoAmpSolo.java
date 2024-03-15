package frc.robot.Commands.Auton.Amp;

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

public class TwoAmpSolo extends AutonCommandBase{
    public TwoAmpSolo(RobotContainer robotContainer){
        super(robotContainer);

        // resetRobotPose(Paths.getInstance().TWO_AMP);

        this.addCommands(
            new TwoAmpSolo(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO2),
            AimAndShoot(robotContainer)
        );
    }
}
