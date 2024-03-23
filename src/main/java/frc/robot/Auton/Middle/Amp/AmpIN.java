package frc.robot.Auton.Middle.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.IntakeShooter;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class AmpIN extends AutonCommandBase{
    public AmpIN(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().AMP_AM);

        this.addCommands(
            new OneAuton(robotContainer),
            Follow(Paths.getInstance().AMP_AM),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().AMP_IN),
                new IntakeShooter()
            ),
            // Follow(Paths.getInstance().AM_AS),
            new AimLiftWithOdometryAuton(),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.2)
        );
    }
}
