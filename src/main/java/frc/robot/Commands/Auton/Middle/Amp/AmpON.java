package frc.robot.Commands.Auton.Middle.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.OneAuton;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class AmpON extends AutonCommandBase{
    public AmpON(RobotContainer robotContainer){
        super(robotContainer);

        resetRobotPose(Paths.getInstance().AMP_AM);

        this.addCommands(
            new OneAuton(robotContainer),
            Follow(Paths.getInstance().AMP_AM),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().AMP_ON),
                new IntakeAuton()
            ),
            // Follow(Paths.getInstance().AM_AS),
            new AimLiftWithOdometryAuton(),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.2)
        );
    }
}
