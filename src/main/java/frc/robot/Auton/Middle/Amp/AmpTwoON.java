package frc.robot.Auton.Middle.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class AmpTwoON extends AutonCommandBase{
    public AmpTwoON(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AmpIN(robotContainer),
            // Follow(Paths.getInstance().AS_AM),
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
