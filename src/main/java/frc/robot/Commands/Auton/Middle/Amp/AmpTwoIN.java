package frc.robot.Commands.Auton.Middle.Amp;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;
import frc.robot.Commands.Intake.IntakeAuton;
import frc.robot.Commands.Lift.AimLiftWithOdometryAuton;
import frc.robot.Commands.Shooter.FeederShootCommandAuton;

public class AmpTwoIN extends AutonCommandBase{
    public AmpTwoIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new AmpON(robotContainer),
            Follow(Paths.getInstance().AS_AM),
            new ParallelDeadlineGroup(
                Follow(Paths.getInstance().AMP_IN),
                new IntakeAuton()
            ),
            Follow(Paths.getInstance().AM_AS),
            new AimLiftWithOdometryAuton(),
            new FeederShootCommandAuton(robotContainer.shooter).withTimeout(0.2)
        );
    }
}
