package frc.robot.Commands.Auton.Amp;

import frc.robot.RobotContainer;
import frc.robot.Commands.Auton.AutonCommandBase;
import frc.robot.Commands.Auton.Paths;

public class ThreeAmpSolo extends AutonCommandBase{
    public ThreeAmpSolo(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new ThreeAmp(robotContainer),
            FollowToIntake(Paths.getInstance().THREE_AMP_SOLO),
            AimAndShoot(robotContainer),
            FollowToIntake(Paths.getInstance().TWO_AMP_SOLO2),
            AimAndShoot(robotContainer)
        );
    }
}
