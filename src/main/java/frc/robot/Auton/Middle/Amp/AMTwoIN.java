package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Amp.TwoAmp;

public class AMTwoIN extends AutonCommandBase{
    public AMTwoIN(RobotContainer robotContainer){
        super(robotContainer);

        this.addCommands(
            new TwoAmp(robotContainer),
            FollowToIntake(Paths.getInstance().AN_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_AS)
        );
    }
}
