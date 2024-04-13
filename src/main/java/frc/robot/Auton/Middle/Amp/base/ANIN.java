package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Amp.TwoAmp;

public class ANIN extends AutonCommandBase{

    public ANIN(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new TwoAmp(robotContainer),
            FollowToIntake(Paths.getInstance().AN_IN),
            GoToShoot(robotContainer, Paths.getInstance().IN_AS)
        );
    }
    
}
