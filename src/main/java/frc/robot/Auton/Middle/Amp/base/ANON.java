package frc.robot.Auton.Middle.Amp.base;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Auton.Amp.TwoAmp;

public class ANON extends AutonCommandBase{

    public ANON(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new TwoAmp(robotContainer),
            FollowToIntake(Paths.getInstance().AN_ON),
            GoToShoot(robotContainer, Paths.getInstance().ON_AS)
        );
    }
    
}
