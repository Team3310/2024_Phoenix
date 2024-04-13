package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Amp.base.ANON;
import frc.robot.Auton.Middle.Amp.base.ASIN2;

public class AmpOIAN2 extends AutonCommandBase{

    public AmpOIAN2(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new ANON(robotContainer),
            new ASIN2(robotContainer)
        );
    }
    
}
