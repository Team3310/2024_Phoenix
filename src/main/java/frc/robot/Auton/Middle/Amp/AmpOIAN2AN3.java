package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Amp.base.AN2AN3;
import frc.robot.Auton.Middle.Amp.base.ANON;
import frc.robot.Auton.Middle.Amp.base.ASIN2;

public class AmpOIAN2AN3 extends AutonCommandBase{

    public AmpOIAN2AN3(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new ANON(robotContainer),
            new ASIN2(robotContainer),
            new AN2AN3(robotContainer)
        );
    }
    
}
