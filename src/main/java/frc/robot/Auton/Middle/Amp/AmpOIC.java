package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Amp.base.ANON;
import frc.robot.Auton.Middle.Amp.base.ASCN;
import frc.robot.Auton.Middle.Amp.base.ASIN;

public class AmpOIC extends AutonCommandBase{

    public AmpOIC(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new ANON(robotContainer),
            new ASIN(robotContainer),
            new ASCN(robotContainer)
        );
    }
    
}
