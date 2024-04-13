package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Amp.base.ANIN;
import frc.robot.Auton.Middle.Amp.base.ASON2;

public class AmpIOAN2 extends AutonCommandBase{

    public AmpIOAN2(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new ANIN(robotContainer),
            new ASON2(robotContainer)
        );
    }
    
}
