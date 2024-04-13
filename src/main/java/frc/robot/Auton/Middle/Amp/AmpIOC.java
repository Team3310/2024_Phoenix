package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Amp.base.ANIN;
import frc.robot.Auton.Middle.Amp.base.ASCN;
import frc.robot.Auton.Middle.Amp.base.ASON;

public class AmpIOC extends AutonCommandBase{

    public AmpIOC(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new ANIN(robotContainer),
            new ASON(robotContainer),
            new ASCN(robotContainer)
        );
    }
    
}
