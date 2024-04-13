package frc.robot.Auton.Middle.Amp;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Amp.base.AN2AN3;
import frc.robot.Auton.Middle.Amp.base.ANIN;
import frc.robot.Auton.Middle.Amp.base.ASON2;

public class AmpIOAN2AN3 extends AutonCommandBase{

    public AmpIOAN2AN3(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new ANIN(robotContainer),
            new ASON2(robotContainer),
            new AN2AN3(robotContainer)
        );
    }
    
}
