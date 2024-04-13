package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMINPN;
import frc.robot.Auton.Middle.Source.Base.SourceON;

public class SourceOIP extends AutonCommandBase{

    public SourceOIP(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new SourceON(robotContainer),
            new SMINPN(robotContainer)
        );
    }
    
}
