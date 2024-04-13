package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMCNPN;
import frc.robot.Auton.Middle.Source.Base.SMCNPN.CNPNPath;
import frc.robot.Auton.Middle.Source.Base.SourceON;

public class SourceOCP extends AutonCommandBase{

    public SourceOCP(RobotContainer robotContainer, CNPNPath path) {
        super(robotContainer);
        
        this.addCommands(
            new SourceON(robotContainer),
            new SMCNPN(robotContainer, path)
        );
    }
    
}
