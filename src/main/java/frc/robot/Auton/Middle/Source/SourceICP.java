package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMCNPN;
import frc.robot.Auton.Middle.Source.Base.SMCNPN.CNPNPath;
import frc.robot.Auton.Middle.Source.Base.SourceIN;

public class SourceICP extends AutonCommandBase{

    public SourceICP(RobotContainer robotContainer, CNPNPath path) {
        super(robotContainer);
        
        this.addCommands(
            new SourceIN(robotContainer),
            new SMCNPN(robotContainer, path)
        );
    }
}
