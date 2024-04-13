package frc.robot.Auton.Middle.Source;

import frc.robot.RobotContainer;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Middle.Source.Base.SMONPN;
import frc.robot.Auton.Middle.Source.Base.SourceIN;

public class SourceIOP extends AutonCommandBase{

    public SourceIOP(RobotContainer robotContainer) {
        super(robotContainer);
        
        this.addCommands(
            new SourceIN(robotContainer),
            new SMONPN(robotContainer)
        );
    }
    
}
