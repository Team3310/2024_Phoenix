package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Drivetrain.DriveMode;
import frc.robot.Swerve.TunerConstants;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.Target;
import frc.robot.util.Camera.Targeting.TargetSimple;
import frc.robot.util.Choosers.SideChooser.SideMode;

public class SetTarget extends Command{
    private TargetSimple targetSimple;

    public SetTarget(TargetSimple targetSimple){
        this.targetSimple = targetSimple; 
    }

    @Override
    public void initialize() {
        Targeting.setTargetSimple(targetSimple);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
