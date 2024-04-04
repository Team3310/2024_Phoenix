package frc.robot.Commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Camera.Targeting;
import frc.robot.util.Camera.Targeting.TargetSimple;

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
