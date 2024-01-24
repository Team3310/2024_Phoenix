package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class BeltSideSpit extends Command{
    private Intake intake;
    private boolean right;

    public BeltSideSpit(boolean right){
        this.intake = Intake.getInstance();
        this.right = right;
    } 

    @Override
    public void initialize() {
        intake.setBeltIntakeRPM((right?1000:-1000));
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
