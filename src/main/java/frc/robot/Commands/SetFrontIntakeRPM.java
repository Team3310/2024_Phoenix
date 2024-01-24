package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class SetFrontIntakeRPM extends Command{
    private Intake intake;
    private double rpm;

    public SetFrontIntakeRPM(double rpm){
        this.intake = Intake.getInstance();
        this.rpm = rpm;
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(rpm);
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
