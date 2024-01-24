package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class SetBackIntakeRPM extends Command{
    private Intake intake;
    private double rpm;

    public SetBackIntakeRPM(double rpm){
        this.intake = Intake.getInstance();
        this.rpm = rpm;
    } 

    @Override
    public void initialize() {
        intake.setBackIntakeRPM(rpm);
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
