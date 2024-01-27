package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class StopIntake extends Command{
    private Intake intake;

    public StopIntake(){
        this.intake = Intake.getInstance();
    } 

    @Override
    public void initialize() {
        intake.stopIntake();
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
