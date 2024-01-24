package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeUp extends Command{
    private Intake intake;

    public IntakeUp(){
        this.intake = Intake.getInstance();
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.UP_INTAKE_RPM);
        intake.setBottomIntakeRPM(Constants.UP_INTAKE_RPM);
        intake.setTopIntakeRPM(-Constants.UP_INTAKE_RPM);
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
