package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeSlurp extends Command{
    private Intake intake;

    public IntakeSlurp(){
        this.intake = Intake.getInstance();
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(0);
        intake.setBackIntakeRPM(Constants.SLURP_INTAKE_RPM);
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
