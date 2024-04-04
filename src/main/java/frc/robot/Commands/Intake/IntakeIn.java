package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeIn extends Command{
    private Intake intake;

    public IntakeIn(){
        this.intake = Intake.getInstance();

        addRequirements(intake);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.FRONT_IN_INTAKE_RPM);
        intake.setBackIntakeRPM(Constants.BACK_IN_INTAKE_RPM);
    }

    @Override
    public void execute() {
        // SmartDashboard.putString("ran command", getName());
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
