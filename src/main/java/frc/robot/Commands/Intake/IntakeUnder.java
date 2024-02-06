package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeUnder extends Command{
    private Intake intake;

    public IntakeUnder(){
        this.intake = Intake.getInstance();

        addRequirements(intake);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.UNDER_INTAKE_RPM);
        intake.setBottomIntakeRPM(-Constants.UNDER_INTAKE_RPM);
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
