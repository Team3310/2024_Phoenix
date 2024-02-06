package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeAutoCommand extends Command{
    private Intake intake;

    public IntakeAutoCommand(){
        this.intake = Intake.getInstance();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.IN_INTAKE_RPM);
        intake.setBackIntakeRPM(Constants.IN_INTAKE_RPM);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        // return intake.getIndexerSensor();
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
