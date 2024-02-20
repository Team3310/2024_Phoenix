package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;

public class IntakeAmp extends Command{
    private Intake intake;
    private Flicker flicker;

    public IntakeAmp(){
        this.intake = Intake.getInstance();
        this.flicker = Flicker.getInstance();

        addRequirements(intake, flicker);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.UP_INTAKE_RPM);
        intake.setBottomIntakeRPM(Constants.UP_INTAKE_RPM);
        intake.setTopIntakeRPM(-Constants.UP_INTAKE_RPM);
        flicker.setRPM(-1.0);
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
