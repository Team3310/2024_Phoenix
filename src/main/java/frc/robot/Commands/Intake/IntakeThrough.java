package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeThrough extends Command{
    private Intake intake;
    
    public IntakeThrough(Intake intake){
        this.intake = intake;

        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.FRONT_IN_INTAKE_RPM*1.5);
        intake.setBottomIntakeRPM(Constants.BACK_EJECT_INTAKE_RPM*3.5);

    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setFrontIntakeRPM(0.0);
        intake.setBottomIntakeRPM(0.0);
    }
}
