package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class IntakeAuton extends Command {
    private Intake intake;
    private Shooter shooter;
    private Lift lift;

    public IntakeAuton(){
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.lift = Lift.getInstance();

        // addRequirements(intake, shooter, lift);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.IN_INTAKE_RPM);
        intake.setBackIntakeRPM(Constants.IN_INTAKE_RPM);
        shooter.setKickerRPM(Constants.KICKER_INTAKE_RPM);
        lift.setHoodAngle(30.0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return shooter.isNoteLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setFrontIntakeRPM(0.0);
        intake.setBackIntakeRPM(0.0);
        shooter.setKickerRPM(0.0);
    }
}
