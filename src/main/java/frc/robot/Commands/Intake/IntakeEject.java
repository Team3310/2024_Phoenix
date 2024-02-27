package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;

public class IntakeEject extends Command {
    private Intake intake;
    private Shooter shooter;
    private Lift lift;
    private Elevator elevator;
    private Flicker flicker;

    public IntakeEject(){
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.lift = Lift.getInstance();
        this.elevator = Elevator.getInstance();
        this.flicker = Flicker.getInstance();

        addRequirements(intake, shooter, lift, elevator, flicker);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.FRONT_EJECT_INTAKE_RPM);
        intake.setBackIntakeRPM(Constants.BACK_EJECT_INTAKE_RPM);
        shooter.setKickerRPM(Constants.KICKER_EJECT_RPM);
        flicker.setRPM(Constants.AMP_EJECT_RPM);
        lift.setLiftAngle(Constants.LIFT_INTAKE_DEGREES);
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
