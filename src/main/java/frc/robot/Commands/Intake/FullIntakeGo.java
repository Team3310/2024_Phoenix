package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class FullIntakeGo extends Command {
    private Intake intake;
    private Shooter shooter;
    private Lift lift;
    private Elevator elevator;

    public FullIntakeGo(){
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.lift = Lift.getInstance();
        this.elevator = Elevator.getInstance();

        addRequirements(intake, shooter, lift, elevator);
    } 

    @Override
    public void initialize() {
        // if (!shooter.isNoteLoaded()) {
            intake.setFrontIntakeRPM(Constants.IN_INTAKE_RPM);
            intake.setBackIntakeRPM(Constants.IN_INTAKE_RPM);
            shooter.setKickerRPM(Constants.KICKER_INTAKE_RPM);
            elevator.setPosition(2.0);
            lift.setLiftAngle(20.0);
        // }
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
        intake.setBackIntakeRPM(0.0);
        shooter.setKickerRPM(0.0);
        elevator.setPosition(0.0);
    }
}
