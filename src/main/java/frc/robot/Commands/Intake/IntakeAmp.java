package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Flicker;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;

public class IntakeAmp extends Command{
    private Intake intake;
    private Flicker flicker;
    private LED led;
    private Lift lift;

    public IntakeAmp(){
        this.intake = Intake.getInstance();
        this.flicker = Flicker.getInstance();
        this.led = LED.getInstance();
        this.lift = Lift.getInstance();

        addRequirements(intake, flicker, led, lift);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.UP_INTAKE_RPM);
        intake.setBottomIntakeRPM(Constants.UP_INTAKE_RPM+500.0);
        intake.setTopIntakeRPM(-Constants.UP_INTAKE_RPM);
        flicker.setRPM(Constants.AMP_INTAKE_RPM);
        lift.setLiftAngle(Constants.LIFT_INTAKE_DEGREES);
        led.setBlink(new Color(0, 0, 255));
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished(){
        return flicker.isNoteLoaded();
    }

    @Override
    public void end(boolean interrupted) {
        // intake.setFrontIntakeRPM(0.0);
        // intake.setBottomIntakeRPM(0.0);
        // intake.setTopIntakeRPM(0.0);
        flicker.setRPM(0.0);
        if(!interrupted){
            led.setSolid(new Color(0, 0, 255));
            flicker.setNoteIn(true);
        }
    }
}
