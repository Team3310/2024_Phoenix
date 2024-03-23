package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Lift;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.TunerConstants;

public class IntakeShooter extends Command {
    private Intake intake;
    private Shooter shooter;
    private Lift lift;
    private Elevator elevator;
    private Drivetrain drive;
    private LED led;
    private boolean trackNote;

    public IntakeShooter(){
        this(false);
    } 

    public IntakeShooter(boolean trackNote){
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();
        this.lift = Lift.getInstance();
        this.elevator = Elevator.getInstance();
        this.drive = TunerConstants.DriveTrain;
        this.led = LED.getInstance();

        this.trackNote = trackNote;

        addRequirements(intake, shooter, lift, elevator, led);
    } 

    @Override
    public void initialize() {
        // if (!shooter.isNoteLoaded()) {
            intake.setFrontIntakeRPM(Constants.FRONT_IN_INTAKE_RPM);
            intake.setBackIntakeRPM(Constants.BACK_IN_INTAKE_RPM);
            shooter.setKickerRPM(Constants.KICKER_INTAKE_RPM);
            // elevator.setPosition(2.0);
            lift.setLiftAngle(Constants.LIFT_INTAKE_DEGREES);
            drive.isTrackingNote = trackNote;
        // }
        // led.setBlink(new Color(0, 255, 0));
        led.setBlink(new Color(243, 204, 20));
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
        // intake.setFrontIntakeRPM(0.0);
        // intake.setBackIntakeRPM(0.0);
        shooter.setKickerRPM(0.0);
        elevator.setPosition(0.0);
        drive.isTrackingNote = false;
        if(!interrupted){
            led.setSolid(new Color(243, 204, 20));
            led.setSolid(new Color(243, 204, 20));
            intake.setNoteIn(true);
        }
    }
}
