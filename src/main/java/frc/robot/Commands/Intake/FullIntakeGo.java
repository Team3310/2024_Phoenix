package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.TunerConstants;

public class FullIntakeGo extends Command {
    private Intake intake;
    private Shooter shooter;

    public FullIntakeGo(){
        this(false);
    } 

    public FullIntakeGo(boolean trackNote){
        this.intake = Intake.getInstance();
        this.shooter = Shooter.getInstance();

        addRequirements(intake, shooter);
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(Constants.FRONT_IN_INTAKE_RPM);
        intake.setBackIntakeRPM(Constants.BACK_IN_INTAKE_RPM);
        shooter.setKickerRPM(Constants.KICKER_INTAKE_RPM);
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
        
    }
}
