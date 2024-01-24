package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class IntakeSpit extends Command{
    private Intake intake;

    public IntakeSpit(){
        this.intake = Intake.getInstance();
    } 

    @Override
    public void initialize() {
        intake.setFrontIntakeRPM(-Constants.SPIT_RPM);

        if(intake.getIndexerSensor()){
            intake.setBackIntakeRPM(-Constants.SPIT_RPM);
        }else if(intake.getUpSensor()){
            intake.setBottomIntakeRPM(-Constants.SPIT_RPM);
            intake.setTopIntakeRPM(Constants.SPIT_RPM);
        }else{
            intake.setBackIntakeRPM(-Constants.SPIT_RPM);
        }
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
