package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Lift;

public class SetLiftOff extends Command{
    private Lift lift;

    public SetLiftOff(Lift lift){
        this.lift = lift;

        addRequirements(lift);
    }

    @Override
    public void initialize(){
        lift.setLiftAngle(Constants.LIFT_MIN_DEGREES+1.0);
    }

    @Override
    public void execute(){
       
    }

    @Override
    public boolean isFinished(){
        return lift.isFinished();
    }

    @Override
    public void end(boolean interrupted){
        // lift.setLiftOff();
    }
}