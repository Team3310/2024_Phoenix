package frc.robot.Commands.Lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Lift;

public class SetLiftAngle extends Command{
    private Lift lift;
    private double angle;

    public SetLiftAngle(Lift hood, double angle){
        this.lift = hood;
        this.angle = angle;
    }

    @Override
    public void initialize(){
        lift.setHoodAngle(angle);
    }

    @Override
    public void execute(){
       
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}