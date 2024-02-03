package frc.robot.Commands.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Hood;

public class SetHoodAngle extends Command{
    private Hood hood;
    private double angle;

    public SetHoodAngle(Hood hood, double angle){
        this.hood = hood;
        this.angle = angle;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        hood.setHoodAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}