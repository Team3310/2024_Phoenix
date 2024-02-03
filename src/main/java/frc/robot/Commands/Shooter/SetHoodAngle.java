package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class SetHoodAngle extends Command{
    private Shooter shooter;
    private double angle;

    public SetHoodAngle(Shooter shooter, double angle){
        this.shooter = shooter;
        this.angle = angle;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        shooter.setHoodAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}