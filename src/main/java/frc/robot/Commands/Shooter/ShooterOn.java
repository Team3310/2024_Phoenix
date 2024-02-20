package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShooterOn extends Command{
    private Shooter shooter;

    public ShooterOn(Shooter shooter){
        this.shooter = shooter;
    }

    @Override
    public void execute(){

    }

    @Override
    public void initialize(){
        shooter.setRightMainRPM(3000.0);
        shooter.setLeftMainRPM(5000.0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}