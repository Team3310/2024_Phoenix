package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class SetShooterKickerRPM extends Command{
    private Shooter shooter;
    private double rpm;

    public SetShooterKickerRPM(Shooter shooter, double rpm){
        this.shooter = shooter;
        this.rpm = rpm;

        addRequirements(shooter);
    }

    @Override
    public void execute(){

    }

    @Override
    public void initialize(){
        if(rpm==0.0){
            shooter.setKickerOff();
        }else{
            shooter.setKickerRPM(rpm);
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}