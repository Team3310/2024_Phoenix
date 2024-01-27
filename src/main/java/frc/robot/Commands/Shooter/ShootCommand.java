package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShootCommand extends Command{
    private Shooter shooter;
    private double rpm, angle;

    public ShootCommand(double rpm, double angle){
        this.shooter = Shooter.getInstance();
        this.rpm = rpm;
        this.angle = angle;
    }

    @Override
    public void initialize(){
        shooter.setShooterRpm(rpm);
        shooter.setHoodAngle(angle);
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
