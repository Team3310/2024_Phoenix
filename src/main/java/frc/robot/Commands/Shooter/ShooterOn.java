package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;

public class ShooterOn extends Command{
    private Shooter shooter;

    public ShooterOn(Shooter shooter){
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void execute(){

    }

    @Override
    public void initialize(){
        shooter.setRightMainRPM(Constants.RIGHT_SCORE_RPM);
        shooter.setLeftMainRPM(Constants.LEFT_SCORE_RPM);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}