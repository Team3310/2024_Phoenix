package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;

public class SetElevatorInches extends Command{
    private Elevator elevator;
    private double inches;


    public SetElevatorInches(Elevator elevator, double inches){
        this.elevator = elevator;
        this.inches = inches;

        addRequirements(elevator);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        elevator.setPosition(inches);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){

    }
}
