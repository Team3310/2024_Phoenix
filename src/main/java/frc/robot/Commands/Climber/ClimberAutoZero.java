package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;

public class ClimberAutoZero extends Command{
    private Climber climber;
    private double MIN_CLIMBER_POSITION_CHANGE = 0.05;
	private double lastClimberRightPosition;
	private double lastClimberLeftPosition;
	private int encoderCountRight;
	private int encoderCountLeft;
    private double speedRight;
    private double speedLeft;

    public ClimberAutoZero(Climber climber){
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize(){
		lastClimberRightPosition = Constants.CLIMBER_MAX_INCHES;
		lastClimberLeftPosition = Constants.CLIMBER_MAX_INCHES;
        speedRight = Constants.CLIMBER_AUTO_ZERO_SPEED;
        speedLeft = Constants.CLIMBER_AUTO_ZERO_SPEED;
		encoderCountRight = 0;
		encoderCountLeft = 0;
		climber.setSpeed(speedLeft, speedRight);
        System.out.println("speedLeft = " + speedLeft + ", speedRight = " + speedRight);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
		double currentClimberRightPosition = climber.getRightPositionInches();
		double currentClimberLeftPosition = climber.getLeftPositionInches();
		double rightPositionChange = lastClimberRightPosition - currentClimberRightPosition;
		double leftPositionChange = lastClimberLeftPosition - currentClimberLeftPosition;
		lastClimberRightPosition = currentClimberRightPosition;
		lastClimberLeftPosition = currentClimberLeftPosition;
		boolean testRight = encoderCountRight > 2 && Math.abs(rightPositionChange) < MIN_CLIMBER_POSITION_CHANGE && climber.getRightCurrent() > Constants.CLIMBER_AUTO_ZERO_MOTOR_CURRENT;
		boolean testLeft = encoderCountLeft > 2 && Math.abs(leftPositionChange) < MIN_CLIMBER_POSITION_CHANGE && climber.getLeftCurrent() > Constants.CLIMBER_AUTO_ZERO_MOTOR_CURRENT;
		System.out.println("encoderCountRight = " + encoderCountRight + ", testRight = " + testRight + ", right climber change = " + rightPositionChange + ", right current = " + climber.getRightCurrent());
		System.out.println("encoderCountLeft = " + encoderCountLeft + ", testLeft = " + testLeft + ", left climber change = " + leftPositionChange + ", left current = " + climber.getLeftCurrent());
		
		if (Math.abs(rightPositionChange) < MIN_CLIMBER_POSITION_CHANGE) {
			encoderCountRight++;
		}
		else {
			encoderCountRight = 0;
		}
		if (Math.abs(leftPositionChange) < MIN_CLIMBER_POSITION_CHANGE) {
			encoderCountLeft++;
		}
		else {
			encoderCountLeft = 0;
		}

        if (testRight) {
            speedRight = 0;
		    climber.setSpeed(speedLeft, speedRight);
            System.out.println("speedLeft = " + speedLeft + ", speedRight = " + speedRight);
        }
        if (testLeft) {
            speedLeft = 0;
		    climber.setSpeed(speedLeft, speedRight);
            System.out.println("speedLeft = " + speedLeft + ", speedRight = " + speedRight);
        }
		
		return testRight && testLeft;
    }

    @Override
    public void end(boolean interrupted){
		climber.setSpeed(0,0);
		climber.setClimberZero(Constants.CLIMBER_MIN_INCHES);
        climber.setPosition(Constants.CLIMBER_MIN_INCHES);
    }
}
