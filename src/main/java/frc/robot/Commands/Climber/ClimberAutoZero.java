package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Climber;

public class ClimberAutoZero extends Command {
	private Climber climber;
	private double MIN_CLIMBER_POSITION_CHANGE = 0.05;
	private double MAX_ATTEMPTS = 2000;
	private double lastClimberRightPosition;
	private double lastClimberLeftPosition;
	private int encoderCountRight;
	private int encoderCountLeft;
	private double speedRight;
	private double speedLeft;
	boolean testRight;
	boolean testLeft;

	public ClimberAutoZero(Climber climber) {
		this.climber = climber;

		addRequirements(climber);
	}

	@Override
	public void initialize() {
		lastClimberRightPosition = Constants.CLIMBER_MAX_INCHES;
		lastClimberLeftPosition = Constants.CLIMBER_MAX_INCHES;
		speedRight = Constants.CLIMBER_AUTO_ZERO_SPEED;
		speedLeft = Constants.CLIMBER_AUTO_ZERO_SPEED;
		encoderCountRight = 0;
		encoderCountLeft = 0;
		testRight = false;
		testLeft = false;
		climber.setZeroing(true);
		climber.setSpeed(speedLeft, speedRight);
		System.out.println("speedLeft = " + speedLeft + ", speedRight = " + speedRight);
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		if (!testRight) {
			double currentClimberRightPosition = climber.getRightPositionInches();
			double rightPositionChange = lastClimberRightPosition - currentClimberRightPosition;
			lastClimberRightPosition = currentClimberRightPosition;
			testRight = encoderCountRight > 2 && Math.abs(rightPositionChange) < MIN_CLIMBER_POSITION_CHANGE
					&& Math.abs(climber.getRightCurrent()) > Constants.CLIMBER_AUTO_ZERO_MOTOR_CURRENT;
			System.out.println("encoderCountRight = " + encoderCountRight + ", testRight = " + testRight
					+ ", right climber change = " + rightPositionChange + ", right current = "
					+ climber.getRightCurrent());

			if (Math.abs(rightPositionChange) < MIN_CLIMBER_POSITION_CHANGE) {
				encoderCountRight++;
			} else {
				encoderCountRight = 0;
			}

			if (testRight) {
				speedRight = 0;
				climber.setSpeed(speedLeft, speedRight);
				System.out.println("speedLeft = " + speedLeft + ", speedRight = " + speedRight);
			}
		}

		if (!testLeft) {
			double currentClimberLeftPosition = climber.getLeftPositionInches();
			double leftPositionChange = lastClimberLeftPosition - currentClimberLeftPosition;
			lastClimberLeftPosition = currentClimberLeftPosition;
			testLeft = encoderCountLeft > 2 && Math.abs(leftPositionChange) < MIN_CLIMBER_POSITION_CHANGE
					&& Math.abs(climber.getLeftCurrent()) > Constants.CLIMBER_AUTO_ZERO_MOTOR_CURRENT;
			System.out.println("encoderCountLeft = " + encoderCountLeft + ", testLeft = " + testLeft
					+ ", left climber change = " + leftPositionChange + ", left current = " + climber.getLeftCurrent());

			if (Math.abs(leftPositionChange) < MIN_CLIMBER_POSITION_CHANGE) {
				encoderCountLeft++;
			} else {
				encoderCountLeft = 0;
			}

			if (testLeft) {
				speedLeft = 0;
				climber.setSpeed(speedLeft, speedRight);
				System.out.println("speedLeft = " + speedLeft + ", speedRight = " + speedRight);
			}
		}

		return (testRight && testLeft) || encoderCountRight > MAX_ATTEMPTS || encoderCountLeft > MAX_ATTEMPTS;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("Climber zero set");
		climber.setSpeed(0, 0);
		climber.setZeroing(false);
		climber.setClimberZero(Constants.CLIMBER_MIN_INCHES);
		climber.setPosition(Constants.CLIMBER_MIN_INCHES);
	}
}
