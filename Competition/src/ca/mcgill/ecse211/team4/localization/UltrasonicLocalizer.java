package ca.mcgill.ecse211.team4.localization;

import java.util.Arrays;

import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class uses the ultrasonic sensor to correct to odometer's theta by taking the average of 
 * two angles at which wall edges are detected.
 * @author Walid Chabchoub & Kevin Laframboise
 */
public class UltrasonicLocalizer {

	/**
	 * Sample provider for the ultrasonic localizer.
	 */
	private SampleProvider us;

	/**
	 * Buffer array for ultrasonic data.
	 */
	private float[] usData;

	/**
	 * Motors driving the robot, used in the sweep operation.
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	/**
	 * Angle at which first wall edge is detected.
	 */
	private double alphaAngle; 

	/**
	 * Angle at which second wall edge is detected.
	 */
	private double betaAngle;

	/**
	 * Angle delta between odometer's zero heading and corrected zero heading.
	 */
	private double deltaTheta;

	/**
	 * Indicates whether the robot is sweeping clockwise.
	 */
	private boolean goingClockwise;

	/**
	 * Distance in cm, as measured by the ultrasonic sensor.
	 */
	private float currentDistance;

	/**
	 * Creates an UltrasonicLocalizer object with given properties.
	 * @param us 
	 * @param usData
	 * @param leftMotor
	 * @param rightMotor
	 */
	public UltrasonicLocalizer(SampleProvider us, float[] usData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.us = us;
		this.usData = usData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		goingClockwise = true;

	}

	/**
	 * Localizes the robot with respect to its distance form the wall.
	 */
	public void localize() {
		leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET);
		rightMotor.setSpeed(Robot.ROTATE_SPEED);
		currentDistance = Helper.getFilteredDistance(us, usData);
		if(currentDistance < 30) { //in case we start facing the walls
			while(currentDistance < 30) { //then move away from the walls in a clockwise manner
				currentDistance = Helper.getFilteredDistance(us, usData);
				leftMotor.forward();
				rightMotor.backward();
			}
			//			try {
			//				Thread.sleep(3000); //leave some time for the robot to move away so not to disturb the sensor
			//			} catch (InterruptedException e) {
			//				e.printStackTrace();
			//			}
			currentDistance = Helper.getFilteredDistance(us, usData); 
			while(currentDistance > 30  && goingClockwise) { //now look for the closest wall clockwise
				currentDistance = Helper.getFilteredDistance(us, usData);
				//sweep(clockwise, 72);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop and record the alpha angle once detected first falling edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = Robot.getOdo().getTheta();
			leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET); //go counter counterclockwise
			rightMotor.setSpeed(Robot.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Sound.beep();
			try {											//keep going counterclockwise for at least 3 sec
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = Helper.getFilteredDistance(us, usData);
			while(currentDistance > 30 && !goingClockwise) { //we're now looking for another falling edge on the other side
				currentDistance = Helper.getFilteredDistance(us, usData);
				//sweep(clockwise, 72);
				leftMotor.backward(); //we're going counterclockwise
				rightMotor.forward();
			}
			leftMotor.setSpeed(0); //stop motors and record beta angle once found the second falling edge
			rightMotor.setSpeed(0);
			betaAngle = Robot.getOdo().getTheta();
		} else { //this is in case we start away from the walls
			while(currentDistance > 30  && goingClockwise) { //look for the closest wall clockwise
				currentDistance = Helper.getFilteredDistance(us, usData);
				//sweep(clockwise, 72);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop and record the alpha angle once detected first falling edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = Robot.getOdo().getTheta();
			leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET); //go counter counterclockwise
			rightMotor.setSpeed(Robot.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Sound.beep();
			//sweep(clockwise, 4);
			try {
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = Helper.getFilteredDistance(us, usData);
			while(currentDistance > 30 && !goingClockwise) { //we're now looking for another falling edge on the other side
				currentDistance = Helper.getFilteredDistance(us, usData);
				//sweep(clockwise, 72);
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			leftMotor.setSpeed(0); //stop motors and record beta angle
			rightMotor.setSpeed(0);
			leftMotor.forward();
			rightMotor.backward();
			betaAngle = Robot.getOdo().getTheta();
			Sound.beep();
		}
		if (alphaAngle > betaAngle) {
			deltaTheta = Math.PI/4.0 - (alphaAngle + betaAngle)/2.0 - 0.12;
		}
		else {
			deltaTheta = (5*Math.PI)/4.0 - (alphaAngle + betaAngle)/2.0 - 0.12;
		} //0.1 used for offset
		double currentTheta = Robot.getOdo().getTheta();
		Robot.getOdo().setTheta(currentTheta + deltaTheta); //correct the Odometer's theta value to the correct one

		Robot.getNav().turnTo(0);
	}
}
