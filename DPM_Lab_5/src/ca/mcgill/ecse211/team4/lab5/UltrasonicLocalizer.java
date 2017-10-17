package ca.mcgill.ecse211.team4.lab5;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {

	private SampleProvider us;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double alphaAngle, betaAngle, deltaTheta;
	private boolean goingClockwise;
	private float currentDistance;

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
		leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
		rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
		currentDistance = getFilteredDistance();
		if(currentDistance < 30) { //in case we start facing the walls
			while(currentDistance < 30) { //then move away from the walls in a clockwise manner
				currentDistance = getFilteredDistance();
				leftMotor.forward();
				rightMotor.backward();
			}
//			try {
//				Thread.sleep(3000); //leave some time for the robot to move away so not to disturb the sensor
//			} catch (InterruptedException e) {
//				e.printStackTrace();
//			}
			currentDistance = getFilteredDistance(); 
			while(currentDistance > 30  && goingClockwise) { //now look for the closest wall clockwise
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop and record the alpha angle once detected first falling edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = ZipLineLab.getOdo().getTheta();
			leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED); //go counter counterclockwise
			rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Sound.beep();
			try {											//keep going counterclockwise for at least 3 sec
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance > 30 && !goingClockwise) { //we're now looking for another falling edge on the other side
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.backward(); //we're going counterclockwise
				rightMotor.forward();
			}
			leftMotor.setSpeed(0); //stop motors and record beta angle once found the second falling edge
			rightMotor.setSpeed(0);
			betaAngle = ZipLineLab.getOdo().getTheta();
		} else { //this is in case we start away from the walls
			while(currentDistance > 30  && goingClockwise) { //look for the closest wall clockwise
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.forward();
				rightMotor.backward();
			}
			leftMotor.setSpeed(0); //stop and record the alpha angle once detected first falling edge
			rightMotor.setSpeed(0);
			goingClockwise = false;
			alphaAngle = ZipLineLab.getOdo().getTheta();
			leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED); //go counter counterclockwise
			rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			Sound.beep();
			//sweep(clockwise, 4);
			try {
				Thread.sleep(3000); //give time for motors to move away from back wall
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			currentDistance = getFilteredDistance();
			while(currentDistance > 30 && !goingClockwise) { //we're now looking for another falling edge on the other side
				currentDistance = getFilteredDistance();
				//sweep(clockwise, 72);
				leftMotor.backward();
				rightMotor.forward();
			}
			Sound.beep();
			leftMotor.setSpeed(0); //stop motors and record beta angle
			rightMotor.setSpeed(0);
			leftMotor.forward();
			rightMotor.backward();
			betaAngle = ZipLineLab.getOdo().getTheta();
			Sound.beep();
		}
		if (alphaAngle > betaAngle) {
			deltaTheta = Math.PI/4.0 - (alphaAngle + betaAngle)/2.0 - 0.12;
		}
		else {
			deltaTheta = (5*Math.PI)/4.0 - (alphaAngle + betaAngle)/2.0 - 0.12;
		} //0.1 used for offset
		double currentTheta = ZipLineLab.getOdo().getTheta();
		ZipLineLab.getOdo().setTheta(currentTheta + deltaTheta); //correct the Odometer's theta value to the correct one
		
		ZipLineLab.getNav().turnTo(0);
	}


	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * Sensor data is filtered. Will return the median of a set of samples
	 * 
	 * @see java.lang.Thread#run()
	 */
	private int getFilteredDistance() {

		for(int i = 0; i < usData.length; i+= us.sampleSize()) {
			us.fetchSample(usData, i * us.sampleSize()); // acquire data
		}
		Arrays.sort(usData);	// sort array
		return (int) ((usData[(usData.length/2)-1] + usData[usData.length/2]) / 2.0 * 100.0); // return median
	}
}
