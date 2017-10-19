package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ZipLineTraversal {

	//Class constants
	private static final int FORWARD_SPEED = 250;
	public static final double GRID_SIZE = 30.48;
	public static final double WHEEL_RADIUS = 0.95;
	public static final double  DISTANCE = 49.5;
	//class variables
	private EV3LargeRegulatedMotor leftMotor, rightMotor, lineMotor;
	
	public ZipLineTraversal(EV3LargeRegulatedMotor lineMotor, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.lineMotor = lineMotor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor; 
	}

	// this method will allow the robot to traverse the zip line
	public void traverse() {
		//start the line motor
		lineMotor.setSpeed(FORWARD_SPEED);
		lineMotor.forward();
		
		//after getting to point (0,1) travel to the point where the zip line is
		leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
		rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
		leftMotor.rotate(ZipLineLab.convertDistance(ZipLineLab.WHEEL_RADIUS, ZipLineLab.GRID_SIZE + 5), true);
		rightMotor.rotate(ZipLineLab.convertDistance(ZipLineLab.WHEEL_RADIUS, ZipLineLab.GRID_SIZE + 5), false);
		
		//convert distance and rotate motors so it travels the entire zip line
		lineMotor.rotate(ZipLineLab.convertDistance(WHEEL_RADIUS, DISTANCE), true);
	}

}
