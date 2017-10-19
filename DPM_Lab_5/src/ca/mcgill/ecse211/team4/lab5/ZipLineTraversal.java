package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ZipLineTraversal {

	//Class constants
	private static final int FORWARD_SPEED = 250;
	public static final double GRID_SIZE = 30.48;
	public static final double WHEEL_RADIUS = 1.9;
	public static final double  DISTANCE = 49.5;

	
	
	//class variables
	private EV3LargeRegulatedMotor leftMotor, rightMotor, lineMotor;
	Navigation navigator;
	
	public ZipLineTraversal(EV3LargeRegulatedMotor lineMotor, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.lineMotor = lineMotor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor; 
		this.navigator = ZipLineLab.getNav(); 
		
	}

	// this method will allow the robot to traverse the zip line
	public void traversal() {
		//start the line motor
		startMotor();
		
		//after getting to point (0,1) travel to the point where the zip line is
		navigator.travelTo(GRID_SIZE+5, GRID_SIZE-2);
		
		//convert distance and rotate motors so it travels the entire zip line
		lineMotor.rotate(ZipLineLab.convertDistance(WHEEL_RADIUS, DISTANCE));
		
		//once on the line stop the wheel motors
		leftMotor.stop();
		rightMotor.stop();
		
		
		
		
		
	}

	// this method will start the third motor when at the x and y position
	public void startMotor() {
		lineMotor.setSpeed(FORWARD_SPEED);
	}

}
