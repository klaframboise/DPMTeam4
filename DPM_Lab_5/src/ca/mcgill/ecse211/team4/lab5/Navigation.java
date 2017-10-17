package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {

	private static final int[] WAYPOINTS = {0,1,1,2,1,0,2,1,2,2};

	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean isNavigating;
	private double waypointX;
	private double waypointY;
	private boolean isAvoiding;
	private Object lock;

	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		isNavigating = false;
		waypointX = 0;
		waypointY = 0;
		lock = new Object();
	}

	public void run() {

		int waypoint = 0;
		while(waypoint < WAYPOINTS.length) {
			synchronized(lock) {
				if(!isAvoiding) {
					Sound.beep();
					travelTo(WAYPOINTS[waypoint] * ZipLineLab.GRID_SIZE, WAYPOINTS[waypoint + 1] * ZipLineLab.GRID_SIZE);

				}
				if(!isAvoiding) {
					waypoint += 2;
				}
			}
		}

	}

	/**
	 * Travels to the given way point.
	 * @param x x-coordinate in cm
	 * @param y y-coordinate in cm
	 */
	void travelTo(double x, double y) {

		// if currently navigating, don't change destination waypoint
		waypointX = (isNavigating())? waypointX : x;
		waypointY = (isNavigating())? waypointY : y;
		isNavigating = true;

		double dX = x - odo.getX();
		double dY = y - odo.getY();
		double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
		double heading = 0; 

		//compute heading
		if (dY >= 0) { //ok
			heading = Math.atan(dX/dY); //isn't it dX/dY ?
		}
		else if (dY < 0) { //ok
			heading = Math.PI + Math.atan(dX/dY);
		}

		// wrap around 2pi
		if(heading > 2 * Math.PI) heading -= 2 * Math.PI;

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		//turn robot to wanted heading 
		turnTo(heading);
		isNavigating = true;

		//travel to x,y
		int rotateAngle = ZipLineLab.convertDistance(ZipLineLab.WHEEL_RADIUS, distance);
		leftMotor.setSpeed(ZipLineLab.FORWARD_SPEED);
		rightMotor.setSpeed(ZipLineLab.FORWARD_SPEED);
		leftMotor.rotate(rotateAngle, true);
		rightMotor.rotate(rotateAngle, false);
		isNavigating = false;
	}

	/**
	 * Turns to given heading using the minimum angle.
	 * @param theta heading in radians
	 */
	void turnTo(double theta) {

		double dTheta = theta - odo.getTheta();
		if(dTheta < 0) dTheta += 2 * Math.PI;
		//System.out.println("\n\n\n\n\n\ndtheta: " + dTheta);

		if (dTheta > Math.PI) {
			dTheta = 2* Math.PI - dTheta;
			turn(Math.toDegrees(dTheta), "left");
		}
		else {
			turn(Math.toDegrees(dTheta), "right");
		}

	}

	/**
	 * Turns in given direction.
	 * @param dTheta change in heading wanted, in degrees
	 * @param direction
	 */
	public void turn(double dTheta, String direction) {

		int distance = ZipLineLab.convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, dTheta);

		// set motor speed
		leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
		rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);

		switch (direction) {
		case "left" :
			leftMotor.rotate(-distance, true);
			rightMotor.rotate(distance, false);
			break;
		case "right" :
			leftMotor.rotate(distance, true);
			rightMotor.rotate(-distance, false);
			break;
		}
	}

	/**
	 * 
	 * @return isNavigating
	 */
	boolean isNavigating() {
		return isNavigating;
	}
	
	/**
	 * Stops the motors and signals the Navigation algorithm that another process is taking over 
	 * the driving of the robot.
	 */
	void pause() {
		leftMotor.stop(true);
		rightMotor.stop(true);

		isAvoiding = true;
		
	}

	/**
	 * Resumes the navigation.
	 */
	void resumeNav() {
		isAvoiding = false;
	}

	public double getWaypointX() {
		return waypointX;
	}

	public double getWaypointY() {
		return waypointY;
	}

	public boolean isAvoiding() {
		return isAvoiding;
	}

}
