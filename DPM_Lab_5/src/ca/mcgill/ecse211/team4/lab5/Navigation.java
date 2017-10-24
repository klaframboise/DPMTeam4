package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {

	private Odometer odo;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean isNavigating;
	private double waypointX;
	private double waypointY;
	private boolean isAvoiding;
	private boolean updateNeeded;
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

		double distance; 
		
		while(true) {
			synchronized(lock) {
				
				//check if path needs to be recalculated
				if(!isAvoiding && updateNeeded) {
					travelTo(waypointX, waypointY, true);
					isNavigating = true;
					updateNeeded = false;
				}
				
				//check if navigation is done using euclidean distance
				distance = Math.sqrt(Math.pow(waypointX - odo.getX(), 2) + Math.pow(waypointY - odo.getY(), 2));
				if(distance < 4.0) {
					isNavigating = false;
				}
				//LocalEV3.get().getTextLCD().drawString(String.valueOf(isNavigating), 5, 4);
				//wait 100ms before recalculating path
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}

	}

	/**
	 * Travels to the given way point.
	 * @param x x-coordinate in cm
	 * @param y y-coordinate in cm
	 * @param immediateReturn
	 */
	public void travelTo(double x, double y, boolean immediateReturn) {

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

		//turn robot to wanted heading 
		turnTo(heading);
		isNavigating = true;

		//travel to x,y
		int rotateAngle = ZipLineLab.convertDistance(ZipLineLab.WHEEL_RADIUS, distance);
		leftMotor.setSpeed(ZipLineLab.FORWARD_SPEED * ZipLineLab.SPEED_OFFSET);
		rightMotor.setSpeed(ZipLineLab.FORWARD_SPEED);
		leftMotor.rotate(rotateAngle, true);
		rightMotor.rotate(rotateAngle, immediateReturn);
	}

	/**
	 * Turns to given heading using the minimum angle.
	 * @param theta heading in radians
	 */
	public void turnTo(double theta) {

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
	 * @param direction left or right
	 */
	public void turn(double dTheta, String direction) {

		int distance = ZipLineLab.convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, dTheta);

		// set motor speed
		leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED * ZipLineLab.SPEED_OFFSET);
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
	
	/**
	 * Indicates to navigation that a recalculation of the path is required.
	 */
	void update() {
		updateNeeded = true;
	}

	/**
	 * 
	 * @return current x waypoint
	 */
	public double getWaypointX() {
		return waypointX;
	}

	/**
	 * 
	 * @return current y waypoint
	 */
	public double getWaypointY() {
		return waypointY;
	}
	
	/**
	 * Sets the destination of the navigation.
	 * This overwrites the previous destination.
	 * @param x in cm
	 * @param y in cm
	 */
	public void setWaypoints(double x, double y) {
		waypointX = x;
		waypointY = y;
		updateNeeded = true;
	}

	/**
	 * 
	 * @return isAvoiding
	 */
	public boolean isAvoiding() {
		return isAvoiding;
	}

}
