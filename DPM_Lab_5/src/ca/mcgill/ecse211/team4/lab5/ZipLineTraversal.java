package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ZipLineTraversal {

	// Class constants
	private static final int FORWARD_SPEED = 250;
	public static final double GRID_SIZE = 30.48;
	public static final double WHEEL_RADIUS = 0.95;
	public static final double DISTANCE = 30.48 * 6;
	// class variables
	private EV3LargeRegulatedMotor leftMotor, rightMotor, lineMotor;
	private double x_c;
	private double y_c;
	private double x_0;
	private double y_0;

	/**
	 * 
	 * @param lineMotor
	 * @param leftMotor
	 * @param rightMotor
	 * @param x_c
	 *            x coordinate of the zipine start in cm
	 * @param y_c
	 *            y coordinate of the zipine start in cm
	 */
	public ZipLineTraversal(EV3LargeRegulatedMotor lineMotor, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, double x_c, double y_c, double x_0, double y_0) {
		this.lineMotor = lineMotor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x_c = x_c;
		this.y_c = y_c;
	}

	/*
	 * Traverse the zipline. This method assumes the robot is placed at (x_0,
	 * y_0).
	 */
	public void traverse() {
		// start the line motor
		lineMotor.setSpeed(FORWARD_SPEED);
		lineMotor.backward();

		double x_f = 0;
		double y_f = 0;

		// adjust where the robot should travel to after localizing @ (x_0, y_0)
		if (x_c > x_0) {
			x_f = (x_c - 5) * ZipLineLab.GRID_SIZE;
			ZipLineLab.getNav().setWaypoints(x_f, y_f);
		} else if (x_c < x_0) {
			x_f = (x_c - 5) * ZipLineLab.GRID_SIZE;
			ZipLineLab.getNav().setWaypoints(x_f, y_f);
		} else if (y_c > y_0) {
			y_f = (y_c = 5) * ZipLineLab.GRID_SIZE;
			ZipLineLab.getNav().setWaypoints(x_f, y_f);
		} else if (y_c < y_0) {
			y_f = (y_c - 5) * ZipLineLab.GRID_SIZE;
			ZipLineLab.getNav().setWaypoints(x_f, y_f);
		}

		ZipLineLab.getNav().travelTo(ZipLineLab.getNav().getWaypointX(), ZipLineLab.getNav().getWaypointY(), true);
		// after getting to point (0,1) travel to the point where the zip line
		// is
	
		lineMotor.rotate(-ZipLineLab.convertDistance(WHEEL_RADIUS, DISTANCE), false);
		leftMotor.stop(true);
		rightMotor.stop(true);
		lineMotor.stop(false);
	}

}
