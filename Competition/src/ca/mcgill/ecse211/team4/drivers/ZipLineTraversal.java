package ca.mcgill.ecse211.team4.drivers;

import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class drives the robot across the zip line.
 * @author Anna Bieber & Kevin Laframboise
 *
 */
public class ZipLineTraversal {

	//Class constants
	/**
	 * Speed at which the zip line wheel turns, in degrees/sec.
	 */
	private static final int FORWARD_SPEED = 250;
	/**
	 * Radius of the zip line wheel, in cm.
	 */
	public static final double WHEEL_RADIUS = 0.95;
	/**
	 * Length of the zip line, in cm.
	 */
	public static final double  DISTANCE = 49.5;
	//class variables
	/**
	 * Motors to be driven by this class.
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor, lineMotor;
	/**
	 * x-coordinate of the center of the zip line start.
	 */
	private double x_c;
	/**
	 * y-coordinate of the center of the zip line start.
	 */
	private double y_c;
	
	/**
	 * Creates a zip line traversal object.
	 * @param lineMotor zip line motor.
	 * @param leftMotor robot's left motor.
	 * @param rightMotor robot's right motor.
	 * @param x_c x coordinate of the zip line start, in cm
	 * @param y_c y coordinate of the zip line start, in cm
	 */
	public ZipLineTraversal(EV3LargeRegulatedMotor lineMotor, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double x_c, double y_c) {
		this.lineMotor = lineMotor;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor; 
		this.x_c = x_c;
		this.y_c = y_c;
	}

	/**
	 * Traverse the zip line. This method assumes that the robot is positioned at (x_0, y_0) prior to call.
	 */
	public void traverse() {
		//start the line motor
		lineMotor.setSpeed(FORWARD_SPEED);
		lineMotor.backward();
		Robot.getNav().travelTo(x_c, y_c + 5, false);
		
		//after getting to point (0,1) travel to the point where the zip line is
		leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET);
		rightMotor.setSpeed(Robot.ROTATE_SPEED);
		leftMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, Robot.GRID_SIZE + 5), true);
		rightMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, Robot.GRID_SIZE + 5), false);
		
		//convert distance and rotate motors so it travels the entire zip line
		lineMotor.rotate(-Helper.convertDistance(WHEEL_RADIUS, DISTANCE), true);
	}

}
