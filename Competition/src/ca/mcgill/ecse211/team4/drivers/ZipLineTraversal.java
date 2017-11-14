package ca.mcgill.ecse211.team4.drivers;

import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class drives the robot across the zip line.
 * @author Anna Bieber & Kevin Laframboise
 *
 */
/**
 * @author Kevin
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
	 * x-coordinate of the center of the zip line end.
	 */
	private double x_fc;
	/**
	 * y-coordinate of the center of the zip line end.
	 */
	private double y_fc;
	/**
	 * x-coordinate of the next grid line intersection in the direction of the zip line.
	 */
	private double x_f0;
	/**
	 * y-coordinate of the next grid line intersection in the direction of the zip line.
	 */
	private double y_f0;

	/**
	 * Creates a zip line traversal object.
	 * @param lineMotor zip line motor.
	 * @param leftMotor robot's left motor.
	 * @param rightMotor robot's right motor.
	 * @param x_c x coordinate of the zip line start, in cm.
	 * @param y_c y coordinate of the zip line start, in cm.
	 * @param x_fc x-coordinate of the center of the zip line end.
	 * @param y_fc y-coordinate of the center of the zip line end.
	 * @param x_f0 x-coordinate of the next grid line intersection in the direction of the zip line.
	 * @param y_f0 y-coordinate of the next grid line intersection in the direction of the zip line.
	 */
	public ZipLineTraversal(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3LargeRegulatedMotor lineMotor, double x_c, double y_c, double x_fc, double y_fc, double x_f0,
			double y_f0) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.lineMotor = lineMotor;
		this.x_c = x_c;
		this.y_c = y_c;
		this.x_fc = x_fc;
		this.y_fc = y_fc;
		this.x_f0 = x_f0;
		this.y_f0 = y_f0;
	}

	/**
	 * Traverse the zip line. This method assumes that the robot is positioned at (x_0, y_0) prior to call.
	 */
	public void traverse() {
		/* Initialize local variable */
		final float FLOOR_RED_INTENSITY = 0.1f;	//TODO measure

		/* Start the line motor */
		lineMotor.setSpeed(FORWARD_SPEED);
		//		lineMotor.forward();


		/* Navigate to start of zipline */
		lineMotor.forward();
		Robot.getNav().travelTo(x_c * Robot.GRID_SIZE, y_c * Robot.GRID_SIZE, false);
		
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		leftMotor.rotate(1440, true);
		rightMotor.rotate(1440, false);

		/* By now we are on the zip line */
		
		lineMotor.rotate(Helper.convertDistance(WHEEL_RADIUS, 1.5 * Robot.GRID_SIZE));

		/* Keep wheels turning while on the ground */
		//		while(LightLocalizer.getRedIntensity() > FLOOR_RED_INTENSITY) {
		//			leftMotor.setSpeed(150);
		//			rightMotor.setSpeed(150);
		//			leftMotor.forward();
		//			rightMotor.forward();
		//			try {
		//				Thread.sleep(100);
		//			} catch (InterruptedException e) {
		//				// TODO Auto-generated catch block
		//				e.printStackTrace();
		//			}
		//		}

		/* Stop motors once no longer on the ground */
		//		leftMotor.stop(true);
		//		rightMotor.stop(false);

		/* Keep line motor turning while not on the ground */
		//		while(LightLocalizer.getRedIntensity() < FLOOR_RED_INTENSITY) {
		//			try {
		//				Thread.sleep(100);
		//			} catch (InterruptedException e) {
		//				// TODO Auto-generated catch block
		//				e.printStackTrace();
		//			}
		//		}		
		//		
		//		
		/* Update odometer */
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		leftMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 25), true);
		rightMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 25), false);
		leftMotor.rotate(- Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, 20), true);
		rightMotor.rotate(Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, 20), false);
		leftMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 20), true);
		rightMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 20), false);
		Robot.getOdo().setX(x_f0 * Robot.GRID_SIZE);
		Robot.getOdo().setY(y_f0 * Robot.GRID_SIZE);
		Robot.getLightLocalizer().localize(false);	// localize

	}

}
