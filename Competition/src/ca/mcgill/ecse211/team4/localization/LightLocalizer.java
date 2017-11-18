package ca.mcgill.ecse211.team4.localization;

import java.util.Arrays;

import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class uses the color sensor to correct to odometer's x, y and theta by
 * evaluating the distance of the robot to a cross-section in the grid.
 * 
 * @author Kevin Laframboise & Wenjie Wei
 *
 */
public class LightLocalizer {

	/**
	 * Distance between the light sensor and the center of rotation of the
	 * robot, in cm.
	 */
	private static final double LS_TO_CENTER = 16.5;

	/**
	 * Offset to compensate for consistent error in angle correction, in radian.
	 * Should be empirically determined.
	 */
	private static final double ANGLE_OFFSET = 0;

	/**
	 * Sample provider for the color sensor. Sample Provider must be in red
	 * mode.
	 */
	private static SampleProvider colorSampler;

	/**
	 * Buffer array for the color sensor data.
	 */
	private static float[] lightData;

	/**
	 * Motors driving the robot, used in the sweep operation.
	 */
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;

	/**
	 * Counts the number of lines detected during sweep operation.
	 */
	private int counter;

	/**
	 * Records the angle at which a line was detected.
	 */
	private double[] angles;

	/**
	 * Distance in x between the robot and the y-axis, in cm.
	 */
	private double dx;

	/**
	 * Distance in y between the robot and the x-axis, in cm.
	 */
	private double dy;

	/**
	 * Angle delta between the odometer's zero heading and true zero heading, in
	 * radians.
	 */
	private double dTheta;

	/**
	 * Debugging flag.
	 */
	private boolean debug;

	/**
	 * X coordinate somewhere on the board used for localization after
	 * navigation
	 */
	private double gridX;

	/**
	 * Y coordinate somewhere on the board used for localization after
	 * navigation
	 */
	private double gridY;

	/**
	 * Creates a LightLocalizer object with given properties.
	 * 
	 * @param colorSampler
	 * @param lightData
	 * @param leftMotor
	 * @param rightMotor
	 * @param debug
	 */
	public LightLocalizer(SampleProvider colorSampler, float[] lightData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, boolean debug) {
		LightLocalizer.colorSampler = colorSampler;
		LightLocalizer.lightData = lightData;
		LightLocalizer.leftMotor = Robot.getDrivingMotors()[0];
		LightLocalizer.rightMotor = Robot.getDrivingMotors()[1];
		this.debug = debug;
		counter = 0;
		angles = new double[4];
	}

	/**
	 * Creates a LightLocalizer object with given properties. Sets debug to
	 * false.
	 * 
	 * @param colorSampler
	 * @param lightData
	 * @param leftMotor
	 * @param rightMotor
	 */
	public LightLocalizer(SampleProvider colorSampler, float[] lightData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		new LightLocalizer(colorSampler, lightData, leftMotor, rightMotor, false);
	}

	/**
	 * Performs a 360 degrees sweep in order to detect grid lines.
	 */
	private void sweep() {
		int rotationAngle = Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, 360);
		counter = 0;
		angles = new double[4];

		leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET);
		rightMotor.setSpeed(Robot.ROTATE_SPEED);

		/* Start rotating 360 deg */
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle, true);

		/* sample light sensor every 25 ms to detect lines */
		do {
			if (getRedIntensity() < Robot.LINE_RED_INTENSITY) {
				angles[counter++] = Robot.getOdo().getTheta();
				Sound.beep();
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		} while (leftMotor.isMoving() && rightMotor.isMoving() && counter < angles.length);
		leftMotor.stop(true);
		rightMotor.stop(true);

	}

	/**
	 * Localizes with consideration for the starting corner. Using this method
	 * will cause the robot to move until the color sensor crosses the next grid
	 * line intersection in the positive x and y axis. Be careful when using
	 * this near forbidden areas.
	 * 
	 * @param startingCorner
	 *            [1,4].
	 * @return true if localization is successful.
	 */
	public boolean localize(int startingCorner) {
		boolean isSuccessful = localize(true);

		if (!isSuccessful)
			return isSuccessful;

		switch (startingCorner) {
		case 0:
			if (debug) {
				System.out.println("Setting x to: " + (Robot.GRID_SIZE + dx));
				System.out.println("Setting y to: " + (Robot.GRID_SIZE + dy));
				System.out.println("Setting theta to: " + (Robot.getOdo().getTheta() + dTheta - Math.PI));
				Button.waitForAnyPress();
			}
			Robot.getOdo().setX(Robot.GRID_SIZE + dx);
			Robot.getOdo().setY(Robot.GRID_SIZE + dy);
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);
			Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
			Robot.getNav().turnTo(0);
			break;
		case 1:
			// Robot.getOdo().setX(11 * Robot.GRID_SIZE + dy);
			// set x to 7 for beta demo.
			Robot.getOdo().setX(Robot.GRID_SIZE + dx);
			Robot.getOdo().setY(Robot.GRID_SIZE + dy);
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);
			Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
			Robot.getNav().turnTo(0);
			Robot.getOdo().setX(7 * Robot.GRID_SIZE);
			Robot.getOdo().setY(1 * Robot.GRID_SIZE);
			Robot.getOdo().setTheta(3 * Math.PI / 2);
			if (debug) {
				System.out.println("x: " + (Robot.getOdo().getX()));
				System.out.println("y: " + (Robot.getOdo().getY()));
				System.out.println("theta: " + (Math.toDegrees(Robot.getOdo().getTheta())));
				Button.waitForAnyPress();
			}
			break;
		case 2:
			// Robot.getOdo().setX(11 * Robot.GRID_SIZE - dx);
			// Robot.getOdo().setY(11 * Robot.GRID_SIZE - dy);
			Robot.getOdo().setX(Robot.GRID_SIZE + dx);
			Robot.getOdo().setY(Robot.GRID_SIZE + dy);
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);
			Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
			Robot.getNav().turnTo(0);
			Robot.getOdo().setX(7 * Robot.GRID_SIZE);
			Robot.getOdo().setY(7 * Robot.GRID_SIZE);
			Robot.getOdo().setTheta(Math.PI);
			break;
		case 3:
			Robot.getOdo().setX(Robot.GRID_SIZE + dx);
			Robot.getOdo().setY(Robot.GRID_SIZE + dy);
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);
			Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
			Robot.getNav().turnTo(0);
			Robot.getOdo().setX(1 * Robot.GRID_SIZE);
			Robot.getOdo().setY(7 * Robot.GRID_SIZE);
			Robot.getOdo().setTheta(Math.PI / 2);
			break;
		}

		return isSuccessful;
	}

	/**
	 * Performs light localization by assuming that this is not the initial
	 * localization. Using this method will cause the robot to move until the
	 * color sensor crosses the next grid line intersection in the positive x
	 * and y axis. Be careful when using this near forbidden areas.
	 * 
	 * @return true if localization is successful.
	 */
	public boolean localize() {
		return localize(false);
	}

	/**
	 * Performs light localization according to whether this is the initial
	 * localization or not. Using this method will cause the robot to move until
	 * the color sensor crosses the next grid line intersection in the positive
	 * x and y axis. Be careful when using this near forbidden areas.
	 * 
	 * @param initialLocalization
	 *            true if this is the first light localization performed.
	 * @return true if localization was successful.
	 */
	public boolean localize(boolean initialLocalization) {

		// signal localization to correction
		Robot.getCorrection().pauseCorrection();

		// wait for correction operation to finish
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		/* Make sure robot is in 3rd quadrant of a grid intersection. */
		/* Remove adjust step if initialLocalization */
		if(!initialLocalization){
			adjust('x');
			adjust('y');
		} else if (initialLocalization){
			adjust('o');
		}

		/* Acquire angle difference between axes */
		sweep();

		/* Compute real position */
		if (counter == 4) {
			dx = -LS_TO_CENTER * Math.cos(Math.abs(angles[3] - angles[1]) / 2); 
			// theta-y is difference in angle between the second and fourth line crossed
			dy = -LS_TO_CENTER * Math.cos(Math.abs(angles[0] - angles[2]) / 2); 
			// theta-x is difference in angle between the first and third line crossed
			// dTheta = -Math.PI/2.0 - (angles[3] - Math.PI) +
			// Math.abs(angles[3] - angles[1])/2.0 - ANGLE_OFFSET;
			dTheta = -Math.PI / 2.0 - (angles[3] - Math.PI) + Math.abs(angles[3] - angles[1]) / 2.0;
			// TODO: double check if the logic here is correct.
			/* Print debug info */
			if (debug) {
				System.out.println("x: " + Robot.getOdo().getX());
				System.out.println("y: " + Robot.getOdo().getY());
				System.out.println("Theta: " + Robot.getOdo().getTheta());
				System.out.println("dx: " + dx);
				System.out.println("dy: " + dy);
				System.out.println("dTheta: " + Math.toDegrees(dTheta));
				Button.waitForAnyPress();
			}
		} else {
			return false; // at least one axis was not detected, cannot perform
							// localization
		}

		/*
		 * Only adjust odometer if this is not the first localization because
		 * starting corner has to be taken into consideration while localizing
		 * for the first time
		 */
		if (!initialLocalization) {

			/* Figure out current x,y position w.r.t. the grid */
			gridX = Math.round(Robot.getOdo().getX() / Robot.GRID_SIZE) * Robot.GRID_SIZE;
			gridY = Math.round(Robot.getOdo().getY() / Robot.GRID_SIZE) * Robot.GRID_SIZE;

			/* Print debug info */
			if (debug) {
				System.out.println("gridX: " + gridX);
				System.out.println("gridY: " + gridY);
				System.out.println("dx: " + dx);
				System.out.println("dy: " + dy);
				System.out.println("dtheta in deg: " + Math.toDegrees(dTheta));
				System.out.println("Odo theta reading before reset: " + Math.toDegrees(Robot.getOdo().getTheta()));
				Button.waitForAnyPress();
			}

			/* Correct odometer */
			if (debug) {
				System.out.println("Setting X to : " + (gridX + dx));
				System.out.println("Setting Y to : " + (gridY + dy));
				System.out.println("Setting Theta to : " + (Robot.getOdo().getTheta() + dTheta + Math.PI));
			}
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta + Math.PI);
			Robot.getOdo().setX(gridX + dx);
			Robot.getOdo().setY(gridY + dy);
		}

		Robot.getCorrection().resumeCorrection(); // resume correction

		return true; // localization was successful

	}

	/**
	 * Drives the robot towards the given axis until a grid line is detected
	 * then backwards 4.5 cm.
	 * 
	 * @param axis
	 *            x or y
	 */
	private void adjust(char axis) {

		switch (axis) {
		case 'y':
			Robot.getNav().turnTo(Math.PI / 2.0);
			break; // y-axis is in positive y direction
		case 'x':
			Robot.getNav().turnTo(0);
			break; // x-axis is in positive x direction
		case 'o':
			Robot.getNav().turnTo(Math.PI / 4.0);
			break; // Origin is to the upper right corner
		default:
			return;
		}

		/* go forward until axis is found */
		while (getRedIntensity() > Robot.LINE_RED_INTENSITY) {
			leftMotor.setSpeed(Robot.FORWARD_SPEED);
			rightMotor.setSpeed(Robot.FORWARD_SPEED);

			leftMotor.forward();
			rightMotor.forward();

			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		/* go back 4.5cm so that color sensor is in 3rd quadrant */
		leftMotor.rotate(-Helper.convertDistance(Robot.WHEEL_RADIUS, 21), true);
		rightMotor.rotate(-Helper.convertDistance(Robot.WHEEL_RADIUS, 21), false);

	}

	/**
	 * Returns the median of a group of sample from the color sensor as the
	 * detected color.
	 * 
	 * @return color intensity detected
	 */
	public static float getRedIntensity() {

		for (int i = 0; i < lightData.length; i++) {
			colorSampler.fetchSample(lightData, i);
		}

		Arrays.sort(lightData);

		return (lightData[(lightData.length / 2) - 1] + lightData[lightData.length / 2]) / 2.0f;
	}

	/**
	 * Returns gridX
	 */
	public double getGridX() {
		return gridX;
	}

	/**
	 * Returns gridY
	 */
	public double getGridY() {
		return gridY;
	}
}
