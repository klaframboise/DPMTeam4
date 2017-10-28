package ca.mcgill.ecse211.team4.localization;

import java.util.Arrays;

import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class uses the color sensor to correct to odometer's x, y and theta by evaluating the distance
 * of the robot to a cross-section in the grid.
 * @author Kevin Laframboise & Wenjie Wei
 *
 */
public class LightLocalizer {

	/**
	 * Distance between the light sensor and the center of rotation of the robot, in cm.
	 */
	private static final double LS_TO_CENTER = 8.9;
	/**
	 * Offset to compensate for consistent error in angle correction, in radian. 
	 * Should be empirically determined.
	 */
	private static final double ANGLE_OFFSET = Math.toRadians(7);
	
	/**
	 * Sample provider for the color sensor. Sample Provider must be in red mode.
	 */
	private static SampleProvider colorSampler;
	
	/**
	 * Buffer array for the color sensor data.
	 */
	private static float[] lightData;
	
	/**
	 * Motors driving the robot, used in the sweep operation.
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	/**
	 * Counts the number of lines detected during sweep operation.
	 */
	int counter;
	
	/**
	 * Records the angle at which a line was detected.
	 */
	double[] angles;
	
	/**
	 * Distance in x between the robot and the y-axis, in cm.
	 */
	double dx;
	
	/**
	 * Distance in y between the robot and the x-axis, in cm.
	 */
	double dy;
	
	/**
	 * Angle delta between the odometer's zero heading and true zero heading, in radians.
	 */
	double dTheta;

	/**
	 * Creates a LightLocalizer object with given properties.
	 * @param colorSampler
	 * @param lightData
	 * @param leftMotor
	 * @param rightMotor
	 */
	public LightLocalizer(SampleProvider colorSampler, float[] lightData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		LightLocalizer.colorSampler = colorSampler;
		LightLocalizer.lightData = lightData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		counter = 0;
		angles = new double[4];
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

		// start rotating 360 deg
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle, true);

		// sample light sensor every 25 ms to detect lines
		do {
			if(getRedIntensity() < Robot.LINE_RED_INTENSITY) {
				angles[counter++] = Robot.getOdo().getTheta();
				Sound.beep();
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		} while(leftMotor.isMoving() && rightMotor.isMoving() && counter < angles.length);
		leftMotor.stop(true);
		rightMotor.stop(true);

	}
	
	/**
	 * Localizes with consideration for the starting corner.
	 * @param startingCorner [0,3]
	 */
	public void localize(int startingCorner) {
		localize(true);
		
		switch(startingCorner) {
		case 0:
			Robot.getOdo().setX(Robot.GRID_SIZE + dx);
			Robot.getOdo().setY(Robot.GRID_SIZE + dy);
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta);
			break;
		case 1:
			Robot.getOdo().setX(7 * Robot.GRID_SIZE + dy);
			Robot.getOdo().setY(Robot.GRID_SIZE - dx);
			Robot.getOdo().setTheta(3.0 * Math.PI/2.0 + Robot.getOdo().getTheta() + dTheta);
			break;
		case 2:
			Robot.getOdo().setX(7 * Robot.GRID_SIZE - dx);
			Robot.getOdo().setY(7 * Robot.GRID_SIZE - dy);
			Robot.getOdo().setTheta(Math.PI + Robot.getOdo().getTheta() + dTheta);
			break;
		case 3:
			Robot.getOdo().setX(Robot.GRID_SIZE - dy);
			Robot.getOdo().setY(7 * Robot.GRID_SIZE + dx);
			Robot.getOdo().setTheta(Math.PI/2.0 + Robot.getOdo().getTheta() + dTheta);
			break;
		}
	}
	
	/**
	 * Performs light localization by assuming that this is not the initial localization.
	 */
	public void localize() {
		localize(false);
	}

	/**
	 * Performs light localization according to whether this is the initial localization or not.
	 * @param initialLocalization true if this is the first light localization performed.
	 */
	public void localize(boolean initialLocalization) {
		
		//signal localization to correction
		Robot.getCorrection().pauseCorrection();
		
		//wait for correction operation to finish
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		while(true) {
			if (initialLocalization) {
				adjust('x');
				adjust('y');
				Robot.getNav().turnTo(0);
			}
			else  {
				Robot.getNav().turnTo(7.0 * Math.PI / 4.0);
			}
			sweep();

			// already in position to localize
			if(counter == 4) {
				dx = -LS_TO_CENTER * Math.cos(Math.abs(angles[2] - angles[0])/2);	//theta-y is difference in angle between the first and third line crossed
				dy =  (-LS_TO_CENTER * Math.cos(Math.abs(angles[3] - angles[1])/2)) - LS_TO_CENTER;	//theta-x is difference in angle between the second and fourth line crossed
				dTheta = -Math.PI/2.0 - (angles[2] - Math.PI) + Math.abs(angles[2] - angles[0])/2.0 - ANGLE_OFFSET;
				break;
			}
			else if(counter == 2) {
				// if the second line is detected at theta < 180 deg, it was the y-axis
				if(angles[1] < Math.PI) {
					adjust('x');	// need to go in positive y direction in order to come across x-axis
				}
				else {
					adjust('y');	// need to go in positive x direction in order to come across y-axis
				}
			}
			//edge case, implement if necessary
			else if(counter == 3 || counter == 1) {
				Sound.buzz();
			}
			else {
				// no lines were found, go in both positive x and y until axis found
				adjust('x');
				adjust('y');
			}
		}
		
		//adjust odometer
		if(!initialLocalization) {
			Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta);
			Robot.getOdo().setX(Robot.getNav().getWaypointX() + dx);
			Robot.getOdo().setY(Robot.getNav().getWaypointX() + dy);

		}
		
		Robot.getCorrection().resumeCorrection();
	
	}

	/**
	 * Drives the robot in the given direction until a grid line is detected + 4.5 cm.
	 * @param dir x or y
	 */
	private void adjust(char dir) {

		switch(dir) {
		// head in positive x direction
		case 'y': Robot.getNav().turnTo(Math.PI/2.0); break;
		case 'x': Robot.getNav().turnTo(0); break;
		default: return;
		}

		// go forward until axis is found
		while(getRedIntensity() > Robot.LINE_RED_INTENSITY) {
			leftMotor.setSpeed(Robot.FORWARD_SPEED * Robot.SPEED_OFFSET);
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
		
		// go 4.5cm past the line
		leftMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 4.5), true);
		rightMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 4.5), false);

	}

	/**
	 * Returns the mean of a group of sample from the color sensor as the detected color.
	 * @return color intensity detected
	 */
	public static float getRedIntensity() {

		for(int i = 0; i < lightData.length; i++) {
			colorSampler.fetchSample(lightData, i);
		}

		Arrays.sort(lightData);

		return (lightData[(lightData.length/2) - 1] + lightData[lightData.length/2]) / 2.0f;
	}
}
