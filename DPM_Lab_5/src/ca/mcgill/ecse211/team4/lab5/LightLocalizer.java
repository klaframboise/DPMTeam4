package ca.mcgill.ecse211.team4.lab5;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

	private static final double LS_TO_CENTER = 8.9;
	private static final double ANGLE_OFFSET = Math.toRadians(7);
	private static SampleProvider colorSampler;
	private static float[] lightData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	int counter;
	double[] angles;
	double dx;
	double dy;
	double dTheta;
	
	// coordinates of the nearest cross section.
	double x_sc;
	double y_sc;
	//coordinates of the start of the zipline
	double x_c;
	double y_c;
	
	public LightLocalizer(SampleProvider colorSampler, float[] lightData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, double x_c, double y_c) {
		LightLocalizer.colorSampler = colorSampler;
		LightLocalizer.lightData = lightData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		counter = 0;
		angles = new double[4];
		this.x_c = x_c;
		this.y_c = y_c;
	}

	/**
	 * Performs a 360 degrees sweep in order to detect grid lines.
	 */
	private void sweep() {
		int rotationAngle = ZipLineLab.convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, 360);
		counter = 0;
		angles = new double[4];

		leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED * ZipLineLab.SPEED_OFFSET);
		rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);

		// start rotating 360 deg
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle, true);

		// sample light sensor every 25 ms to detect lines
		do {
			if (getRedIntensity() < ZipLineLab.LINE_RED_INTENSITY) {
				angles[counter++] = ZipLineLab.getOdo().getTheta();
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
	 * Localizes with consideration for the starting corner.
	 * 
	 * @param startingCorner
	 *            [0,3]
	 */
	public void localize(int startingCorner) {
		localize(true);

		switch (startingCorner) {
		case 0:
			x_sc = ZipLineLab.GRID_SIZE;
			y_sc = ZipLineLab.GRID_SIZE;
			ZipLineLab.getOdo().setX(x_sc + dx);
			ZipLineLab.getOdo().setY(y_sc + dy);
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
			ZipLineLab.getNav().travelTo(x_sc, y_sc, false);
			ZipLineLab.getNav().turnTo(0);
			ZipLineLab.getOdo().setTheta(0);
			break;
		case 1:
			x_sc = 7 * ZipLineLab.GRID_SIZE;
			y_sc = ZipLineLab.GRID_SIZE;
			ZipLineLab.getOdo().setX(x_sc + dx);
			ZipLineLab.getOdo().setY(y_sc + dy);
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
			ZipLineLab.getNav().travelTo(x_sc, y_sc, false);
			ZipLineLab.getNav().turnTo(0);
			ZipLineLab.getOdo().setTheta(3 * Math.PI/2);
			break;
		case 2:
			x_sc = 7 * ZipLineLab.GRID_SIZE;
			y_sc = 7 * ZipLineLab.GRID_SIZE;
			ZipLineLab.getOdo().setX(x_sc + dx);
			ZipLineLab.getOdo().setY(y_sc + dy);
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
			ZipLineLab.getNav().travelTo(x_sc, y_sc, false);
			ZipLineLab.getNav().turnTo(0);
			ZipLineLab.getOdo().setTheta(Math.PI);
			break;
		case 3:
			x_sc = ZipLineLab.GRID_SIZE;
			y_sc = 7 * ZipLineLab.GRID_SIZE;
			ZipLineLab.getOdo().setX(x_sc + dx);
			ZipLineLab.getOdo().setY(y_sc + dy);
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
			ZipLineLab.getNav().travelTo(x_sc, y_sc, false);
			ZipLineLab.getNav().turnTo(0);
			ZipLineLab.getOdo().setTheta(Math.PI/2);
			break;
		}
	}

	/**
	 * Performs light localization by assuming that this is not the initial
	 * localization.
	 */
	public void localize() {
		localize(false);
	}

	/**
	 * Performs light localization according to initial localization.
	 * 
	 * @param initialLocalization
	 */
	public void localize(boolean initialLocalization) {

		// signal localization to correction
		ZipLineLab.getCorrection().pauseCorrection();

		// wait for correction operation to finish
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		while (true) {
			if (initialLocalization) {
				adjust('x');
				adjust('y');
				ZipLineLab.getNav().turnTo(0);
			} else {
				ZipLineLab.getNav().turnTo(7.0 * Math.PI / 4.0);
			}
			sweep();

			// already in position to localize
			if (counter == 4) {
				dx = -LS_TO_CENTER * Math.cos(Math.abs(angles[2] - angles[0]) / 2); // theta-y
																					// is
																					// difference
																					// in
																					// angle
																					// between
																					// the
																					// first
																					// and
																					// third
																					// line
																					// crossed
				dy = (-LS_TO_CENTER * Math.cos(Math.abs(angles[3] - angles[1]) / 2)) - LS_TO_CENTER; // theta-x
																										// is
																										// difference
																										// in
																										// angle
																										// between
																										// the
																										// second
																										// and
																										// fourth
																										// line
																										// crossed
				dTheta = -Math.PI / 2.0 - (angles[2] - Math.PI) + Math.abs(angles[2] - angles[0]) / 2.0 - ANGLE_OFFSET;
				break;
			} else if (counter == 2) {
				// if the second line is detected at theta < 180 deg, it was the
				// y-axis
				if (angles[1] < Math.PI) {
					adjust('x'); // need to go in positive y direction in order
									// to come across x-axis
				} else {
					adjust('y'); // need to go in positive x direction in order
									// to come across y-axis
				}
			}
			// edge case, implement if necessary
			else if (counter == 3 || counter == 1) {
				Sound.buzz();
			} else {
				// no lines were found, go in both positive x and y until axis
				// found
				adjust('x');
				adjust('y');
			}
		}

		// adjust odometer
		if (!initialLocalization) {
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
			ZipLineLab.getOdo().setX(ZipLineLab.getNav().getWaypointX() + dx);
			ZipLineLab.getOdo().setY(ZipLineLab.getNav().getWaypointX() + dy);

		}
		ZipLineLab.getCorrection().resumeCorrection();

	}

	/**
	 * Drives the robot in the given direction until a grid line is detected +
	 * 4.5 cm.
	 * 
	 * @param dir
	 *            x or y
	 */
	private void adjust(char dir) {

		switch (dir) {
		// head in positive x direction
		case 'y':
			ZipLineLab.getNav().turnTo(Math.PI / 2.0);
			break;
		case 'x':
			ZipLineLab.getNav().turnTo(0);
			break;
		default:
			return;
		}

		// go forward for 10cm until axis is found
		// if no line is detected, reverse 15cm. 
		while (getRedIntensity() > ZipLineLab.LINE_RED_INTENSITY) {
			leftMotor.setSpeed(ZipLineLab.FORWARD_SPEED * ZipLineLab.SPEED_OFFSET);
			rightMotor.setSpeed(ZipLineLab.FORWARD_SPEED);

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
		leftMotor.rotate(ZipLineLab.convertDistance(ZipLineLab.WHEEL_RADIUS, 4.5), true);
		rightMotor.rotate(ZipLineLab.convertDistance(ZipLineLab.WHEEL_RADIUS, 4.5), false);

	}

	/**
	 * Returns the mean of a group of sample from the color sensor as the
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
	 * Do another light localization @ (x0, y0). fix the angle and adjust the location.
	 * @param x_0
	 * @param y_0
	 */
	public void lightLocalizeX0Y0(double x_0, double y_0) {
		sweep();
		while (true) {
			if (counter == 4) {
				dx = x_0 - LS_TO_CENTER * Math.cos(Math.abs(angles[2] - angles[0]) / 2); 
				dy = y_0 - LS_TO_CENTER * Math.cos(Math.abs(angles[3] - angles[1]) / 2); 
				dTheta = -Math.PI / 2.0 - (angles[2] - Math.PI) + Math.abs(angles[2] - angles[0]) / 2.0 - ANGLE_OFFSET;
				break;
			} else if (counter == 2) {
				// if the second line is detected at theta < 180 deg, it was the
				// y-axis
				if (angles[1] < Math.PI) {
					adjust('x'); // need to go in positive y direction in order
									// to come across x-axis
				} else {
					adjust('y'); // need to go in positive x direction in order
									// to come across y-axis
				}
			}
			// TODO: edge case counter == 3, implement if necessary
			 else {
				// no lines were found, go in both positive x and y until axis
				// found
				adjust('x');
				adjust('y');
			}
		}
		
		ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
		ZipLineLab.getOdo().setX(ZipLineLab.getNav().getWaypointX() + dx);
		ZipLineLab.getOdo().setY(ZipLineLab.getNav().getWaypointY() + dy);
		
		LocalEV3.get().getTextLCD().drawString(String.valueOf(dx), 0, 5);
		LocalEV3.get().getTextLCD().drawString(String.valueOf(dy), 0, 6);
		if(dx < 1 && dy < 1){
			if(x_c > x_0 && y_c == y_0)
				ZipLineLab.getNav().turnTo(Math.PI/2);
			else if(x_c < x_0 && y_c == y_0)
				ZipLineLab.getNav().turnTo(3*Math.PI/2);
			else if(y_c > y_0 && x_c == x_0)
				ZipLineLab.getNav().turnTo(0);
			else if(y_c < y_0 && x_c == x_0)
				ZipLineLab.getNav().turnTo(Math.PI);			
		}
		else {
			if(x_c > x_0 && y_c == y_0)
				ZipLineLab.getNav().turnTo(Math.PI/2);
			else if(x_c < x_0 && y_c == y_0)
				ZipLineLab.getNav().turnTo(3*Math.PI/2);
			else if(y_c > y_0 && x_c == x_0)
				ZipLineLab.getNav().turnTo(0);
			else if(y_c < y_0 && x_c == x_0)
				ZipLineLab.getNav().turnTo(Math.PI);			
			
			ZipLineLab.getNav().travelTo(ZipLineLab.getNav().getWaypointX(), ZipLineLab.getNav().getWaypointY(), false);
		}
		
	}
}
