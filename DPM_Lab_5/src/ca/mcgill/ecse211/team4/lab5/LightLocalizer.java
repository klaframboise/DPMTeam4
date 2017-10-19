package ca.mcgill.ecse211.team4.lab5;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

	private static final double LS_TO_CENTER = 8.9;
	private static SampleProvider colorSampler;
	private static float[] lightData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	int counter;
	double[] angles;
	double dx;
	double dy;
	double dTheta;

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
		int rotationAngle = ZipLineLab.convertAngle(ZipLineLab.WHEEL_RADIUS, ZipLineLab.TRACK, 360);
		counter = 0;
		angles = new double[4];

		leftMotor.setSpeed(ZipLineLab.ROTATE_SPEED);
		rightMotor.setSpeed(ZipLineLab.ROTATE_SPEED);

		// start rotating 360 deg
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle, true);

		// sample light sensor every 25 ms to detect lines
		do {
			if(getRedIntensity() < ZipLineLab.LINE_RED_INTENSITY) {
				angles[counter++] = ZipLineLab.getOdo().getTheta();
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
			ZipLineLab.getOdo().setX(ZipLineLab.GRID_SIZE + dx);
			ZipLineLab.getOdo().setY(ZipLineLab.GRID_SIZE + dy);
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
			break;
		case 1:
			ZipLineLab.getOdo().setX(7 * ZipLineLab.GRID_SIZE - dy);
			ZipLineLab.getOdo().setY(ZipLineLab.GRID_SIZE + dx);
			ZipLineLab.getOdo().setTheta(Math.PI/2.0 + ZipLineLab.getOdo().getTheta() + dTheta);
			break;
		case 2:
			ZipLineLab.getOdo().setX(7 * ZipLineLab.GRID_SIZE - dx);
			ZipLineLab.getOdo().setY(7 * ZipLineLab.GRID_SIZE - dy);
			ZipLineLab.getOdo().setTheta(Math.PI + ZipLineLab.getOdo().getTheta() + dTheta);
			break;
		case 3:
			ZipLineLab.getOdo().setX(ZipLineLab.GRID_SIZE + dy);
			ZipLineLab.getOdo().setY(7 * ZipLineLab.GRID_SIZE - dx);
			ZipLineLab.getOdo().setTheta(3.0 * Math.PI/2.0 + ZipLineLab.getOdo().getTheta() + dTheta);
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
	 * Performs light localization according to initial localization.
	 * @param initialLocalization
	 */
	public void localize(boolean initialLocalization) {
		
		//signal localization to correction
		ZipLineLab.getCorrection().pauseCorrection();
		
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
			}
			ZipLineLab.getNav().turnTo(0);
			sweep();

			// already in position to localize
			if(counter == 4) {
				dx = -LS_TO_CENTER * Math.cos((angles[2] - angles[0])/2);	//theta-y is difference in angle between the first and third line crossed
				dy =  (-LS_TO_CENTER * Math.cos((angles[3] - angles[1])/2)) - LS_TO_CENTER;	//theta-x is difference in angle between the second and fourth line crossed
				dTheta = -Math.PI/2.0 - (angles[2] - Math.PI) + (angles[2] - angles[0])/2.0;
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
			ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
		}
		
		
		ZipLineLab.getCorrection().resumeCorrection();
	
	}

	/**
	 * Drives the robot in the given direction until a grid line is detected + 4.5 cm.
	 * @param dir x or y
	 */
	private void adjust(char dir) {

		switch(dir) {
		// head in positive x direction
		case 'y': ZipLineLab.getNav().turnTo(Math.PI/2.0); break;
		case 'x': ZipLineLab.getNav().turnTo(0); break;
		default: return;
		}

		// go forward until axis is found
		while(getRedIntensity() > ZipLineLab.LINE_RED_INTENSITY) {
			leftMotor.setSpeed(ZipLineLab.FORWARD_SPEED);
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
