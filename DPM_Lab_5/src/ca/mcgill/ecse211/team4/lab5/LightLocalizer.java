package ca.mcgill.ecse211.team4.lab5;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

	private static final float LINE_RED_INTENSITY = 0.30f;
	private static final double LS_TO_CENTER = 10;
	private SampleProvider colorSampler;
	private float[] lightData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	int counter;
	double[] angles;

	public LightLocalizer(SampleProvider colorSampler, float[] lightData, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.colorSampler = colorSampler;
		this.lightData = lightData;
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
			if(getRedIntensity() < LINE_RED_INTENSITY) {
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
	 * Localizes the robot with respect to its distance from the grid lines.
	 */
	public void localize(boolean angleOnly) {
		double x;
		double y;
		double dTheta;

		while(true) {
			if(angleOnly) {
				ZipLineLab.getNav().turnTo(0);
			}
			else {
				adjust('x');
				adjust('y');
			}
			sweep();

			// already in position to localize
			if(counter == 4) {
				x = -LS_TO_CENTER * Math.cos((angles[2] - angles[0])/2);	//theta-y is difference in angle between the first and third line crossed
				y =  (-LS_TO_CENTER * Math.cos((angles[3] - angles[1])/2)) - LS_TO_CENTER;	//theta-x is difference in angle between the second and fourth line crossed
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
		
		// adjust odometer
		if(!angleOnly) {
			ZipLineLab.getOdo().setX(x);
			ZipLineLab.getOdo().setY(y);
		}
		ZipLineLab.getOdo().setTheta(ZipLineLab.getOdo().getTheta() + dTheta);
	
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
		while(getRedIntensity() > LINE_RED_INTENSITY) {
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
	private float getRedIntensity() {

		for(int i = 0; i < lightData.length; i++) {
			colorSampler.fetchSample(lightData, i);
		}

		Arrays.sort(lightData);

		return (lightData[(lightData.length/2) - 1] + lightData[lightData.length/2]) / 2.0f;
	}
}
