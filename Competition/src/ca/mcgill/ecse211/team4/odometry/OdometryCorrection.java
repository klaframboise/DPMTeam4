/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.team4.odometry;

import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;

/**
 * This thread corrects the odometer's x, y and theta using the color sensor to detect grid lines.
 * @author Kevin Laframboise
 *
 */
public class OdometryCorrection extends Thread {

	/* Constants */
	/**
	 * Sleeping time between each odometer correction, in ms.
	 */
	private static final long CORRECTION_PERIOD = 10;

	/**
	 * Odometer to correct.
	 */
	private Odometer odo;
	
	/**
	 * Indicates whether the robot is currently performing localization.
	 */
	private boolean isLocalizing;

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odo = odometer;
	}

	/**
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		long correctionStart, correctionEnd;
		double gridX = 0;
		double gridY = 0;

		while (true) {
			correctionStart = System.currentTimeMillis();

			if(LightLocalizer.getRedIntensity() < Robot.LINE_RED_INTENSITY && !isLocalizing) {
				Sound.beep();

				// get grid coordinate
				gridX = odo.getX() / Robot.GRID_SIZE;
				gridY = odo.getY() / Robot.GRID_SIZE;
				
				// correct the coordinate which most likely corresponds to the axis detected
				if(gridX % 1 <  gridY % 1) {
					odo.setX(Math.round(gridX) * Robot.GRID_SIZE);
				}
				else {
					odo.setY(Math.round(gridY) * Robot.GRID_SIZE);
				}
				
				//update nav to recompute path
				Robot.getNav().update();
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	
	/**
	 * Pauses the correction.
	 */
	public void pauseCorrection() {
		isLocalizing = true;
	}
	
	/**
	 * Resumes the correction.
	 */
	public void resumeCorrection() {
		isLocalizing = false;
	}

}
