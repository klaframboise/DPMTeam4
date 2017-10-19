/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.Sound;

public class OdometryCorrection extends Thread {

	/* Constants */
	private static final long CORRECTION_PERIOD = 10;

	private Odometer odo;
	private boolean isLocalizing;

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odo = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		double gridX = 0;
		double gridY = 0;

		while (true) {
			correctionStart = System.currentTimeMillis();

			if(LightLocalizer.getRedIntensity() < ZipLineLab.LINE_RED_INTENSITY && !isLocalizing) {
				Sound.beep();

				// get grid coordinate
				gridX = odo.getX() / ZipLineLab.GRID_SIZE;
				gridY = odo.getY() / ZipLineLab.GRID_SIZE;
				
				// assuming odometer is accurate within +/- 3cm
				if((gridX % 1 < 0.1 || gridX % 1 > 0.9) || (gridX % 1 < gridY % 1)) {
					odo.setX((int)gridX * ZipLineLab.GRID_SIZE);
				}
				// assuming odometer is accurate within +/- 3cm
				if((gridY % 1 < 0.1 || gridY % 1 > 0.9) || (gridY % 1 < gridX % 1)) {
					odo.setY((int)gridY * ZipLineLab.GRID_SIZE);
				}
				ZipLineLab.getNav().update();
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
	
	public void pauseCorrection() {
		isLocalizing = true;
	}
	
	public void resumeCorrection() {
		isLocalizing = false;
	}

}
