package ca.mcgill.ecse211.team4.robot;

import java.util.Arrays;

import lejos.robotics.SampleProvider;

/**
 * This class provides helpful methods that can be used throughout the project. All the methods in this are static.
 * This is to avoid code duplication and consolidate methods that are used often by different classes.
 * @author Kevin Laframboise
 *
 */
public abstract class Helper {

	/**
	 * This method converts a change in heading to a rotation amount to be requested to the motors.
	 * This method was provided during lab 2 and is used as-is.
	 * @param wheelRadius radius of the robot's wheel.
	 * @param track track width of the robot.
	 * @param angle change in heading required.
	 * @return wheel rotation amount, in degrees.
	 */
	public static int convertAngle(double wheelRadius, double track, double angle) {
		return convertDistance(wheelRadius, Math.PI * track * angle / 360.0);
	}

	/**
	 * This method converts a distance to a rotation amount to be requested to the motors.
	 * This method was provided during lab 2 and is used as-is.
	 * @param wheelRadius radius of the robot's wheel.
	 * @param distance to be traveled.
	 * @return rotation amount, in degrees.
	 */
	public static int convertDistance(double wheelRadius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * wheelRadius));
	}
	
	/**
	 * Returns the median of a group of sample from the ultrasonic sensor.
	 * @param us SampleProvider to be used by the method.
	 * @param usData buffer array for sensor data.
	 * @return distance from the wall, in cm.
	 */
	public static int getFilteredDistance(SampleProvider us, float[] usData) {

		for(int i = 0; i < usData.length; i+= us.sampleSize()) {
			us.fetchSample(usData, i * us.sampleSize()); // acquire data
		}
		Arrays.sort(usData);	// sort array
		return (int) ((usData[(usData.length/2)-1] + usData[usData.length/2]) / 2.0 * 100.0); // return median
	}
	
	/**
	 * Converts a pair of "upper right-lower left" coordinates to a list of coordinates representing
	 * the four corners of a rectangular area.
	 * @param upperRightX
	 * @param upperRightY
	 * @param lowerLeftX
	 * @param lowerLeftY
	 * @return a bidimensional array containing the coordinates of the four corners of a rectangular area.
	 * 		   The array is structured as follows: coords[i][j], i cycles corners from upper right, to lower right,
	 * 		   to lower left, to upper left. j = 0 is the x coordinate, j = 1, the y.
	 */
	public static int[][] convertToFourCorners(int upperRightX, int upperRightY, int lowerLeftX, int lowerLeftY) {
		int coords[][] = new int[4][2];
		coords[0][0] = upperRightX;
		coords[0][1] = upperRightY;
		coords[1][0] = upperRightX;
		coords[1][1] = lowerLeftY;
		coords[2][0] = lowerLeftX;
		coords[2][1] = lowerLeftY;
		coords[3][0] = lowerLeftX;
		coords[3][1] = upperRightY;
		return coords;
	}

}
