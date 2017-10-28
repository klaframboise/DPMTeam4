package ca.mcgill.ecse211.team4.robot;

/**
 * This class provides helpful methods that can be used throughout the project. All the methods in this are static.
 * This is to avoid code duplication and consolidate methods that are used often by different classes.
 * @author Kevin
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

}
