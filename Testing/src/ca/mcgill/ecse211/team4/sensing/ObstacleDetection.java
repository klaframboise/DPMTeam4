package ca.mcgill.ecse211.team4.sensing;

import ca.mcgill.ecse211.team4.drivers.Navigation;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class detects and avoids obstacles.
 * @author Kevin Laframboise
 *
 */
public class ObstacleDetection extends Thread {

	/**
	 * Sample provider for the ultrasonic localizer.
	 */
	private SampleProvider us;

	/**
	 * Buffer array for ultrasonic data.
	 */
	private float[] usData;
	
	/**
	 * Navigation to interrupt in the case where an obstacle needs to be avoided.
	 */
	private Navigation nav;
	
	/**
	 * Motors driving the robot.
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	/**
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		
	}
}
