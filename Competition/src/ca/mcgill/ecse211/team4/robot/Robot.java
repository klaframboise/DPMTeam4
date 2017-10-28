package ca.mcgill.ecse211.team4.robot;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.odometry.OdometryCorrection;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Main class.
 * @author Kevin Laframboise
 *
 */
public class Robot {

	/**
	 * Speed at which the robot is driven when going straight.
	 */
	public static final int FORWARD_SPEED = 0;
	
	/**
	 * Speed at which the robot is driven when rotating.
	 */
	public static final int ROTATE_SPEED = 0;
	
	/**
	 * Offset to compensate for mechanical design weight imbalance.
	 */
	public static final float SPEED_OFFSET = 0.986f;
	
	/**
	 * Size of the grid.
	 */
	public static final double GRID_SIZE = 0;
	
	/**
	 * Red intensity of a grid line.
	 */
	public static final float LINE_RED_INTENSITY = 0;
	
	/**
	 * Track width of the robot.
	 */
	public static final double TRACK = 0;
	
	/**
	 * Radius of the robot's wheels.
	 */
	public static final double WHEEL_RADIUS = 0;
	
	/**
	 * The robot's navigation.
	 */
	private static Navigation nav;
	
	/**
	 * The robot's odometer.
	 */
	private static Odometer odo;
	
	/**
	 * The robot's odometry correction.
	 */
	private static OdometryCorrection correction;
	
	/**
	 * Map containing the game parameters.
	 */
	private static Map<String, Integer> gameParameters;
	
	/**
	 * Can either be "green" or "red. This will affect strategies.
	 */
	private static String teamColor;
	
	/**
	 * Instance of the left driving motor.
	 */
	private static EV3LargeRegulatedMotor leftMotor;
	
	/**
	 * Instance of the right driving motor.
	 */
	private static EV3LargeRegulatedMotor rightMotor;
	
	/**
	 * Instance of the zip line motor.
	 */
	private static EV3LargeRegulatedMotor lineMotor;
	
	/**
	 * Instance of the servo motor.
	 */
	private static EV3LargeRegulatedMotor servo;

	/**
	 * Initialize components and sets up game. 
	 * @param args
	 */
	public void main(String[] args) {
		// TODO Auto-generated method stub
	}
	
	/**
	 * 
	 * @return odometry correction
	 */
	public static OdometryCorrection getCorrection() {
		return correction;
	}

	/**
	 * 
	 * @return nav
	 */
	public static Navigation getNav() {
		return nav;
	}

	/**
	 * 
	 * @return odo
	 */
	public static Odometer getOdo() {
		return odo;
	}
	
	/**
	 * 
	 * @return gameParameters
	 */
	public static Map getGameParameters() {
		return gameParameters; 
	}

	/**
	 * 
	 * @return teamColor
	 */
	public static String getTeamColor() {
		return teamColor;
	}

	/**
	 * 
	 * @return array containing driving motors. Index 0 contains the leftMotor, 1, the rightMotor.
	 */
	public static EV3LargeRegulatedMotor[] getDrivingMotors() {
		EV3LargeRegulatedMotor[] motors = {leftMotor, rightMotor};
		return motors;
	}

	/**
	 * 
	 * @return lineMotor
	 */
	public static EV3LargeRegulatedMotor getLineMotor() {
		return lineMotor;
	}

	/**
	 * 
	 * @return servo
	 */
	public static EV3LargeRegulatedMotor getServo() {
		return servo;
	}

}
