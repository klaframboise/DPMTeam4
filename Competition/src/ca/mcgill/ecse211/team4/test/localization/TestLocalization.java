package ca.mcgill.ecse211.team4.test.localization;

import java.io.IOException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.robot.Display;
import ca.mcgill.ecse211.team4.robot.GameSetup;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import ca.mcgill.ecse211.team4.sensing.FlagDetection;
import ca.mcgill.ecse211.team4.strategy.NavigationStrategy;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * Main class.
 * @author Kevin Laframboise
 *
 */
public class TestLocalization {

	/**
	 * Team number.
	 */
	public static final int TEAM_NUMBER = 4;
	/**
	 * Speed at which the robot is driven when going straight.
	 */
	public static final int FORWARD_SPEED = 200;
	
	/**
	 * Speed at which the robot is driven when rotating.
	 */
	public static final int ROTATE_SPEED = 150;
	
	/**
	 * Offset to compensate for mechanical design weight imbalance.
	 */
	public static final float SPEED_OFFSET = 0.997f;
	
	/**
	 * Size of the grid.
	 */
	public static final double GRID_SIZE = 30.48;
	
	/**
	 * Red intensity of a grid line.
	 */
	public static final float LINE_RED_INTENSITY = 0.30f;
	
	/**
	 * Track width of the robot.
	 */
	public static final double TRACK = 13.85;
	
	/**
	 * Radius of the robot's wheels.
	 */
	public static final double WHEEL_RADIUS = 2.1;
	
	/**
	 * The robot's navigation.
	 */
	private static Navigation nav;
	
	/**
	 * The robot's odometer.
	 */
	private static Odometer odo;
	
	/**
	 * The robot's light localizer.
	 */
	private static LightLocalizer lightLocalizer;
	
	/**
	 * Map containing the game parameters.
	 */
	private static Map<String, Integer> gameParameters;
	
	/**
	 * Can either be "Green" or "Red". This will affect strategies.
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
	@SuppressWarnings("resource")	//resources are used for the full life cycle of the program, no need to close
	public static void main(String[] args) {
		
		Robot robot = new Robot();
		
		int x_0, y_0, x_d, y_d;
		/* Initialize navigation strategy */
//		NavigationStrategy navStrat = new NavigationStrategy(teamColor, gameParameters, nav);
		
		/* Initialize flag detection */
//		FlagDetection flagDetection = new FlagDetection(colorSampler, colorData, gameParameters.get("O" + teamColor.charAt(0)).intValue(), gameParameters);*/
				
		/* Start testing */
		Robot.getOdo().start();
		Robot.getNav().start();
		Display display = new Display(Robot.getOdo());
		display.start();
		/* The following section commands the robot to turn 1800 degrees. */
//		int rotateAmount = Helper.convertAngle(WHEEL_RADIUS, TRACK, 1800);
//		LocalEV3.get().getTextLCD().drawString(String.valueOf(rotateAmount), 0, 4);
//		Robot.getDrivingMotors()[0].setSpeed(100 * SPEED_OFFSET);
//		Robot.getDrivingMotors()[1].setSpeed(100);
//		Robot.getDrivingMotors()[0].rotate(rotateAmount, true);
//		Robot.getDrivingMotors()[1].rotate((int) (-rotateAmount / SPEED_OFFSET), false);
		
		/* Localize */
		Robot.getUSLocalizer().localize();
		Robot.getLightLocalizer().localize(Robot.getGameParameters().get("GreenCorner").intValue());
		
		x_0 = Robot.getGameParameters().get("ZO_G_x").intValue();
		y_0 = Robot.getGameParameters().get("ZO_G_y").intValue();
		x_d = Robot.getGameParameters().get("SR_LL_x").intValue();
		y_d = Robot.getGameParameters().get("SR_LL_y").intValue();
		/* Tester of some navigation strategy to avoid the zipline */
		if(Math.abs((y_0 * GRID_SIZE - Robot.getOdo().getY())) > Math.abs((x_0 * GRID_SIZE - Robot.getOdo().getX()))){
			Robot.getNav().travelTo(Robot.getOdo().getX(), y_0 * GRID_SIZE, false);
			Robot.getLightLocalizer().localize(false);
			Robot.getNav().travelTo((x_0 - 0.1) * GRID_SIZE, (y_0 - 0.1) * GRID_SIZE, false);
		} else { 
			Robot.getNav().travelTo(x_0 * GRID_SIZE, Robot.getOdo().getY(), false);
			Robot.getLightLocalizer().localize(false);
			Robot.getNav().travelTo((x_0 - 0.1) * GRID_SIZE, (y_0 - 0.1) * GRID_SIZE, false);
		}
		Robot.getLightLocalizer().localize(false);
		Robot.getNav().travelTo(Robot.getLightLocalizer().getGridX(), Robot.getLightLocalizer().getGridY(), false);
		
		Robot.getZipLineTraversal().traverse();
		Robot.getNav().travelTo(x_d * Robot.GRID_SIZE, y_d * Robot.GRID_SIZE, false);
		Robot.getLightLocalizer().localize(false);
		Robot.getNav().travelTo(x_d * GRID_SIZE, y_d * GRID_SIZE, false);	
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
	 * @return {@link Robot#lightLocalizer}
	 */
	public static LightLocalizer getLightLocalizer() {
		return lightLocalizer;
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