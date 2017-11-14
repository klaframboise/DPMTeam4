package ca.mcgill.ecse211.team4.test.loclaization;

import java.io.IOException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.odometry.OdometryCorrection;
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
	 * The robot's odometry correction.
	 */
	private static OdometryCorrection correction;
	
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
		
		int x_0, y_0;
		/* Initialize navigation strategy */
//		NavigationStrategy navStrat = new NavigationStrategy(teamColor, gameParameters, nav);
		
		/* Initialize flag detection */
//		FlagDetection flagDetection = new FlagDetection(colorSampler, colorData, gameParameters.get("O" + teamColor.charAt(0)).intValue(), gameParameters);*/
				
		/* Start testing */
		LocalEV3.get().getTextLCD().drawString("This is the tester.", 0, 0);
		LocalEV3.get().getTextLCD().drawString("Press Enter to continue.", 0, 1);
		Button.waitForAnyPress();
		
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
		Robot.getLightLocalizer().localize(1);
		System.out.println(Robot.getOdo().getX());
		System.out.println(Robot.getOdo().getY());
		System.out.println(Math.toDegrees(Robot.getOdo().getTheta()));
		
		x_0 = 2;
		y_0 = 1;
		/* Tester of some navigation strategy to avoid the zipline */
		
		
		
		
		Robot.getNav().travelTo((x_0-0.2) * GRID_SIZE, (y_0-0.2) * GRID_SIZE, false);
		Robot.getLightLocalizer().localize(false);
		Robot.getNav().travelTo(Robot.getLightLocalizer().getGridX(), Robot.getLightLocalizer().getGridY(), false);
		//Robot.getNav().turnTo(Math.PI/2);
		
		//go mount the zipline
		
//		Robot.getLineMotor().setSpeed(250);
//		Robot.getLineMotor().forward();
		
//		LocalEV3.get().getTextLCD().drawString(String.valueOf(Robot.getOdo().getX()), 0, 5);
//		LocalEV3.get().getTextLCD().drawString(String.valueOf(Robot.getOdo().getY()), 0, 6);
		
//		Button.waitForAnyPress();
//		Robot.getNav().setWaypoints(2 * GRID_SIZE, 3 * GRID_SIZE);
		/*navStrat.navigateToObjectiveZone();						//navigate to objective
		flagDetection.searchAndDetect();						//detect flag
		navStrat.navigateBack();								//navigate back to start
		Sound.playNote(Sound.FLUTE, Sound.DOUBLE_BEEP, 500);*/	//celebrate
		
		/* Test for localization after dismounting from the zipline */
		//first set coordinates that odometer thinks we are @ (x_0, y_0).
//		Robot.getOdo().setX(1 * Robot.GRID_SIZE);
//		Robot.getOdo().setY(6 * Robot.GRID_SIZE);
//		Robot.getOdo().setTheta(Math.PI/2);
		Robot.getZipLineTraversal().traverse();
		
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