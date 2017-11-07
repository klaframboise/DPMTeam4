package ca.mcgill.ecse211.team4.robot;

import java.io.IOException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.odometry.OdometryCorrection;
import ca.mcgill.ecse211.team4.sensing.FlagDetection;
import ca.mcgill.ecse211.team4.strategy.NavigationStrategy;
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
public class Robot {

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
	public static final float SPEED_OFFSET = 0.9875f;
	
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
	public static final double TRACK = 13.5;
	
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
	public void main(String[] args) {
		
		/* Initialize components */
		leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		lineMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		servo = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
		odo = new Odometer(leftMotor, rightMotor);
		correction = new OdometryCorrection(odo);
		nav = new Navigation(odo, leftMotor, rightMotor);
		
		/* Initialize ultrasonic sensor */
		float[] usData = new float[10];
		SampleProvider us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1")).getMode("Distance");
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(us, usData, leftMotor, rightMotor);
		
		/* Initialize localization color sensor */
		float[] lightData = new float[10];
		SampleProvider lightSampler = new EV3ColorSensor(LocalEV3.get().getPort("S4")).getMode("Red");
		lightLocalizer = new LightLocalizer(lightSampler, lightData, leftMotor, rightMotor);
		
		/* Initialize flag detection color sensor */
		float[] colorData = new float[10];
		SampleProvider colorSampler = new EV3ColorSensor(LocalEV3.get().getPort("S3")).getMode("ColorID");
		
		/* Initialize display */
		Display display = new Display();
		display.start();
		
		/* Get game parameters */
		try {
			gameParameters = GameSetup.getGameParameters(true);
		} catch (IOException | ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			gameParameters = null;
		}
		
		/* Get team color */
		if(gameParameters.get("RedTeam").intValue() == TEAM_NUMBER) {
			teamColor = "Red";
		}
		else {
			teamColor = "Green";
		}
		
		/* Initialize navigation strategy */
		NavigationStrategy navStrat = new NavigationStrategy(teamColor, gameParameters, nav);
		
		/* Initialize flag detection */
		FlagDetection flagDetection = new FlagDetection(colorSampler, colorData, gameParameters.get("O" + teamColor.charAt(0)).intValue(), gameParameters);
				
		/* Start game */
		odo.start();
		nav.start();
		/* Localize */
		usLocalizer.localize();
		lightLocalizer.localize(gameParameters.get(teamColor + "Corner").intValue());
		
		navStrat.navigateToObjectiveZone();						//navigate to objective
		flagDetection.searchAndDetect();						//detect flag
		navStrat.navigateBack();								//navigate back to start
		Sound.playNote(Sound.FLUTE, Sound.DOUBLE_BEEP, 500);	//celebrate
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
