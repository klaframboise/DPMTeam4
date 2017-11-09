package ca.mcgill.ecse211.team4.robot;

//import java.io.IOException;
//import java.util.Map;

//import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.team4.drivers.Navigation;
//import ca.mcgill.ecse211.team4.lab5.ZipLineTraversal;
import ca.mcgill.ecse211.team4.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.odometry.OdometryCorrection;
import ca.mcgill.ecse211.team4.odometry.OdometryDisplay;
//import ca.mcgill.ecse211.team4.sensing.FlagDetection;
//import ca.mcgill.ecse211.team4.strategy.NavigationStrategy;
import lejos.hardware.Button;
//import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

/**
 * Main class.
 * @author Kevin Laframboise
 *
 */
public class Robot {

	/**
	 * Team number.
	 */
	//public static final int TEAM_NUMBER = 4;
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
	 * Offset to compensate for the distance loss of right wheel
	 */
	public static final float ANGLE_OFFSET = SPEED_OFFSET;
	
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
	//private static Map<String, Integer> gameParameters;
	
	/**
	 * Can either be "Green" or "Red". This will affect strategies.
	 */
	//private static String teamColor;
	
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
	private static EV3MediumRegulatedMotor servo;

	/**
	 * Initialize components and sets up game. 
	 * @param args
	 */
	@SuppressWarnings("resource")	//resources are used for the full life cycle of the program, no need to close
	public static void main(String[] args) {
		
		/* Initialize components */
		leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		lineMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
		servo = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
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
		LightLocalizer lightLocalizer = new LightLocalizer(lightSampler, lightData, leftMotor, rightMotor);
		/* Initialize flag detection color sensor */
//		float[] colorData = new float[10];
//		SampleProvider colorSampler = new EV3ColorSensor(LocalEV3.get().getPort("S3")).getMode("ColorID");
		
		/* Initialize Display */
		TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay display = new OdometryDisplay(odo, t);
		int buttonChoice;
		
		/* Get game parameters */
//		try {
//			gameParameters = GameSetup.getGameParameters(true);
//		} catch (IOException | ParseException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//			gameParameters = null;
//		}
//		
		/* Get team color */
//		if(gameParameters.get("RedTeam").intValue() == TEAM_NUMBER) {
//			teamColor = "Red";
//		}
//		else {
//			teamColor = "Green";
//		}
		int[] coords = { 0, 0 };
		int x_0 = 0;
		int y_0 = 0;
		int x_c = 0;
		int y_c = 0;
		int sc = 0;
		t.drawString("This is a tester", 0, 0);
		
		buttonChoice = Button.waitForAnyPress();
		promptCoordInput(t, "x0", "y0", coords);
		x_0 = coords[0];
		y_0 = coords[1];
		promptCoordInput(t, "xc", "yc", coords);
		x_c = coords[0];
		y_c = coords[1];

		// prompt for sc
		while (true) {
			// clear display
			t.clear();
			
			// prompt user for input
			t.drawString("SC: Up & Down", 0, 0);
			t.drawString(String.valueOf(sc), 0, 2);

			// wait for button press
			buttonChoice = Button.waitForAnyPress();

			// act on user input
			switch (buttonChoice) {
			case Button.ID_UP:
				sc++;
				break;
			case Button.ID_DOWN:
				sc--;
				break;
			default:
				break;
			}

			// wrap around [0,3]
			sc = (sc < 0) ? 3 : sc % 4;

			// break out of loop if enter was pressed
			if (buttonChoice == Button.ID_ENTER)
				break;

		}
		
		odo.start();
		display.start();
		usLocalizer.localize();
		lightLocalizer.localize(sc);
		/* Initialize navigation strategy */
//		NavigationStrategy navStrat = new NavigationStrategy(teamColor, gameParameters, nav);
		
		/* Initialize flag detection */
//		FlagDetection flagDetection = new FlagDetection(colorSampler, colorData, gameParameters.get("O" + teamColor.charAt(0)).intValue(), gameParameters);
				
		/* Start game */ 
		//odo.start();
		//nav.start();
		/* Localize */
		//usLocalizer.localize();
//		lightLocalizer.localize(gameParameters.get(teamColor + "Corner").intValue());
		
//		navStrat.navigateToObjectiveZone();						//navigate to objective
//		flagDetection.searchAndDetect();						//detect flag
//		navStrat.navigateBack();								//navigate back to start
//		Sound.playNote(Sound.FLUTE, Sound.DOUBLE_BEEP, 500);	//celebrate
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
//	public static Map getGameParameters() {
//		return gameParameters; 
//	}

	/**
	 * 
	 * @return teamColor
	 */
//	public static String getTeamColor() {
//		return teamColor;
//	}

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
	public static EV3MediumRegulatedMotor getServo() {
		return servo;
	}

	private static void promptCoordInput(TextLCD t, String xLabel, String yLabel, int[] coords) {
		int buttonChoice;
		while (true) {
			// clear display
			t.clear();

			// prompt user for input
			t.drawString(xLabel + ": Left & Right", 0, 0);
			t.drawString(yLabel + ": Up & Down", 0, 1);
			t.drawString("(" + coords[0] + ", " + coords[1] + ")", 0, 3);

			// wait for button press
			buttonChoice = Button.waitForAnyPress();

			// act on user input
			switch (buttonChoice) {
			case Button.ID_RIGHT:
				coords[0]++;
				break;
			case Button.ID_LEFT:
				coords[0]--;
				break;
			case Button.ID_UP:
				coords[1]++;
				break;
			case Button.ID_DOWN:
				coords[1]--;
				break;
			default:
				break;
			}

			// wrap around [0,8]
			coords[0] = (coords[0] < 0) ? 8 : coords[0] % 9;
			coords[1] = (coords[1] < 0) ? 8 : coords[1] % 9;

			// break out of loop if enter was pressed
			if (buttonChoice == Button.ID_ENTER)
				break;
		}
	}
}
