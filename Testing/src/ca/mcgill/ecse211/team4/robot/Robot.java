package ca.mcgill.ecse211.team4.robot;

/**
 * This is the main class of the tester. 
 * The main method will be implemented here while other helper 
 * methods will be in individual class files in the package
 * @author Wenjie Wei
 * 
 */

import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.odometry.OdometryCorrection;
import ca.mcgill.ecse211.team4.odometry.OdometryDisplay;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.team4.drivers.Navigation;
import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Robot {
	/* constants */
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 13.5;
	public static final double GRID_SIZE = 30.48;
	public static final float LINE_RED_INTENSITY = 0.30f;
	public static final int FORWARD_SPEED = 175;
	public static final int ROTATE_SPEED = 150;
	public static final int SAMPLE_SIZE = 10;
	public static final float SPEED_OFFSET = 0.9860f;
	public static final float ANGLE_OFFSET = SPEED_OFFSET;

	/* Static system wide variables */
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port rearLightPort = LocalEV3.get().getPort("S3");
	private static final Port frontLightPort = LocalEV3.get().getPort("S2");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor lineMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	private static Odometer odo = new Odometer(leftMotor, rightMotor);
	private static OdometryCorrection correction = new OdometryCorrection(odo);
	private static Navigation nav = new Navigation(odo, leftMotor, rightMotor);

	/**
	 * Main testing method. Task 1: let the robot go long distance and check out
	 * the angle error, if it is going in a straight line?
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		/* Local variables */
//		@SuppressWarnings("resource")
//		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
//		SampleProvider usSampler = usSensor.getMode("Distance");
//		float[] usData = new float[usSampler.sampleSize() * SAMPLE_SIZE]; // usData

		TextLCD t = LocalEV3.get().getTextLCD();
		OdometryDisplay display = new OdometryDisplay(odo, t);
		int buttonChoice;

		leftMotor.setAcceleration(250);
		rightMotor.setAcceleration(250);

		while (true) {
			t.clear();
			t.drawString("Click ENTER to start", 0, 0);

			buttonChoice = Button.waitForAnyPress();
			if (buttonChoice == Button.ID_ENTER)
				break;
		}
		display.start();
		nav.setWaypoints(0*GRID_SIZE, 9*GRID_SIZE);
		odo.start();
		nav.start();
		lineMotor.setSpeed(-180);
		lineMotor.rotate(convertDistance(2.1, 300));
		while (nav.isNavigating()) {
			try {
				Thread.sleep(25);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		
		if (Button.waitForAnyPress() == Button.ID_ESCAPE)
			System.exit(0);
	}

	public static Navigation getNav() {
		return nav;
	}

	public static Odometer getOdo() {
		return odo;
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public static OdometryCorrection getCorrection() {
		return correction;
	}
}
