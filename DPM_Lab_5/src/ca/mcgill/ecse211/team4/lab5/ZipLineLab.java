package ca.mcgill.ecse211.team4.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ZipLineLab {

	/* Constants */
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 13.5;
	public static final double GRID_SIZE = 30.48;
	public static final int FORWARD_SPEED = 250;
	public static final int ROTATE_SPEED = 150;
	private static final int SAMPLE_SIZE = 10;
	
	/* Static system wide variables */
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lightPort = LocalEV3.get().getPort("S4");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static Odometer odo = new Odometer(leftMotor, rightMotor);
	private static Navigation nav = new Navigation(odo, leftMotor, rightMotor);

	public static void main(String[] args) {
		/* Local variables */
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usSampler = usSensor.getMode("Distance"); // usDistance provides samples from this instance
		float[] usData = new float[usSampler.sampleSize() * SAMPLE_SIZE]; // usData is the buffer in which data are
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(lightPort);
		SampleProvider colorSampler = colorSensor.getMode("Red");
		float[] lightData = new float[colorSampler.sampleSize() * SAMPLE_SIZE];
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(usSampler, usData, leftMotor, rightMotor);
		LightLocalizer lightLocalizer = new LightLocalizer(colorSampler, lightData, leftMotor, rightMotor);
		int buttonChoice;
		int zipX = 0;
		int zipY = 0;
		TextLCD t = LocalEV3.get().getTextLCD();
		
		while(true) {
			//clear display
			t.clear();
			
			//prompt user for input
			t.drawString("X: Left & Right", 0, 0);
			t.drawString("Y: Up & Down", 0, 1);
			t.drawString("(" + zipX + ", " + zipY + ")",  0, 3);
			
			//wait for button press
			buttonChoice = Button.waitForAnyPress();
			
			//act on user input
			switch(buttonChoice) {
			case Button.ID_RIGHT: zipX++; break;
			case Button.ID_LEFT: zipX--; break;
			case Button.ID_UP: zipY++; break;
			case Button.ID_DOWN: zipY--; break;
			default: break;
			}
			
			//wrap around [0,12]
			zipX = (zipX < 0)? 12 : zipX % 13;
			zipY = (zipY < 0)? 12 : zipY % 13;
			
			//break out of loop if enter was pressed
			if(buttonChoice == Button.ID_ENTER) break;
			
		} 
		
		//travel to zip line start after localization
		usLocalizer.localize();
		lightLocalizer.localize();
		nav.travelTo(zipX * GRID_SIZE, zipY * GRID_SIZE);
		nav.turnTo(3.0 * Math.PI/ 2.0);
		
		//wait for input per design requirements
		Button.waitForAnyPress();
		
		//prepare for zip line traversal
		//TODO implement zip line traversal

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
	 * @return nav
	 */
	public static Navigation getNav() {
		return nav;
	}

	/**
	 * From Lab 2 sample code
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * From Lab 2 sample code
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
