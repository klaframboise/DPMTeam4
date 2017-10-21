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
	public static final int FORWARD_SPEED = 175;
	public static final int ROTATE_SPEED = 100;
	public static final float SPEED_OFFSET = 0.98f;
	public static final float LINE_RED_INTENSITY = 0.30f;
	private static final int SAMPLE_SIZE = 10;
	
	/* Static system wide variables */
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lightPort = LocalEV3.get().getPort("S4");
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor lineMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static Odometer odo = new Odometer(leftMotor, rightMotor);
	private static OdometryCorrection correction = new OdometryCorrection(odo);
	private static Navigation nav = new Navigation(odo, leftMotor, rightMotor);

	@SuppressWarnings("unused")
	public static void main(String[] args) {
		/* Local variables */
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
		SampleProvider usSampler = usSensor.getMode("Distance"); // usSampler provides samples from this instance
		float[] usData = new float[usSampler.sampleSize() * SAMPLE_SIZE]; // usData is the buffer in which data are
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(lightPort);
		SampleProvider colorSampler = colorSensor.getMode("Red");
		float[] lightData = new float[colorSampler.sampleSize() * SAMPLE_SIZE];
		TextLCD t = LocalEV3.get().getTextLCD();
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(usSampler, usData, leftMotor, rightMotor);
		LightLocalizer lightLocalizer = new LightLocalizer(colorSampler, lightData, leftMotor, rightMotor);
		OdometryDisplay display = new OdometryDisplay(odo, t);
		int buttonChoice;
		int[] coords = {0,0};
		int x_0 = 0;
		int y_0 = 0;
		int x_c = 0;
		int y_c = 0;
		int sc = 0;
		
		leftMotor.setAcceleration(500);
		rightMotor.setAcceleration(500);
		
		promptCoordInput(t, "x0", "y0", coords);
		x_0 = coords[0];
		y_0 = coords[1];
		promptCoordInput(t, "xc", "yc", coords);
		x_c = coords[0];
		y_c = coords[1];
		
		ZipLineTraversal lineTraversal = new ZipLineTraversal(lineMotor, leftMotor, rightMotor, x_c * GRID_SIZE, y_c * GRID_SIZE);
		//prompt for sc
		while(true) {
			//clear display
			t.clear();
			
			//prompt user for input
			t.drawString("SC: Up & Down", 0, 0);
			t.drawString(String.valueOf(sc), 0, 2);
			
			//wait for button press
			buttonChoice = Button.waitForAnyPress();
			
			//act on user input
			switch(buttonChoice) {
			case Button.ID_UP: sc++; break;
			case Button.ID_DOWN: sc--; break;
			default: break;
			}
			
			//wrap around [0,3]
			sc = (sc < 0)? 3 : sc % 4;
			
			//break out of loop if enter was pressed
			if(buttonChoice == Button.ID_ENTER) break;
			
		} 
		
		//start odometry and display
		odo.start();
		//correction.start();
		display.start();
		//localize
		usLocalizer.localize();
		lightLocalizer.localize(sc);
		nav.turnTo(0);
		//TODO check that localization corrects according to sc
		Button.waitForAnyPress();	//debugging purposes only
		//navigate to waypoint
		nav.setWaypoints(x_0 * GRID_SIZE, y_0 * GRID_SIZE);
		nav.start();
		//wait for nav to end
		while(nav.isNavigating()) {
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		Button.waitForAnyPress();	//debugging purposes only
		//localize
		lightLocalizer.localize();
		//turn to face zipline
		nav.turnTo(Math.PI/2.0);
		//wait for input per design requirements
		Button.waitForAnyPress();
		lineTraversal.traverse();

	}
	
	/**
	 * Prompts for an 2D point.
	 * @param t reference to th TextLCD
	 * @param xLabel name of x coordinate
	 * @param yLabel name of y coordinate
	 * @param coords array in which the coords are placed
	 */
	private static void promptCoordInput(TextLCD t, String xLabel, String yLabel, int[] coords) {
		int buttonChoice;
		while(true) {
			//clear display
			t.clear();
			
			//prompt user for input
			t.drawString(xLabel + ": Left & Right", 0, 0);
			t.drawString(yLabel + ": Up & Down", 0, 1);
			t.drawString("(" + coords[0] + ", " + coords[1] + ")",  0, 3);
			
			//wait for button press
			buttonChoice = Button.waitForAnyPress();
			
			//act on user input
			switch(buttonChoice) {
			case Button.ID_RIGHT: coords[0]++; break;
			case Button.ID_LEFT: coords[0]--; break;
			case Button.ID_UP: coords[1]++; break;
			case Button.ID_DOWN: coords[1]--; break;
			default: break;
			}
			
			//wrap around [0,8]
			coords[0] = (coords[0] < 0)? 8 : coords[0] % 9;
			coords[1] = (coords[1] < 0)? 8 : coords[1] % 9;
			
			//break out of loop if enter was pressed
			if(buttonChoice == Button.ID_ENTER) break;
		}
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

	public static OdometryCorrection getCorrection() {
		return correction;
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
