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
	public static final int ROTATE_SPEED = 150;
	public static final float SPEED_OFFSET = 0.9875f;
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

	public static void main(String[] args) {
		/* Local variables */
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is
		// the instance
		SampleProvider usSampler = usSensor.getMode("Distance"); // usSampler
		// provides
		// samples
		// from this
		// instance
		float[] usData = new float[usSampler.sampleSize() * SAMPLE_SIZE]; // usData
		// is
		// the
		// buffer
		// in
		// which
		// data
		// are
		@SuppressWarnings("resource")
		SensorModes colorSensor = new EV3ColorSensor(lightPort);
		SampleProvider colorSampler = colorSensor.getMode("Red");
		float[] lightData = new float[colorSampler.sampleSize() * SAMPLE_SIZE];
		TextLCD t = LocalEV3.get().getTextLCD();
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(usSampler, usData, leftMotor, rightMotor);
		OdometryDisplay display = new OdometryDisplay(odo, t);
		ZipLineTraversal lineTraversal;
		LightLocalizer lightLocalizer;
		int buttonChoice;
		int[] coords = { 0, 0 };
		int x_0 = 0;
		int y_0 = 0;
		int x_c = 0;
		int y_c = 0;
		int sc = 0;

		leftMotor.setAcceleration((int) (250 * SPEED_OFFSET));
		rightMotor.setAcceleration(250);

		// get zipline location from user
		promptCoordInput(t, "x0", "y0", coords);
		x_0 = coords[0];
		y_0 = coords[1];
		promptCoordInput(t, "xc", "yc", coords);
		x_c = coords[0];
		y_c = coords[1];

		lineTraversal = new ZipLineTraversal(lineMotor, leftMotor, rightMotor, x_c * GRID_SIZE, y_c * GRID_SIZE,
				x_0 * GRID_SIZE, y_0 * GRID_SIZE);

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
		
		//initialize light localizer with 
		lightLocalizer = new LightLocalizer(colorSampler, lightData, leftMotor, rightMotor, x_c, y_c);
		// start odometry and display
		odo.start();
		display.start();
		// localize
		usLocalizer.localize();
		lightLocalizer.localize(sc);
		// TODO check that localization corrects according to sc

		// debugging purposes only
		if (Button.waitForAnyPress() == Button.ID_ESCAPE)
			System.exit(0);
		// navigate to waypoint

		/**
		 * Now figure out the logic to navigate from each SC while avoiding the
		 * zipline
		 */
		//set nav priority to max
		nav.setPriority(Thread.MAX_PRIORITY);
		if (sc == 2) {
			nav.setWaypoints(x_0 * GRID_SIZE, odo.getY() + 5);
			nav.start();
			while (nav.isNavigating()) {
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			nav.setWaypoints(x_0 * GRID_SIZE, y_0 * GRID_SIZE);

		} else {
			nav.setWaypoints(x_0 * GRID_SIZE, y_0 * GRID_SIZE);
			nav.start();
		}
		// TODO: Current Navigation Error is too big. this error will grow if it
		// is to travel a longer distance
		// Try fix navigation first
		// wait for nav to end
		while (nav.isNavigating()) {
			try {
				Thread.sleep(25);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		if (Button.waitForAnyPress() == Button.ID_ESCAPE)
			System.exit(0);
		// localize
		// turn counterclockwise a little bit to avoid missing the first line (edge
		// case)
		leftMotor.rotate(-150, true);
		rightMotor.rotate(150, false);
		lightLocalizer.lightLocalizeX0Y0(x_0, y_0);

		// turn to face zipline
		LocalEV3.get().getTextLCD().drawString(String.valueOf(x_c), 0, 4);
		LocalEV3.get().getTextLCD().drawString(String.valueOf(x_0), 0, 5);
		LocalEV3.get().getTextLCD().drawString(String.valueOf(y_c), 5, 4);
		LocalEV3.get().getTextLCD().drawString(String.valueOf(y_0), 5, 5);
		if (x_c > x_0)
			nav.turnTo(Math.PI / 2.0);
		else if (x_c < x_0)
			nav.turnTo(3 * Math.PI / 2); 
		else if (y_c > y_0)
			nav.turnTo(0);
		else if (y_c < y_0)
			nav.turnTo(Math.PI);

		if(sc == 3) {
			nav.turnTo(Math.PI/2.0);
		}
		// wait for input per design requirements
		if (Button.waitForAnyPress() == Button.ID_ESCAPE)
			System.exit(0);
		if (sc == 0) {
		leftMotor.rotate(0 , true);
		rightMotor.rotate(0 , false);
		}
		else if(sc == 3) {
			odo.setTheta(odo.getTheta() - Math.PI);
			leftMotor.rotate(120, true);
			rightMotor.rotate(-120, false);
			//nav.turnTo(Math.PI/2.0);
		}
		lineTraversal.traverse();
		if (odo.getX() - x_c > 10 || odo.getY() - y_c > 10) {
			leftMotor.stop(true);
			rightMotor.stop();
		}
	}

	/**
	 * Prompts for an 2D point.
	 * 
	 * @param t
	 *            reference to th TextLCD
	 * @param xLabel
	 *            name of x coordinate
	 * @param yLabel
	 *            name of y coordinate
	 * @param coords
	 *            array in which the coords are placed
	 */
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
	 * 
	 * @return correction
	 */
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
