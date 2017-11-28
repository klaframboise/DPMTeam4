package ca.mcgill.ecse211.team4.robot;

import ca.mcgill.ecse211.team4.odometry.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * This class displays information on the EV3's display. Derived from the lab 2 class
 * "OdometryDisplay.java".
 * 
 * @author Kevin Laframboise
 *
 */
public class Display extends Thread {

  /**
   * Update frequency of the display, in ms.
   */
  private static final long DISPLAY_PERIOD = 250;

  /**
   * Reference to the EV3's display
   */
  private TextLCD t;

  /**
   * Reference to the Odometer's from which display data comes.
   */
  private Odometer odometer;

  /**
   * Creates a display object that will print on the EV3's display the current data of the given
   * Odometer.
   * 
   * @param odometer
   */
  public Display(Odometer odometer) {
    this.odometer = odometer;
    t = LocalEV3.get().getTextLCD();
  }

  /**
   * Starts the display. Should be used via {@link Display#start()}.
   */
  public void run() {

    /* Initialize local variables */
    long displayStart;
    long displayEnd;
    double[] position = new double[3];

    while (true) {

      displayStart = System.currentTimeMillis();

      /* Clear the lines for displaying odometry information */
      t.clear();
      t.drawString("X:              ", 0, 0);
      t.drawString("Y:              ", 0, 1);
      t.drawString("T:              ", 0, 2);


      odometer.getPosition(position, new boolean[] {true, true, true}); // get the odometry
                                                                        // information

      position[2] = (position[2] * 180) / Math.PI; // convert odo's theta to degrees

      /* Display odometry information */
      for (int i = 0; i < 3; i++) {
        t.drawString(formattedDoubleToString(position[i], 2), 3, i);
      }

      /* Throttle the OdometryDisplay */
      displayEnd = System.currentTimeMillis();
      if (displayEnd - displayStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that Display will be interrupted
          // by another thread
        }
      }
    }
  }

  /**
   * Formats a double to a usable String for display purposes. Provided in Lab 2 and used as-is.
   * 
   * @param x the double to be formatted.
   * @param places number of decimal places.
   * @return
   */
  private static String formattedDoubleToString(double x, int places) {
    String result = "";
    String stack = "";
    long t;

    // put in a minus sign as needed
    if (x < 0.0)
      result += "-";

    // put in a leading 0
    if (-1.0 < x && x < 1.0)
      result += "0";
    else {
      t = (long) x;
      if (t < 0)
        t = -t;

      while (t > 0) {
        stack = Long.toString(t % 10) + stack;
        t /= 10;
      }

      result += stack;
    }

    // put the decimal, if needed
    if (places > 0) {
      result += ".";

      // put the appropriate number of decimals
      for (int i = 0; i < places; i++) {
        x = Math.abs(x);
        x = x - Math.floor(x);
        x *= 10.0;
        result += Long.toString((long) x);
      }
    }

    return result;
  }
}
