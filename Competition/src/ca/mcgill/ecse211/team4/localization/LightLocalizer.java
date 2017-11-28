package ca.mcgill.ecse211.team4.localization;

import java.util.Arrays;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class uses the color sensor to correct to odometer's x, y and theta by evaluating the
 * distance of the robot to a cross-section in the grid.
 * 
 * @author Kevin Laframboise & Wenjie Wei
 *
 */
public class LightLocalizer {

  /**
   * Distance between the light sensor and the center of rotation of the robot, in cm.
   */
  public static final double LS_TO_CENTER = 16.5;

  /**
   * Sample provider for the color sensor. Sample Provider must be in red mode.
   */
  private static SampleProvider colorSampler;

  /**
   * Buffer array for the color sensor data.
   */
  private static float[] lightData;

  /**
   * Left motor driving the robot, used in the sweep operation.
   */
  private static EV3LargeRegulatedMotor leftMotor;

  /**
   * Right motor driving the robot, used in the sweep operation.
   */
  private static EV3LargeRegulatedMotor rightMotor;

  /**
   * Counts the number of lines detected during sweep operation.
   */
  private int counter;

  /**
   * Records the angle at which a line was detected.
   */
  private double[] angles;

  /**
   * Distance in x between the robot and the y-axis, in cm.
   */
  private double dx;

  /**
   * Distance in y between the robot and the x-axis, in cm.
   */
  private double dy;

  /**
   * X coordinate of the intersection around which localization is occurring.
   */
  private double gridX;

  /**
   * Y coordinate of the intersection around which localization is occurring.
   */
  private double gridY;

  /**
   * Angle delta between the odometer's zero heading and true zero heading, in radians.
   */
  private double dTheta;

  /**
   * Debugging flag.
   */
  private boolean debug;


  /**
   * Creates a LightLocalizer object with given properties.
   * 
   * @param colorSampler
   * @param lightData
   * @param leftMotor
   * @param rightMotor
   * @param debug, setting this to true will cause the algorithm to print more verbose information
   *        to the console and will require the user to press a button after each step of the
   *        localization.
   */
  public LightLocalizer(SampleProvider colorSampler, float[] lightData,
      EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, boolean debug) {
    LightLocalizer.colorSampler = colorSampler;
    LightLocalizer.lightData = lightData;
    LightLocalizer.leftMotor = Robot.getDrivingMotors()[0];
    LightLocalizer.rightMotor = Robot.getDrivingMotors()[1];
    this.debug = debug;
    counter = 0;
    angles = new double[4];
  }

  /**
   * Creates a LightLocalizer object with given properties. Sets debug to false.
   * 
   * @see LightLocalizer#LightLocalizer(SampleProvider, float[], EV3LargeRegulatedMotor,
   *      EV3LargeRegulatedMotor, boolean)
   * 
   * @param colorSampler
   * @param lightData
   * @param leftMotor
   * @param rightMotor
   */
  public LightLocalizer(SampleProvider colorSampler, float[] lightData,
      EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    new LightLocalizer(colorSampler, lightData, leftMotor, rightMotor, false);
  }

  /**
   * Performs a 360 degrees sweep in order to detect grid lines.
   */
  private void sweep() {

    /* Initialize angle data buffer and counter */
    counter = 0;
    angles = new double[4];

    /* Set motor speed */
    leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET);
    rightMotor.setSpeed(Robot.ROTATE_SPEED);

    /* Start rotating 360 deg */
    int rotationAngle = Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, 360);
    leftMotor.rotate(rotationAngle, true);
    rightMotor.rotate(-rotationAngle, true);

    /* Sample light sensor every 25 ms to detect lines */
    do {
      if (getRedIntensity() < Robot.LINE_RED_INTENSITY) {
        angles[counter++] = Robot.getOdo().getTheta();
        Sound.beep();
        try {
          Thread.sleep(25);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
      /* Keep sampling while robot is turning or less than 4 lines have been detected */
    } while (leftMotor.isMoving() && rightMotor.isMoving() && counter < angles.length);

    /* Stop motors when done sweeping */
    leftMotor.stop(true);
    rightMotor.stop(true);

  }

  /**
   * Localizes with consideration for the starting corner. It localizes at every corner as if it
   * were corner 0 and then flips signs accordingly when updating the odometer. Using this method
   * will cause the robot to move until the color sensor crosses the next grid line intersection in
   * the positive x and y axis. Be careful when using this near forbidden areas.
   * 
   * @param startingCorner [0,3].
   * @return true if localization is successful.
   */
  public boolean localize(int startingCorner) {

    boolean isSuccessful = localize(true); // perform localization to get dx, dy and dtheta

    /* Return false if localization was unsuccesful */
    if (!isSuccessful) {
      return isSuccessful;
    }

    /* Treat starting corner accordingly */
    switch (startingCorner) {
      case 0:

        /* Print debug info */
        if (debug) {
          System.out.println("Setting x to: " + (Robot.GRID_SIZE + dx));
          System.out.println("Setting y to: " + (Robot.GRID_SIZE + dy));
          System.out.println("Setting theta to: " + (Robot.getOdo().getTheta() + dTheta - Math.PI));
          Button.waitForAnyPress();
        }

        /* Update odometer */
        Robot.getOdo().setX(Robot.GRID_SIZE + dx);
        Robot.getOdo().setY(Robot.GRID_SIZE + dy);
        Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);

        /* Travel travel to (1,1) */
        Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
        Robot.getNav().turnTo(0);

        break; // exit switch case

      case 1:

        /* Update odometer */
        Robot.getOdo().setX(Robot.GRID_SIZE + dx);
        Robot.getOdo().setY(Robot.GRID_SIZE + dy);
        Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);

        /* Travel to what the odometer thinks is (1,1) */
        Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
        Robot.getNav().turnTo(0);

        /* Set odo to (MAX_COORD, 1), which is where the robot is at this point */
        Robot.getOdo().setX(Robot.MAX_COORD * Robot.GRID_SIZE);
        Robot.getOdo().setY(1 * Robot.GRID_SIZE);
        Robot.getOdo().setTheta(3 * Math.PI / 2);

        /* Print debug info */
        if (debug) {
          System.out.println("x: " + Robot.getOdo().getX());
          System.out.println("y: " + Robot.getOdo().getY());
          System.out.println("theta: " + Math.toDegrees(Robot.getOdo().getTheta()));
          Button.waitForAnyPress();
        }

        break; // exit switch case

      case 2:

        /* Update odometer */
        Robot.getOdo().setX(Robot.GRID_SIZE + dx);
        Robot.getOdo().setY(Robot.GRID_SIZE + dy);
        Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);

        /* Travel to what the odometer thinks is (1,1) */
        Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
        Robot.getNav().turnTo(0);

        /* Set odo to (MAX_COORD, MAX_COORD), which is where the robot is at this point */
        Robot.getOdo().setX(Robot.MAX_COORD * Robot.GRID_SIZE);
        Robot.getOdo().setY(Robot.MAX_COORD * Robot.GRID_SIZE);
        Robot.getOdo().setTheta(Math.PI);

        break; // exit switch case

      case 3:

        /* Update odometer */
        Robot.getOdo().setX(Robot.GRID_SIZE + dx);
        Robot.getOdo().setY(Robot.GRID_SIZE + dy);
        Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta - Math.PI);

        /* Travel to what the odometer thinks is (1,1) */
        Robot.getNav().travelTo(Robot.GRID_SIZE, Robot.GRID_SIZE, false);
        Robot.getNav().turnTo(0);

        /* Set odo to (1, MAX_COORD), which is where the robot is at this point */
        Robot.getOdo().setX(1 * Robot.GRID_SIZE);
        Robot.getOdo().setY(Robot.MAX_COORD * Robot.GRID_SIZE);
        Robot.getOdo().setTheta(Math.PI / 2);

        break; // exit switch case
    }

    return isSuccessful; // return the result of the localization, which has to be true at this
                         // point

  }

  /**
   * Performs light localization by assuming that this is not the initial localization. Using this
   * method will cause the robot to move until the color sensor crosses the next grid line
   * intersection in the positive x and y axis. Be careful when using this near forbidden areas.
   * 
   * @return true if localization is successful.
   */
  public boolean localize() {
    return localize(false);
  }

  /**
   * Performs light localization according to whether this is the initial localization or not. Using
   * this method will cause the robot to move until the color sensor crosses the next grid line
   * intersection in the positive x and y axis. Be careful when using this near forbidden areas.
   * 
   * @param initialLocalization true if this is the first light localization performed.
   * @return true if localization was successful.
   */
  public boolean localize(boolean initialLocalization) {

    /* Make sure robot is in 3rd quadrant of a grid intersection. */
    if (!initialLocalization) {
      adjust('x');
      adjust('y');
    } else if (initialLocalization) {
      adjust('o'); // during initial localization, we can safely assume that the robot is roughly at
                   // the center of the square. This allows us to move to the intersection by
                   // following a 45 deg angle.
    }

    /* Safety precaution to ensure color sensor is away from any lines when starting sampling */
    if (!initialLocalization) {
      Robot.getNav().turnTo(Math.PI / 4);
    }

    sweep(); // acquire angle data

    /* Compute real position */
    if (counter == 4) {

      /* Compute dx, dy and dtheta */
      dx = -LS_TO_CENTER * Math.cos(Math.abs(angles[3] - angles[1]) / 2);
      dy = -LS_TO_CENTER * Math.cos(Math.abs(angles[0] - angles[2]) / 2);
      dTheta = -Math.PI / 2.0 - (angles[3] - Math.PI) + Math.abs(angles[3] - angles[1]) / 2.0;

      /* Print debug info */
      if (debug) {
        System.out.println("x: " + Robot.getOdo().getX());
        System.out.println("y: " + Robot.getOdo().getY());
        System.out.println("Theta: " + Robot.getOdo().getTheta());
        System.out.println("dx: " + dx);
        System.out.println("dy: " + dy);
        System.out.println("dTheta: " + Math.toDegrees(dTheta));
        Button.waitForAnyPress();
      }

    } else {
      return false; // at least one axis was not detected, cannot perform
                    // localization
    }

    /*
     * Only adjust odometer if this is not the first localization because starting corner has to be
     * taken into consideration while localizing for the first time
     */
    if (!initialLocalization) {

      /* Figure out current x,y position w.r.t. the grid */
      gridX = Math.round(Robot.getOdo().getX() / Robot.GRID_SIZE) * Robot.GRID_SIZE;
      gridY = Math.round(Robot.getOdo().getY() / Robot.GRID_SIZE) * Robot.GRID_SIZE;

      /* Print debug info */
      if (debug) {
        System.out.println("gridX: " + gridX);
        System.out.println("gridY: " + gridY);
        System.out.println("dx: " + dx);
        System.out.println("dy: " + dy);
        System.out.println("dtheta in deg: " + Math.toDegrees(dTheta));
        System.out.println(
            "Odo theta reading before reset: " + Math.toDegrees(Robot.getOdo().getTheta()));
        System.out.println("Setting X to : " + (gridX + dx));
        System.out.println("Setting Y to : " + (gridY + dy));
        System.out.println("Setting Theta to : " + (Robot.getOdo().getTheta() + dTheta + Math.PI));
        Button.waitForAnyPress();
      }

      /* Correct odometer */
      Robot.getOdo().setTheta(Robot.getOdo().getTheta() + dTheta + Math.PI);
      Robot.getOdo().setX(gridX + dx);
      Robot.getOdo().setY(gridY + dy);
    }

    return true; // localization was successful

  }

  /**
   * Drives the robot towards the given axis until a grid line is detected then backwards 21 cm.
   * 
   * @param axis x, y, o. 'o' stands for origin, meaning that the robot will be driven in a 45 deg
   *        angle towards the intersection. 'o' should only be used when the robot is roughly at the
   *        center of a square.
   */
  private void adjust(char axis) {

    switch (axis) {

      case 'y':

        Robot.getNav().turnTo(Math.PI / 2.0); // turn towards y axis
        break; // exit switch case

      case 'x':

        Robot.getNav().turnTo(0); // turn towards x axis
        break; // exit switch case

      case 'o':

        Robot.getNav().turnTo(Math.PI / 4.0); // turn towards upper right intersection
        break; // exit switch case

      default:
        return; // return in case of invalid input
    }

    /* Go forward until axis is found */
    while (getRedIntensity() > Robot.LINE_RED_INTENSITY) {

      /* Start motors forward */
      leftMotor.setSpeed(Robot.FORWARD_SPEED);
      rightMotor.setSpeed(Robot.FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.forward();

      /* Sleep 25 ms before next sampling */
      try {
        Thread.sleep(25);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

    /* Go back 21 cm so that color sensor is in 3rd quadrant */
    leftMotor.rotate(-Helper.convertDistance(Robot.WHEEL_RADIUS, 21), true);
    rightMotor.rotate(-Helper.convertDistance(Robot.WHEEL_RADIUS, 21), false);

  }

  /**
   * Returns the median of a group of sample from the color sensor as the detected color.
   * 
   * @return color intensity detected
   */
  public static float getRedIntensity() {

    /* Get data */
    for (int i = 0; i < lightData.length; i++) {
      colorSampler.fetchSample(lightData, i);
    }

    /* Sort and return median */
    Arrays.sort(lightData);
    return (lightData[(lightData.length / 2) - 1] + lightData[lightData.length / 2]) / 2.0f;
  }

  /**
   * @return {@link LightLocalizer#gridX}
   */
  public double getGridX() {
    return gridX;
  }

  /**
   * @return {@link LightLocalizer#gridY}
   */
  public double getGridY() {
    return gridY;
  }
}
