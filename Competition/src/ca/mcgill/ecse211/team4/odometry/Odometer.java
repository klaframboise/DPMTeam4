package ca.mcgill.ecse211.team4.odometry;

import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Tracks the movement of the robot using odometry principles.
 * 
 * @author Walid Chabchoub & Kevin Laframboise
 *
 */
public class Odometer extends Thread {

  /**
   * Current x coordinate, in cm.
   */
  private double x;

  /**
   * Current y coordinate, in cm.
   */
  private double y;

  /**
   * Current heading, in radians.
   */
  private double theta;

  /**
   * Current left motor tachometer count, in degrees.
   */
  private int leftMotorTachoCount;

  /**
   * Current right motor tachometer count, in degrees.
   */
  private int rightMotorTachoCount;

  /**
   * Left motor tracked by odometer.
   */
  private EV3LargeRegulatedMotor leftMotor;

  /**
   * Right motor tracked by odometer.
   */
  private EV3LargeRegulatedMotor rightMotor;

  /**
   * Lock object for mutual exclusion.
   */
  private Object lock;

  /**
   * Sleep period between odometer computations, in ms.
   */
  private static final long ODOMETER_PERIOD = 25;

  /**
   * Creates an odometer object that, once the thread is started, will track the right and left
   * motor passed as argument.
   * 
   * @param leftMotor
   * @param rightMotor
   */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.x = 0.0;
    this.y = 0.0;
    this.theta = 0.0;
    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;
    lock = new Object();

    // reset motor tachos
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
  }

  /**
   * Starts the odometer. Should always be used through {@link Odometer#start()} so that it is
   * running as a spearated thread.
   */
  public void run() {

    /* Initialize local variables */
    long updateStart;
    long updateEnd;
    int leftLastTachoCount;
    int rightLastTachoCount;
    double dLeftWheel;
    double dRightWheel;
    double deltaD;
    double deltaT;
    double dX;
    double dY;

    /* Start odometry */
    while (true) {

      updateStart = System.currentTimeMillis(); // keeping track of start time to ensure constant
                                                // time updates

      /* Keep last iteration's tacho counts */
      rightLastTachoCount = rightMotorTachoCount;
      leftLastTachoCount = leftMotorTachoCount;

      /* Get new tacho counts */
      rightMotorTachoCount = rightMotor.getTachoCount();
      leftMotorTachoCount = leftMotor.getTachoCount();

      /* Find each wheel's displacement */
      dLeftWheel = Math.PI * Robot.WHEEL_RADIUS * (leftMotorTachoCount - leftLastTachoCount) / 180;
      dRightWheel =
          Math.PI * Robot.WHEEL_RADIUS * (rightMotorTachoCount - rightLastTachoCount) / 180;

      deltaD = (dLeftWheel + dRightWheel) / 2; // magnitude of displacement
      deltaT = (dLeftWheel - dRightWheel) / Robot.TRACK; // change in heading

      /* Update theta */
      synchronized (lock) {
        theta = (theta + deltaT) % (2 * Math.PI); // update heading
        theta = (theta < 0) ? theta + 2 * Math.PI : theta; // convert negative angle to positive
                                                           // equivalent
      }

      dX = deltaD * Math.sin(theta); // displacement on x-axis
      dY = deltaD * Math.cos(theta); // displacement on y-axis

      /* Update coords */
      synchronized (lock) {
        x += dX;
        y += dY;
      }

      /* This ensures that the odometer only runs once every period */
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here because it is not
          // expected that the odometer will be interrupted by
          // another thread
        }
      }
    }
  }

  /**
   * Places the current x, y and theta in the position array. Provided in lab 2 and used as-is.
   * 
   * @param position array to fill with x, y and theta. Element at index 0 is x, 1 is y and 2 is
   *        theta.
   * @param update dictates which coordinate to place in the array. Follows same indexing as
   *        position.
   */
  public void getPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        position[0] = x;
      if (update[1])
        position[1] = y;
      if (update[2])
        position[2] = theta;
    }
  }

  /**
   * @return {@link Odometer#x}
   */
  public double getX() {
    double result;

    synchronized (lock) {
      result = x;
    }

    return result;
  }

  /**
   * @return {@link Odometer#y}
   */
  public double getY() {
    double result;

    synchronized (lock) {
      result = y;
    }

    return result;
  }

  /**
   * 
   * @return {@link Odometer#theta}
   */
  public double getTheta() {
    double result;

    synchronized (lock) {
      result = theta;
    }

    return result;
  }

  /**
   * Changes the current x, y and theta in to the value in the position array. Provided in lab 2 and
   * used as-is.
   * 
   * @param position array containing new x, y and theta. Element at index 0 is x, 1 is y and 2 is
   *        theta.
   * @param update dictates which coordinate to update. Follows same indexing as position.
   */
  public void setPosition(double[] position, boolean[] update) {
    // ensure that the values don't change while the odometer is running
    synchronized (lock) {
      if (update[0])
        x = position[0];
      if (update[1])
        y = position[1];
      if (update[2])
        theta = position[2];
    }
  }

  /**
   * Changes the current x position.
   * 
   * @param x new x, in cm.
   */
  public void setX(double x) {
    synchronized (lock) {
      this.x = x;
    }
  }

  /**
   * Changes the current y position.
   * 
   * @param y new y, in cm.
   */
  public void setY(double y) {
    synchronized (lock) {
      this.y = y;
    }
  }

  /**
   * Changes the current heading.
   * 
   * @param theta new theta, in radians.
   */
  public void setTheta(double theta) {
    synchronized (lock) {
      this.theta = theta;
    }
  }

  /**
   * @return {@link Odometer#leftMotorTachoCount}
   */
  public int getLeftMotorTachoCount() {
    return leftMotorTachoCount;
  }

  /**
   * @return {@link Odometer#rightMotorTachoCount}
   */
  public int getRightMotorTachoCount() {
    return rightMotorTachoCount;
  }

}
