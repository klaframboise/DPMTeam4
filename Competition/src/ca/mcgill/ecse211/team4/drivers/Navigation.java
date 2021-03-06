package ca.mcgill.ecse211.team4.drivers;

import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Drives the robot to given waypoints using the shortest path.
 * 
 * @author Walid Chabchoub & Kevin Laframboise
 *
 */
public class Navigation extends Thread {

  /**
   * Odometer tracking the robot's movement.
   */
  private Odometer odo;

  /**
   * Left motor driven by navigation.
   */
  private EV3LargeRegulatedMotor leftMotor;

  /**
   * Right motor driven by navigation.
   */
  private EV3LargeRegulatedMotor rightMotor;

  /**
   * Indicates whether the robot is currently navigating to a waypoint.
   */
  private boolean isNavigating;

  /**
   * Indicates whether the robot is currently avoiding an obstacle.
   */
  private boolean isAvoiding;

  /**
   * Indicates whether the path should be recalculated.
   */
  private boolean updateNeeded;

  /**
   * Destination waypoint x coordinate, in cm.
   */
  private double waypointX;

  /**
   * Destination waypoint y coordinate, in cm.
   */
  private double waypointY;

  /**
   * Lock object for mutual exclusion.
   */
  private Object lock;

  /**
   * Creates a Navigation object that will drive the given motors to waypoints, according to the
   * position of the robot reported by the odometer.
   * 
   * @param odo odometer tracking the robot's movements.
   * @param leftMotor to be driven by Navigation.
   * @param rightMotor to be driven by Navigation.
   */
  public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) {
    this.odo = odo;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    isNavigating = false;
    waypointX = 0;
    waypointY = 0;
    lock = new Object();
  }

  /**
   * Starts the navigation. Use {@link Navigation#start()} to use as a thread.
   */
  public void run() {

    double distance;

    while (true) {

      /* Check if path needs to be recalculated */
      if (!isAvoiding && updateNeeded) {
        travelTo(waypointX, waypointY, true);
        synchronized (lock) {
          isNavigating = true;
          updateNeeded = false;
        }
      }

      /* Check if navigation is done, using euclidean distance */
      distance =
          Math.sqrt(Math.pow(waypointX - odo.getX(), 2) + Math.pow(waypointY - odo.getY(), 2));
      if (distance < 1.0) {
        synchronized (lock) {
          isNavigating = false;
        }
      }

      /* Wait 100ms before recalculating path */
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }

  /**
   * Travels to the given waypoint. Sets the wheel speed offset to {@link Robot#SPEED_OFFSET}.
   * 
   * @param x x-coordinate in cm.
   * @param y y-coordinate in cm.
   * @param immediateReturn dictates whether the method should return immediately or wait for the
   *        motor movements to finish.
   */
  public void travelTo(double x, double y, boolean immediateReturn) {
    travelTo(x, y, Robot.SPEED_OFFSET, immediateReturn);
  }

  /**
   * Travels to the given waypoint.
   * 
   * @param x x-coordinate in cm.
   * @param y y-coordinate in cm.
   * @param speedOffset coefficient by which the speed of the left motor is multiplied.
   * @param immediateReturn dictates whether the method should return immediately or wait for the
   *        motor movements to finish.
   */
  public void travelTo(double x, double y, float speedOffset, boolean immediateReturn) {

    /* Initialize local variable */
    double dX = x - odo.getX();
    double dY = y - odo.getY();
    double distance = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
    double heading = 0;

    /* Compute heading */
    if (dY >= 0) {
      heading = Math.atan(dX / dY);
    } else if (dY < 0) {
      heading = Math.PI + Math.atan(dX / dY);
    }

    /* Wrap heading around 2pi */
    if (heading > 2 * Math.PI)
      heading -= 2 * Math.PI;

    /* Turn robot to wanted heading */
    System.out.println("My current heading is " + Robot.getOdo().getTheta());
    System.out.println("I am turning to " + heading);
    turnTo(heading);
    synchronized (lock) {
      isNavigating = true;
    }

    /* Travel to x,y */
    int rotateAngle = Helper.convertDistance(Robot.WHEEL_RADIUS, distance);
    leftMotor.setSpeed(Robot.FORWARD_SPEED * speedOffset);
    rightMotor.setSpeed(Robot.FORWARD_SPEED);
    leftMotor.rotate(rotateAngle, true);
    rightMotor.rotate((int) (rotateAngle / speedOffset), immediateReturn);
  }

  /**
   * Turns to given heading using the minimum angle.
   * 
   * @param theta heading in radians.
   */
  public void turnTo(double theta) {

    double dTheta = theta - odo.getTheta(); // compute change in heading

    /* Convert negative change in heading to positive equivalent */
    if (dTheta < 0) {
      dTheta += 2 * Math.PI;
    }

    /* Determine which direction to turn in */
    if (dTheta > Math.PI) {
      dTheta = 2 * Math.PI - dTheta; // adjust dtheta when turning in negative theta direction
      turn(Math.toDegrees(dTheta), "left");
    } else {
      turn(Math.toDegrees(dTheta), "right");
    }

  }

  /**
   * Turns in given direction. The method will not return immediately.
   * 
   * @param dTheta change in heading wanted, in degrees.
   * @param direction left or right.
   */
  public void turn(double dTheta, String direction) {
    turn(dTheta, direction, false);
  }

  /**
   * Turns in given direction.
   * 
   * @param dTheta change in heading wanted, in degrees.
   * @param direction left or right.
   * @param immediateReturn dictates whether the method returns immediately
   */
  public void turn(double dTheta, String direction, boolean immediateReturn) {


    int distance = Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, dTheta); // convert angle to
                                                                                 // rotation amount

    /* Set motor speed */
    leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET);
    rightMotor.setSpeed(Robot.ROTATE_SPEED);

    /* Turn according to given direction */
    switch (direction) {
      case "left":
        leftMotor.rotate(-distance, true);
        rightMotor.rotate(distance, immediateReturn);
        break;
      case "right":
        leftMotor.rotate(distance, true);
        rightMotor.rotate(-distance, immediateReturn);
        break;
    }
  }


  /**
   * Stops the motors and signals the Navigation algorithm that another process is taking over the
   * driving of the robot by setting {@link Navigation#isAvoiding} to true.
   */
  public void pause() {
    leftMotor.stop(true);
    rightMotor.stop(true);

    synchronized (lock) {
      isAvoiding = true;
    }
  }

  /**
   * Resumes the navigation by setting {@link Navigation#isAvoiding} to false.
   */
  public void resumeNav() {
    synchronized (lock) {
      isAvoiding = false;
    }
  }

  /**
   * Indicates to navigation that a recalculation of the path is required.
   */
  public void update() {
    synchronized (lock) {
      updateNeeded = true;
    }
  }

  /**
   * @return {@link Navigation#waypointX}
   */
  public double getWaypointX() {
    return waypointX;
  }

  /**
   * @return {@link Navigation#waypointY}
   */
  public double getWaypointY() {
    return waypointY;
  }

  /**
   * Sets the destination of the navigation. This overwrites the previous destination.
   * 
   * @param x in cm
   * @param y in cm
   */
  public void setWaypoints(double x, double y) {
    synchronized (lock) {
      waypointX = x;
      waypointY = y;
      updateNeeded = true;
    }
  }

  /**
   * @return {@link Navigation#isAvoiding}
   */
  public boolean isAvoiding() {
    return isAvoiding;
  }

  /**
   * @return {@link Navigation#isNavigating}
   */
  public boolean isNavigating() {
    return isNavigating;
  }

}
