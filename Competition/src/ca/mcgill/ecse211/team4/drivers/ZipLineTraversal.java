package ca.mcgill.ecse211.team4.drivers;

import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class drives the robot across the zip line.
 * 
 * @author Anna Bieber & Kevin Laframboise
 *
 */
public class ZipLineTraversal {

  /**
   * Speed at which the zip line wheel turns, in degrees/sec.
   */
  private static final int FORWARD_SPEED = 250;

  /**
   * Radius of the zip line wheel, in cm.
   */
  public static final double WHEEL_RADIUS = 0.95;

  /**
   * Length of the zip line, in cm.
   */
  public static final double DISTANCE = 49.5;

  /**
   * Left motor, used in the zip line approach.
   */
  private EV3LargeRegulatedMotor leftMotor;
  
  /**
   * Right motor, used in the zip line approach.
   */
  private EV3LargeRegulatedMotor rightMotor; 
  
  /**
   * Zip line motor, used during traversal.
   */
  private EV3LargeRegulatedMotor lineMotor;

  /**
   * x-coordinate of the center of the zip line start.
   */
  private double x_c;

  /**
   * y-coordinate of the center of the zip line start.
   */
  private double y_c;

  /**
   * x-coordinate of the next grid line intersection in the direction of the zip line.
   */
  private double x_f0;

  /**
   * y-coordinate of the next grid line intersection in the direction of the zip line.
   */
  private double y_f0;


  /**
   * Creates a zip line traversal object.
   * 
   * @param lineMotor zip line motor.
   * @param leftMotor robot's left motor.
   * @param rightMotor robot's right motor.
   * @param x_c x coordinate of the zip line start, in cm.
   * @param y_c y coordinate of the zip line start, in cm.
   * @param x_f0 x-coordinate of the next grid line intersection in the direction of the zip line, in cm.
   * @param y_f0 y-coordinate of the next grid line intersection in the direction of the zip line, in cm.
   */
  public ZipLineTraversal(EV3LargeRegulatedMotor lineMotor, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, double x_c, double y_c, double x_f0, double y_f0) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.lineMotor = lineMotor;
    this.x_c = x_c;
    this.y_c = y_c;
    this.x_f0 = x_f0;
    this.y_f0 = y_f0;
  }

  /**
   * Traverse the zip line. This method assumes that the robot is positioned at (x_0, y_0) prior to
   * call.
   */
  public void traverse() {

    /* Start the line motor */
    lineMotor.setSpeed(FORWARD_SPEED);
    lineMotor.forward();

    /* Navigate to start of zipline */
    System.out.println("Going to: " + x_c + ", " + y_c);
    Robot.getNav().travelTo(x_c, y_c, false);
    Sound.beep();

    /* Set the driving motor to go forward. This helps the line motor to mount the zip line */
    leftMotor.setSpeed(250);
    rightMotor.setSpeed(250);
    leftMotor.rotate(1440, true);
    rightMotor.rotate(1440, false);

    lineMotor.rotate(Helper.convertDistance(WHEEL_RADIUS, 1.5 * Robot.GRID_SIZE)); // keep line
                                                                                   // motor turning
                                                                                   // until end of
                                                                                   // zip line

    /* Line up for localization after dismount */
    leftMotor.setSpeed(250);
    rightMotor.setSpeed(250);
    
    /* Go forward to clear platform */
    leftMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 25), true); 
    rightMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 25), false);
    
    /* Turn slightly to compensate for weight imbalance during dismount */
    leftMotor.rotate(-Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, 20), true); 
    rightMotor.rotate(Helper.convertAngle(Robot.WHEEL_RADIUS, Robot.TRACK, 20), false);
    
    /* Move to vicinity of next intersection */
    leftMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 20), true); 
    rightMotor.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, 20), false);

    /* Update odometer */
    Robot.getOdo().setX(x_f0);
    Robot.getOdo().setY(y_f0);

    Robot.getLightLocalizer().localize(false); // localize

  }

}
