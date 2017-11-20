package ca.mcgill.ecse211.team4.localization;

import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class uses the ultrasonic sensor to correct to odometer's theta by taking the average of two
 * angles at which wall edges are detected.
 * 
 * @author Walid Chabchoub
 */
public class UltrasonicLocalizer {

  /**
   * Sample provider for the ultrasonic localizer.
   */
  private SampleProvider us;

  /**
   * Buffer array for ultrasonic data.
   */
  private float[] usData;

  /**
   * Left motor driving the robot, used in the sweep operation.
   */
  private EV3LargeRegulatedMotor leftMotor;

  /**
   * Right motor driving the robot, used in the sweep operation.
   */
  private EV3LargeRegulatedMotor rightMotor;

  /**
   * Angle at which first wall edge is detected.
   */
  private double alphaAngle;

  /**
   * Angle at which second wall edge is detected.
   */
  private double betaAngle;

  /**
   * Angle delta between odometer's zero heading and corrected zero heading.
   */
  private double deltaTheta;

  /**
   * Indicates whether the robot is sweeping clockwise.
   */
  private boolean goingClockwise;

  /**
   * Distance in cm, as measured by the ultrasonic sensor.
   */
  private float currentDistance;

  /**
   * Creates an UltrasonicLocalizer object with given properties.
   * 
   * @param us
   * @param usData
   * @param leftMotor
   * @param rightMotor
   */
  public UltrasonicLocalizer(SampleProvider us, float[] usData, EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) {
    this.us = us;
    this.usData = usData;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    goingClockwise = true;

  }

  /**
   * Localizes the robot with respect to its distance from the wall.
   */
  public void localize() {

    /* Set motor speed */
    leftMotor.setSpeed(Robot.ROTATE_SPEED * Robot.SPEED_OFFSET);
    rightMotor.setSpeed(Robot.ROTATE_SPEED);

    /* Move to face away from wall if starting facint it */
    currentDistance = Helper.getFilteredDistance(us, usData);
    if (currentDistance < 30) { // in case we start facing the walls
      while (currentDistance < 30) { // then move away from the walls in a clockwise manner
        currentDistance = Helper.getFilteredDistance(us, usData);
        leftMotor.forward();
        rightMotor.backward();
      }

      /* Once wall is cleared by US sensor, start looking for falling edge */
      currentDistance = Helper.getFilteredDistance(us, usData);
      while (currentDistance > 30 && goingClockwise) { // now look for the closest wall clockwise
        currentDistance = Helper.getFilteredDistance(us, usData);
        leftMotor.forward();
        rightMotor.backward();
      }

      /* Stop and record the alpha angle once detected first falling edge */
      leftMotor.stop(true);
      rightMotor.stop(false);
      alphaAngle = Robot.getOdo().getTheta();
      Sound.beep();

      /* Start turning counter-clockwise to clear wall that was just detected */
      goingClockwise = false;
      leftMotor.backward();
      rightMotor.forward();
      try { // keep going counterclockwise for at least 3 sec
        Thread.sleep(3000); // give time for motors to move away from back wall
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      /* Once wall is cleared by US sensor, start looking for second falling edge */
      currentDistance = Helper.getFilteredDistance(us, usData);
      while (currentDistance > 30 && !goingClockwise) { // we're now looking for another falling
                                                        // edge on the other side
        currentDistance = Helper.getFilteredDistance(us, usData);
        leftMotor.backward(); // we're going counterclockwise
        rightMotor.forward();
      }

      /* Stop motors and record beta angle once found the second falling edge */
      leftMotor.stop(true);
      rightMotor.stop(false);
      betaAngle = Robot.getOdo().getTheta();

    } else { // this is in case we start away from the walls

      /* Start looking for falling edge */
      while (currentDistance > 30 && goingClockwise) { // look for the closest wall clockwise
        currentDistance = Helper.getFilteredDistance(us, usData);
        leftMotor.forward();
        rightMotor.backward();
      }

      /* Stop and record the alpha angle once detected first falling edge */
      leftMotor.stop(true);
      rightMotor.stop(false);
      alphaAngle = Robot.getOdo().getTheta();
      Sound.beep();

      /* Start turning counter-clockwise to clear wall that was just detected */
      goingClockwise = false;
      leftMotor.backward();
      rightMotor.forward();
      try {
        Thread.sleep(3000); // give time for motors to move away from back wall
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      /* Once wall is cleared by US sensor, start looking for second falling edge */
      currentDistance = Helper.getFilteredDistance(us, usData);
      while (currentDistance > 30 && !goingClockwise) { // we're now looking for another falling
                                                        // edge on the other side
        currentDistance = Helper.getFilteredDistance(us, usData);
        leftMotor.backward();
        rightMotor.forward();
      }

      /* Stop motors and record beta angle once found the second falling edge */
      leftMotor.stop(true); // stop motors and record beta angle
      rightMotor.stop(false);
      betaAngle = Robot.getOdo().getTheta();
      Sound.beep();
      Sound.beep();

    }

    /* Find 0 deg heading */
    if (alphaAngle > betaAngle) {
      deltaTheta = Math.PI / 4.0 - (alphaAngle + betaAngle) / 2.0 - 0.12;
    } else {
      deltaTheta = (5 * Math.PI) / 4.0 - (alphaAngle + betaAngle) / 2.0 - 0.12;
    }

    /* Update odometer */
    double currentTheta = Robot.getOdo().getTheta();
    Robot.getOdo().setTheta(currentTheta + deltaTheta); // correct the Odometer's theta value to the
                                                        // correct one
    
    Robot.getNav().turnTo(0); // turn to face 0 deg heading

  }

}
