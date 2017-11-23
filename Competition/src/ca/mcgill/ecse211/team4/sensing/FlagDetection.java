package ca.mcgill.ecse211.team4.sensing;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class searches and detects the objective flag.
 * 
 * @author Anna Bieber & Kevin Laframboise
 *
 */
public class FlagDetection {

  /**
   * Color mapping from sensor return values (index) to server color values
   * (element).
   */
  private static final int[] colorMapping = { 1, 0, 2, 3, 0, 0, 4, 0 };
  /**
   * Sample provider for the color sensor. Sample Provider must be in color ID
   * mode.
   */
  private SampleProvider colorSampler;

  /**
   * Buffer array for the color sensor data.
   */
  private float[] colorData;

  /**
   * Color ID of the objective.
   */
  private int objectiveColorID;

  /**
   * x-coordinate of the lower left corner of the search zone.
   */
  private int zone_LL_x;

  /**
   * y-coordinate of the lower left corner of the search zone.
   */
  private int zone_LL_y;

  /**
   * x-coordinate of the upper right corner of the search zone.
   */
  private int zone_UR_x;

  /**
   * y-coordinate of the upper right corner of the search zone.
   */
  private int zone_UR_y;

  /**
   * Sample provider for the ultrasonic localizer.
   */
  private SampleProvider us;

  /**
   * Buffer array for ultrasonic data.
   */
  private float[] usData;

  /**
   * Creates a FlagDetection object with given parameter
   * 
   * @param colorSampler
   * @param colorData
   * @param us
   * @param usData
   * @param objectiveColorID
   * @param gameParameters
   *            Map containing the game parameters. Used to determine the
   *            search zone.
   */
  public FlagDetection(SampleProvider colorSampler, float[] colorData, SampleProvider us, float[] usData,
      int objectiveColorID, Map<String, Long> gameParameters) {
    this.colorSampler = colorSampler;
    this.colorData = colorData;
    this.objectiveColorID = objectiveColorID;
    this.us = us;
    this.usData = usData;
    Robot.getDrivingMotors()[0] = Robot.getDrivingMotors()[0];
    Robot.getDrivingMotors()[1] = Robot.getDrivingMotors()[1];

    /* Get the search zone and objective color from the game parameters */
    if (Robot.getTeamColor().equals("Green")) {
      zone_LL_x = gameParameters.get("SR_LL_x").intValue();
      zone_LL_y = gameParameters.get("SR_LL_y").intValue();
      zone_UR_x = gameParameters.get("SR_UR_x").intValue();
      zone_UR_y = gameParameters.get("SR_UR_y").intValue();
      objectiveColorID = Robot.getGameParameters().get("OR").intValue();
    } else {
      zone_LL_x = gameParameters.get("SG_LL_x").intValue();
      zone_LL_y = gameParameters.get("SG_LL_y").intValue();
      zone_UR_x = gameParameters.get("SG_UR_x").intValue();
      zone_UR_y = gameParameters.get("SG_UR_y").intValue();
      objectiveColorID = Robot.getGameParameters().get("OG").intValue();
    }

    /* Set servo speed */
    Robot.getServo().setSpeed(45);
  }

  /**
   * Searches for the objective and detects flag by beeping 3 times.
   */
  public void searchAndDetect() {
    int numberBlockDetected = 0;
    /* Initializing local variables */
    int currentDistance;
    int coordinates[][] = points();
    Robot.getServo().rotateTo(60, true); // rotate servo to the left

    /*
     * Travel to the four corners of the search zone while checking
     * ultrasonic for close-by objects
     */
    long start = System.currentTimeMillis();
    int i = 0;
    while (true) {
      /* Start navigating around search zone and return immediately */
      System.out.println("Travelling to (" + coordinates[i % 4][0] + ", " + coordinates[i % 4][1] + ")");
      Robot.getNav().travelTo(coordinates[i % 4][0] * Robot.GRID_SIZE, coordinates[i % 4][1] * Robot.GRID_SIZE, false);

      /* Poll ultrasonic sensor */
      /*currentDistance = Helper.getFilteredDistance(us, usData);
			while (currentDistance > 35 && Robot.getDrivingMotors()[0].isMoving()) {
				currentDistance = Helper.getFilteredDistance(us, usData);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}*/

      if(System.currentTimeMillis() - start > 30000) {
        break;
      }

      i++;

      /*
       * Check if loop broke because object was detected
       * Update number of blocks detected accordingly each time we see a block
       * 
       * New lines of code implemented to avoid getting stuck on the same block over and over again
       * If we have detected a block and it's not the block we want,
       * go back to the previous position, and move forward for 1/3 tile, and we search again.
       */
      //			if (currentDistance < 35) {
      //				System.out.println("Block detected with distance " + currentDistance);
      //				/* Stop motors if this is the first block seen */
      //				//Wenjie's method
      //				if (numberBlockDetected == 0) {
      //					numberBlockDetected++;
      //					Robot.getDrivingMotors()[0].stop(true);
      //					Robot.getDrivingMotors()[1].stop(true);
      //					Robot.getServo().rotateTo(0);
      //
      //					// locateFlag();
      //					boolean flagCaptured = captureFlag();
      //					if (flagCaptured)
      //						break;
      //				} else {
      //					numberBlockDetected = 0;
      //					if (Helper.calculateDistanceError(coordinates[i][0] * Robot.GRID_SIZE,
      //							coordinates[i][1] * Robot.GRID_SIZE) > Robot.GRID_SIZE / 2) {
      //						Robot.getDrivingMotors()[0]
      //								.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, Robot.GRID_SIZE / 3), true);
      //						Robot.getDrivingMotors()[1]
      //								.rotate(Helper.convertDistance(Robot.WHEEL_RADIUS, Robot.GRID_SIZE / 3), false);
      //					} else {
      //						Robot.getNav().travelTo(coordinates[i][0] * Robot.GRID_SIZE,
      //								coordinates[i][1] * Robot.GRID_SIZE, false);
      //					}
      //				}
      //				
      //				//Kevin's method
      //				Robot.getDrivingMotors()[0].stop(true);
      //				Robot.getDrivingMotors()[1].stop(true);
      //				Robot.getServo().rotateTo(0);
      //				
      //				locateFlag();
      //				
      //				i--; // continue with previous waypoint
    }
    Robot.getServo().rotateTo(0);
  }

  /**
   * This method will locate the flag by sweeping the ultrasonic sensor. The
   * angle at which the measured distance is smallest is the heading towards
   * which the object is. It will then drive the robot towards the flag and
   * poll the color sensor to see if the color of the flag is the one we are
   * looking for.
   * 
   * @return true if our flag is located
   */
  private boolean locateFlag() {

    /* Initialize local variable */
    double relHeadingToObject = getHeadingToObject();
    int colorReading = getColorId(colorSampler, colorData);
    boolean flagDetected = false;

    /* Turn towards the detected object */
    Robot.getNav().turn(relHeadingToObject, "left");
    Robot.getServo().rotateTo(0, false);

    /*
     * Set the motor speed very slow so as to not push the blocks out of the
     * box
     */
    Robot.getDrivingMotors()[0].setSpeed(100);
    Robot.getDrivingMotors()[1].setSpeed(100);

    /*
     * Drive robot forward until one of the four colors are detected. This
     * will happen when the color sensor is very close to the flag
     */
    int startTachoCount = Robot.getOdo().getLeftMotorTachoCount();
    while (true) {

      /* Get motors going */
      Robot.getDrivingMotors()[0].forward();
      Robot.getDrivingMotors()[1].forward();

      colorReading = getColorId(colorSampler, colorData); // poll color
      // sensor
      /* Check if reading corresponds to one of the possible flags */
      if (colorReading >= 0 && colorReading <= 7) {
        if (colorMapping[colorReading] != 0) {
          Robot.getDrivingMotors()[0].stop(true);
          Robot.getDrivingMotors()[1].stop(true);
          System.out.println("Color read: " + colorMapping[colorReading]);
          System.out.println("Looking for: " + objectiveColorID);
          break;
        }
      }
    }
    int dTacho = Robot.getOdo().getLeftMotorTachoCount() - startTachoCount;

    /* Check if this is our flag */
    if (colorMapping[colorReading] == objectiveColorID) {
      Sound.beep();
      Sound.beep();
      Sound.beep();
      flagDetected = true; // our flag is detected
    }

    /* Go back to where the robot was when locateFlag() was called */
    Robot.getDrivingMotors()[0].setSpeed(Robot.FORWARD_SPEED);
    Robot.getDrivingMotors()[1].setSpeed(Robot.FORWARD_SPEED);
    Robot.getDrivingMotors()[0].rotate(-dTacho, true);
    Robot.getDrivingMotors()[1].rotate(-dTacho, false);

    Robot.getServo().rotateTo(60); // put servo back in detecting position

    return flagDetected; // our flag wasn't detected

  }

  /**
   * This method returns, in degrees, the angle, as reported by the
   * getPosition() method of the servo motor at which the smallest distance
   * was detected. This angle should be the heading required to go towards the
   * object.
   * 
   * @return heading to the object, relative to the current heading of the
   *         robot, in degrees.
   */
  private double getHeadingToObject() {

    /* Initialize local variables */
    Map<Float, Integer> sweepResults = new HashMap<Float, Integer>();
    Iterator<Float> it;
    int minDistance = Integer.MAX_VALUE;
    int tmpDistance;
    float angleToMinDistance = 0;
    Float tmpAngle;

    /* Prepare servo for sweep */
    Robot.getServo().rotateTo(0, false);

    /* Get data by sweeping */
    Robot.getNav().turn(90, "left", true);
    while (Robot.getDrivingMotors()[0].isMoving()) {
      sweepResults.put(new Float(Robot.getOdo().getTheta()), Helper.getFilteredDistance(us, usData));
    }

    /* Find minimum distance */
    it = sweepResults.keySet().iterator();
    while (it.hasNext()) {
      tmpAngle = it.next();
      tmpDistance = sweepResults.get(tmpAngle).intValue();
      System.out.println("Distance at: " + tmpAngle.floatValue() + " is " + tmpDistance);
      if (tmpDistance < minDistance) {
        minDistance = tmpDistance;
        angleToMinDistance = tmpAngle.floatValue();
      }
    }

    return angleToMinDistance;

  }

  private boolean captureFlag() {
    boolean flagCaptured = false;
    int colorReading = getColorId(colorSampler, colorData);
    Robot.getNav().turnTo(Robot.getOdo().getTheta() - Math.PI / 4);
    Robot.getServo().rotateTo(0, false);

    Robot.getDrivingMotors()[0].setSpeed(200);
    Robot.getDrivingMotors()[1].setSpeed(200);

    int startLeftTachoCount = Robot.getDrivingMotors()[0].getTachoCount();
    int startRightTachoCount = Robot.getDrivingMotors()[1].getTachoCount();
    int dLeftTacho = 0;
    int dRightTacho = 0;

    while (true) {
      Robot.getDrivingMotors()[0].forward();
      Robot.getDrivingMotors()[1].forward();

      colorReading = getColorId(colorSampler, colorData);

      dLeftTacho = Robot.getDrivingMotors()[0].getTachoCount() - startLeftTachoCount;
      dRightTacho = Robot.getDrivingMotors()[1].getTachoCount() - startRightTachoCount;
      /*
       * Keep track of the tacho count of both motors If the robot have
       * traveled a long distance and still didn't detect any color, then
       * simply reverse TODO: find which value works the best
       */
      if (dLeftTacho > Helper.convertDistance(Robot.WHEEL_RADIUS, 20)) {
        Robot.getDrivingMotors()[0].stop(true);
        Robot.getDrivingMotors()[1].stop(false);
        break;
      }

      /* Check if the color detected is the desired one */
      if (colorReading >= 0 && colorReading <= 6) {
        Robot.getDrivingMotors()[0].stop(true);
        Robot.getDrivingMotors()[1].stop(true);
        System.out.println("Color ID found is: " + colorReading);
        break;
      }
    }

    if (colorReading == objectiveColorID) {
      flagCaptured = true;
      for (int i = 0; i < 3; i++)
        Sound.beep(); // beeps three times to indicate capturing
    } else {
      Robot.getServo().rotateTo(60, false);
    }

    Robot.getDrivingMotors()[0].setSpeed(Robot.FORWARD_SPEED);
    Robot.getDrivingMotors()[1].setSpeed(Robot.FORWARD_SPEED);
    Robot.getDrivingMotors()[0].rotate(-dLeftTacho, true);
    Robot.getDrivingMotors()[1].rotate(-dRightTacho, false);
    return flagCaptured;
  }

  /**
   * This method creates a bidimensional array containing the path the robot
   * must follow to go all around the search zone in a rectangle.
   * 
   * @return a bidimensional array containing the waypoints to travel to.
   */
  public int[][] points() {

    /* Get all the points of the rectangle */
    int zone_UL_x = zone_LL_x;
    int zone_UL_y = zone_UR_y;
    int zone_LR_x = zone_UR_x;
    int zone_LR_y = zone_LL_y;

    /* Get the current value of X and Y, in cm */
    double currentX = Robot.getOdo().getX();
    double currentY = Robot.getOdo().getY();

    /*
     * If we started on the green side and our robot enters the searching
     * area on the x axis
     */
    if (currentX < (zone_LR_x * Robot.GRID_SIZE) && Robot.getTeamColor() == "Green") {
      int coordinates[][] = { { zone_LR_x, zone_LR_y }, { zone_UR_x, zone_UR_y }, { zone_UL_x, zone_UL_y },
          { zone_LL_x, zone_LL_y } };
      return coordinates;
    }
    /*
     * If we started on the red side and our robot enters the searching area
     * on the x axis
     */
    else if (currentX < (zone_LR_x * Robot.GRID_SIZE) && Robot.getTeamColor() == "Red") {
      int coordinates[][] = { { zone_UL_x, zone_UL_y }, { zone_LL_x, zone_LL_y }, { zone_LR_x, zone_LR_y },
          { zone_UR_x, zone_UR_y } };
      return coordinates;

    }
    /*
     * If we started on the green side and our robot enters the searching
     * area on the y axis
     */
    else if (currentY < (zone_UR_y * Robot.GRID_SIZE) && Robot.getTeamColor() == "Green") {
      int coordinates[][] = { { zone_UR_x, zone_UR_y }, { zone_UL_x, zone_UL_y }, { zone_LL_x, zone_LL_y },
          { zone_LR_x, zone_LR_y } };
      return coordinates;
    }

    /*
     * If none of the cases above apply there's only one possibility left we
     * started on the red side and our robot enters the searching area on
     * the y axis
     */
    int coordinates[][] = { { zone_LL_x, zone_LL_y }, { zone_LR_x, zone_LR_y }, { zone_UR_x, zone_UR_y },
        { zone_UL_x, zone_UL_y } };
    return coordinates;

  }

  /**
   * Returns the median of a set of samples taken by the color sensor in
   * ColorID mode.
   * 
   * @param colorSampler
   * @param colorData
   * @return the ColorID mapping
   */
  public static int getColorId(SampleProvider colorSampler, float[] colorData) {

    /* Acquire data */
    for (int i = 0; i < colorData.length; i += colorSampler.sampleSize()) {
      colorSampler.fetchSample(colorData, i * colorSampler.sampleSize());
    }

    /* Compute and return median */
    Arrays.sort(colorData); // sort array
    return (int) colorData[colorData.length / 2]; // return median
  }
}
