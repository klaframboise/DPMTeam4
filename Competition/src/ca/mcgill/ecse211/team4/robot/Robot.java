package ca.mcgill.ecse211.team4.robot;

import java.io.IOException;
import java.util.Map;
import org.json.simple.parser.ParseException;
import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.drivers.ZipLineTraversal;
import ca.mcgill.ecse211.team4.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.odometry.Odometer;
import ca.mcgill.ecse211.team4.sensing.FlagDetection;
import ca.mcgill.ecse211.team4.strategy.NavigationStrategy;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * Main class.
 * 
 * @author Kevin Laframboise
 *
 */
public class Robot {

  /**
   * Size of the grid.
   */
  public static final double GRID_SIZE = 30.48;

  /**
   * Denotes the maximum possible coordinate on the square play surface.
   */
  public static final int MAX_COORD = 11;

  /**
   * Team number.
   */
  public static final int TEAM_NUMBER = 4;

  /**
   * Speed at which the robot is driven when going straight.
   */
  public static final int FORWARD_SPEED = 300;

  /**
   * Speed at which the robot is driven when rotating.
   */
  public static final int ROTATE_SPEED = 250;

  /**
   * Offset to compensate for mechanical design weight imbalance.
   */
  public static final float SPEED_OFFSET = 0.99f;

  /**
   * Red intensity of a grid line.
   */
  public static final float LINE_RED_INTENSITY = 0.30f;

  /**
   * Red intensity when the sensor almost sticks to the platform
   */
  public static final float PLATFORM_RED_INTENSITY = 0.1f;

  /**
   * Track width of the robot.
   */
  public static final double TRACK = 13.85;

  /**
   * Radius of the robot's wheels.
   */
  public static final double WHEEL_RADIUS = 2.1;

  /**
   * The robot's navigation.
   */
  private static Navigation nav;

  /**
   * The robot's odometer.
   */
  private static Odometer odo;

  /**
   * The robot's light localizer.
   */
  private static LightLocalizer lightLocalizer;

  /**
   * 
   */
  private static UltrasonicLocalizer usLocalizer;

  /**
   * Map containing the game parameters.
   */
  private static Map<String, Long> gameParameters;

  /**
   * Can either be "Green" or "Red". This will affect strategies.
   */
  private static String teamColor;

  /**
   * Instance of the left driving motor.
   */
  private static EV3LargeRegulatedMotor leftMotor;

  /**
   * Instance of the right driving motor.
   */
  private static EV3LargeRegulatedMotor rightMotor;

  /**
   * Instance of the zip line motor.
   */
  private static EV3LargeRegulatedMotor lineMotor;

  /**
   * Instance of the servo motor.
   */
  private static EV3LargeRegulatedMotor servo;

  /**
   * Instance of the zip line driver.
   */
  private static ZipLineTraversal ziplineTraversal;

  /**
   * Instance of the flag detection.
   */
  private static FlagDetection flagDetection;


  @SuppressWarnings("resource") // we don't bother closing the sensor port because they are used
                                // throughout the lifetime of the class.
  /**
   * Constructs the robot. Initializes all components, sensors and gets game parameters. This should
   * be the first procedure called in any outside main method (i.e. tests) in order to avoid
   * NullPointerExceptions and similar problems.
   */
  public Robot() {

    /* Initialize components */
    leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    lineMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    servo = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    odo = new Odometer(leftMotor, rightMotor);
    nav = new Navigation(odo, leftMotor, rightMotor);

    /* Initialize ultrasonic sensor */
    float[] usData = new float[10];
    SampleProvider us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1")).getMode("Distance");
    usLocalizer = new UltrasonicLocalizer(us, usData, leftMotor, rightMotor);

    /* Initialize localization color sensor */
    float[] lightData = new float[10];
    SampleProvider lightSampler = new EV3ColorSensor(LocalEV3.get().getPort("S4")).getMode("Red");
    lightLocalizer = new LightLocalizer(lightSampler, lightData, leftMotor, rightMotor, false);

    /* Initialize flag detection color sensor */
    float[] colorData = new float[9];
    SampleProvider colorSampler =
        new EV3ColorSensor(LocalEV3.get().getPort("S2")).getMode("ColorID");

    /* Get game parameters */
    try {
      gameParameters = GameSetup.getGameParameters(true);
    } catch (IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      gameParameters = null;
    }

    /* Initialize team color */
    teamColor = (gameParameters.get("RedTeam").intValue() == TEAM_NUMBER) ? "Red" : "Green";

    /* Initialize flag detection */
    flagDetection = new FlagDetection(colorSampler, colorData, us, usData,
        gameParameters.get("O" + teamColor.charAt(0)).intValue(), gameParameters);

    /* Initialize zipline traversal */
    double x_c = gameParameters.get("ZC_G_x").intValue();
    double y_c = gameParameters.get("ZC_G_y").intValue();
    double x_f0 = gameParameters.get("ZO_R_x").intValue();
    double y_f0 = gameParameters.get("ZO_R_y").intValue();
    ziplineTraversal = new ZipLineTraversal(lineMotor, leftMotor, rightMotor, x_c, y_c, x_f0, y_f0);

    /* Set motor attributes */
    leftMotor.setAcceleration(500);
    rightMotor.setAcceleration(500);

  }

  /**
   * Launches the game logic after Robot is constructed.
   * 
   * @param args
   */
  public static void main(String[] args) {

    new Robot(); // construct robot in order to initialize fields

    /* Initialize display */
    Display display = new Display(odo);
    display.start();


    /* Start game */
    odo.start();
    nav.start();

    /* Localize */
    usLocalizer.localize();
    lightLocalizer.localize(gameParameters.get(teamColor + "Corner").intValue());

    /* Follow navigation strategy */
    NavigationStrategy navStrat = new NavigationStrategy(teamColor, gameParameters, nav);
    navStrat.navigateToObjectiveZone(); // navigate to objective
    flagDetection.searchAndDetect(); // detect flag
    navStrat.navigateBack(); // navigate back to start
    Sound.playNote(Sound.FLUTE, Sound.DOUBLE_BEEP, 500); // celebrate

  }

  /**
   * 
   * @return {@link Robot#nav}
   */
  public static Navigation getNav() {
    return nav;
  }

  /**
   * 
   * @return {@link Robot#odo}
   */
  public static Odometer getOdo() {
    return odo;
  }

  /**
   * 
   * @return {@link Robot#lightLocalizer}
   */
  public static LightLocalizer getLightLocalizer() {
    return lightLocalizer;
  }

  /**
   * 
   * @return {@link Robot#usLocalizer}
   */
  public static UltrasonicLocalizer getUSLocalizer() {
    return usLocalizer;
  }

  /**
   * 
   * @return {@link Robot#gameParameters}
   */
  public static Map<String, Long> getGameParameters() {
    return gameParameters;
  }

  /**
   * 
   * @return {@link Robot#teamColor}
   */
  public static String getTeamColor() {
    return teamColor;
  }

  /**
   * 
   * @return array containing driving motors. Index 0 contains the leftMotor, 1, the rightMotor.
   */
  public static EV3LargeRegulatedMotor[] getDrivingMotors() {
    EV3LargeRegulatedMotor[] motors = {leftMotor, rightMotor};
    return motors;
  }

  /**
   * 
   * @return {@link Robot#lineMotor}
   */
  public static EV3LargeRegulatedMotor getLineMotor() {
    return lineMotor;
  }

  /**
   * 
   * @return {@link Robot#servo}
   */
  public static EV3LargeRegulatedMotor getServo() {
    return servo;
  }

  /**
   * 
   * @return {@link Robot#ziplineTraversal}
   */
  public static ZipLineTraversal getZipLineTraversal() {
    return ziplineTraversal;
  }

  /**
   * 
   * @return {@link Robot#flagDetection}
   */
  public static FlagDetection getFlagDetection() {
    return flagDetection;
  }
}
