package ca.mcgill.ecse211.team4.strategy;

import java.util.Map;
import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.drivers.ZipLineTraversal;
import ca.mcgill.ecse211.team4.localization.LightLocalizer;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.Sound;

/**
 * This class implements a strategy to get to the objective and back to starting point.
 * 
 * @author Kevin Laframboise
 *
 */
public class NavigationStrategy {

  /**
   * Left wheel speed offset to be used on shallow crossing
   */
  public static final float CROSSING_OFFSET = 0.985f;

  /**
   * Must be "Red" or "Green". This will change the strategy used.
   */
  private String teamColor;

  /**
   * Map containing the game parameters.
   */
  private Map<String, Long> gameParameters;

  /**
   * Navigation object used by strategy.
   */
  private Navigation nav;

  /**
   * First character of our team's color.
   */
  private char teamChar;

  /**
   * First character of the other team's color.
   */
  private char otherTeam;

  /**
   * Creates a NavigationStrategy object with the given parameters.
   * 
   * @param teamColor "red" or "green"
   * @param gameParameters
   * @param nav
   */
  public NavigationStrategy(String teamColor, Map<String, Long> gameParameters, Navigation nav) {
    this.teamColor = teamColor;
    this.gameParameters = gameParameters;
    this.nav = nav;
    teamChar = teamColor.charAt(0); // extract first character of our team's color
    otherTeam = (teamChar == 'G') ? 'R' : 'G'; // determine the other team's first character
  }

  /**
   * Uses the navigation strategy dictated by team color to navigate to the objective.
   */
  public void navigateToObjectiveZone() {

    if (teamColor.equals("Green"))
      navToObjectiveByZip();
    else
      navToObjectiveByCrossing();
  }

  /**
   * Uses the navigation strategy dictated by team color to navigate back to the starting point.
   */
  public void navigateBack() {

    if (teamColor.equals("Green"))
      navBackByCrossing();
    else
      navBackByZip();
  }

  /**
   * Navigates to the objective using the zip line.
   */
  private void navToObjectiveByZip() {

    /* Initialize game parameters variables */
    int Z0_x = gameParameters.get("ZO_" + teamChar + "_x").intValue();
    int Z0_y = gameParameters.get("ZO_" + teamChar + "_y").intValue();
    int ZC_x = gameParameters.get("ZC_" + teamChar + "_x").intValue();
    int ZC_y = gameParameters.get("ZC_" + teamChar + "_y").intValue();
    int[] zipLineEndPoints = getZipLineEndpoints(Z0_x, Z0_y, ZC_x, ZC_y);
    int ZF0_x = zipLineEndPoints[2];
    int ZF0_y = zipLineEndPoints[3];
    int[][] objCoords =
        Helper.convertToFourCorners(gameParameters.get("S" + otherTeam + "_UR_x").intValue(),
            gameParameters.get("S" + otherTeam + "_UR_y").intValue(),
            gameParameters.get("S" + otherTeam + "_LL_x").intValue(),
            gameParameters.get("S" + otherTeam + "_LL_y").intValue()); // convert the objective
                                                                       // search region to a set of
                                                                       // 4 points

    /* Initialize zip line driver */
    ZipLineTraversal traversal = new ZipLineTraversal(Robot.getLineMotor(),
        Robot.getDrivingMotors()[0], Robot.getDrivingMotors()[1], ZC_x * Robot.GRID_SIZE,
        ZC_y * Robot.GRID_SIZE, ZF0_x * Robot.GRID_SIZE, ZF0_y * Robot.GRID_SIZE);

    /* Travel to start of zip line */
    travelTo((Z0_x - 0.1) * Robot.GRID_SIZE, (Z0_y - 0.1) * Robot.GRID_SIZE, true); // 0.1 offset
                                                                                    // helps ensure
                                                                                    // color sensor
                                                                                    // is in thrid
                                                                                    // quadrant
    Robot.getLightLocalizer().localize();
    travelTo(Z0_x * Robot.GRID_SIZE, Z0_y * Robot.GRID_SIZE, false);

    /* Traverse zip line */
    traversal.traverse();

    /* Travel to search area */
    int closestCorner = findClosestCorner(objCoords); // find the closest corner of the search area
    travelTo((objCoords[closestCorner][0] - 0.1) * Robot.GRID_SIZE,
        (objCoords[closestCorner][1] - 0.1) * Robot.GRID_SIZE, true); // travel to closest corner
    Robot.getLightLocalizer().localize();

  }


  /**
   * Navigates to the objective using the shallow crossing.
   */
  private void navToObjectiveByCrossing() {

    /* Cross */
    double[][] path = getCrossingPath(); // get path waypoints
    localizeBeforeCrossing(path); // localize before shallow crossing
    for (double[] point : path) { // cross point-by point
      Robot.getNav().travelTo(point[0] * Robot.GRID_SIZE, point[1] * Robot.GRID_SIZE,
          CROSSING_OFFSET, false);
    }

    clearCrossing(path); // go forward until back of robot clears crossing
    Robot.getLightLocalizer().localize(false); // localize

    /* Get search area */
    int[][] objCoords =
        Helper.convertToFourCorners(gameParameters.get("S" + otherTeam + "_UR_x").intValue(),
            gameParameters.get("S" + otherTeam + "_UR_y").intValue(),
            gameParameters.get("S" + otherTeam + "_LL_x").intValue(),
            gameParameters.get("S" + otherTeam + "_LL_y").intValue()); // convert the objective
                                                                       // search region to a set of
                                                                       // 4 points
    /* Travel to search area */
    int closestCorner = findClosestCorner(objCoords); // find the closest corner of the search area
    travelTo((objCoords[closestCorner][0] - 0.1) * Robot.GRID_SIZE,
        (objCoords[closestCorner][1] - 0.1) * Robot.GRID_SIZE, true); // travel to closest corner
    Robot.getLightLocalizer().localize();

  }

  /**
   * Navigates back to the start using the zip line.
   */
  private void navBackByZip() {

    /* Initialize game parameters variables */
    int Z0_x = gameParameters.get("ZO_" + otherTeam + "_x").intValue();
    int Z0_y = gameParameters.get("ZO_" + otherTeam + "_y").intValue();
    int ZC_x = gameParameters.get("ZC_" + otherTeam + "_x").intValue();
    int ZC_y = gameParameters.get("ZC_" + otherTeam + "_y").intValue();
    int[] zipLineEndPoints = getZipLineEndpoints(Z0_x, Z0_y, ZC_x, ZC_y);
    int ZF0_x = zipLineEndPoints[2];
    int ZF0_y = zipLineEndPoints[3];
    double startX = findStart()[0]; // x coord of our team's starting pos
    double startY = findStart()[1]; // y coord of our team's starting pos

    /* Initialize zip line driver */
    ZipLineTraversal traversal = new ZipLineTraversal(Robot.getLineMotor(),
        Robot.getDrivingMotors()[0], Robot.getDrivingMotors()[1], ZC_x * Robot.GRID_SIZE,
        ZC_y * Robot.GRID_SIZE, ZF0_x * Robot.GRID_SIZE, ZF0_y * Robot.GRID_SIZE);

    /* Travel to start of zip line */
    travelTo((Z0_x - 0.1) * Robot.GRID_SIZE, (Z0_y - 0.1) * Robot.GRID_SIZE, true);
    Robot.getLightLocalizer().localize();
    travelTo(Z0_x * Robot.GRID_SIZE, Z0_y * Robot.GRID_SIZE, false);

    /* Traverse zip line */
    traversal.traverse();

    /* Travel to start */
    travelTo(startX * Robot.GRID_SIZE, startY * Robot.GRID_SIZE, false);

  }

  /**
   * Navigates back to the start using the shallow crossing.
   */
  private void navBackByCrossing() {

    /* Cross */
    double[][] path = getCrossingPath(); // get path waypoints
    localizeBeforeCrossing(path); // localize prior to crossing
    for (double[] point : path) { // cross point-by-point
      Robot.getNav().travelTo(point[0] * Robot.GRID_SIZE, point[1] * Robot.GRID_SIZE,
          CROSSING_OFFSET, false);
    }

    clearCrossing(path); // go forward until back of the robot has cleared the shallow crossing
    Robot.getLightLocalizer().localize(false); // localize

    /* Travel to start area */
    double startX = findStart()[0];
    double startY = findStart()[1];
    travelTo(startX * Robot.GRID_SIZE, startY * Robot.GRID_SIZE, false);
  }

  /**
   * This method divides the given path in shorter paths where the robot will localize after
   * reaching the end of each shorter path. This method calls itself recursively until the original
   * x and y are reached.
   * 
   * @param x destination-x, in cm
   * @param y destination-y, in cm
   * @param localize indicates whether the robot is allowed to localize during travel to operation
   */
  private void travelTo(double x, double y, boolean localize) {

    /* Initialize local variables */
    final double LOCALIZATION_FREQ = 4 * Robot.GRID_SIZE; // localize every LOCALIZATION_FREQ cm in
                                                          // a given direction
    boolean localizationRequired = false;
    double currentX = Robot.getOdo().getX();
    double currentY = Robot.getOdo().getY();
    double waypointX = x;
    double waypointY = y;
    int posX = (currentX < x) ? 1 : -1; // indicates if the x waypoint is in the positive x
                                        // direction w.r.t. currentX
    int posY = (currentY < y) ? 1 : -1; // indicates if the y waypoint is in the positive y
                                        // direction w.r.t. currentY

    /*
     * Check if x is far enough to require a localization If so, then set a new waypoint closer
     * after which localization will be performed
     */
    if (Math.abs(currentX - x) > LOCALIZATION_FREQ && localize) {
      waypointX = currentX + (LOCALIZATION_FREQ * posX);
      localizationRequired = true;
    }

    if (Math.abs(currentY - y) > LOCALIZATION_FREQ && localize) {
      waypointY = currentY + (LOCALIZATION_FREQ * posY);
      localizationRequired = true;
    }

    /* Travel to waypoint and wait for nav to end */
    nav.travelTo(waypointX, waypointY, false);

    /* Localize if required and continue navigating to original waypoint */
    if (localizationRequired) {
      Robot.getLightLocalizer().localize();
      travelTo(x, y, localize);
    }

  }

  /**
   * This method returns, in an array of integers, the waypoints at the end of the zipline. The
   * array is structured as follows:
   * 
   * <pre>
   * Index   Value
   * 0       x of zip line end point
   * 1       y of zip line end point
   * 2       x of "other" end point
   * 3       y of "other" end point
   * </pre>
   * 
   * @param z0_x x of "other" start point.
   * @param z0_y y of "other" start point.
   * @param zC_x x of zip line start point.
   * @param zC_y y of zip line start point.
   * @return int array containing zip line endpoints
   */
  public static int[] getZipLineEndpoints(int z0_x, int z0_y, int zC_x, int zC_y) {

    int[] result = new int[4]; // initialize result array

    /* Get direction of the zip line by comparing start point and "other" start point */
    int dx = ((zC_x - z0_x) != 0) ? (zC_x - z0_x) / Math.abs(zC_x - z0_x) : zC_x - z0_x; // difference
                                                                                         // in x
                                                                                         // between
                                                                                         // start
                                                                                         // and
                                                                                         // "other"
                                                                                         // start
    int dy = ((zC_y - z0_y) != 0) ? (zC_y - z0_y) / Math.abs(zC_y - z0_y) : zC_y - z0_y; // difference
                                                                                         // in y
                                                                                         // between
                                                                                         // start
                                                                                         // and
                                                                                         // "other"
                                                                                         // start

    /* Case where zip line is along x or y-axis */
    if (dx == 0 || dy == 0) {
      result[0] = zC_x + (dx * 4);
      result[1] = zC_y + (dy * 4);
    }

    /* Case where zip line is diagonal */
    else {
      result[0] = zC_x + (dx * 3);
      result[1] = zC_y + (dy * 3);
    }

    /* Compute other endpoints, one foot away in the direction of the zipline */
    result[2] = result[0] + dx;
    result[3] = result[1] + dy;

    return result;
  }

  /**
   * Finds the closest corner to the robot's current position of a rectangular area.
   * 
   * @param an array containing 4 points following the structure outlined in
   *        {@link Helper#convertToFourCorners(int, int, int, int)}
   * @return the index of the closest point
   */
  private int findClosestCorner(int[][] rectArea) {

    /* Initialize local variables */
    double currentX = Robot.getOdo().getX();
    double currentY = Robot.getOdo().getY();
    double distance;
    double minimumDistance = Double.MAX_VALUE;
    int closestCorner = 0;

    /* Find closest corner */
    for (int i = 0; i < rectArea.length; i++) {

      distance = Math.sqrt(Math.pow(currentX - rectArea[i][0] * Robot.GRID_SIZE, 2)
          + Math.pow(currentY - rectArea[i][1] * Robot.GRID_SIZE, 2)); // calculate distance to
                                                                       // corner

      if (distance < minimumDistance) {
        minimumDistance = distance; // record as minimum distance if smaller than previous minimum
        closestCorner = i; // record corner as being the closest
      }

    }

    return closestCorner;
  }

  /**
   * This method returns the coordinates of the starting point
   * 
   * @return an array containing the coordinates. x at i = 0, y at i = 1.
   */
  private double[] findStart() {

    double[] start = new double[2];

    switch (gameParameters.get(teamColor + "Corner").intValue()) {
      case 0:
        start[0] = 0.5;
        start[1] = 0.5;
        break;
      case 1:
        start[0] = Robot.MAX_COORD + 0.5;
        start[1] = 0.5;
        break;
      case 2:
        start[0] = Robot.MAX_COORD + 0.5;
        start[1] = Robot.MAX_COORD + 0.5;
        break;
      case 3:
        start[0] = 0.5;
        start[1] = Robot.MAX_COORD + 0.5;
        break;
    }

    return start;
  }

  /**
   * Computes the 3-waypoints path to be taken in order to successfully cross the shallow crossing.
   * The robot will buzz if it detects inconsistencies in the definition of the zones and of the
   * crossing (eg.: if the crossing does not start where a navigable zone ends).
   * 
   * @return the path to follow on the crossing in a bidiemnsional array path[i][j] where i is the
   *         waypoint number and j, x or y.
   */
  private double[][] getCrossingPath() {

    double[][] path = new double[3][2]; // result array

    /* Get shallow crossing and zones properties */
    int r_ll_x = gameParameters.get("Red_LL_x").intValue();
    int r_ll_y = gameParameters.get("Red_LL_y").intValue();
    int r_ur_x = gameParameters.get("Red_UR_x").intValue();
    int r_ur_y = gameParameters.get("Red_UR_y").intValue();
    int g_ll_x = gameParameters.get("Green_LL_x").intValue();
    int g_ll_y = gameParameters.get("Green_LL_y").intValue();
    int g_ur_x = gameParameters.get("Green_UR_x").intValue();
    int g_ur_y = gameParameters.get("Green_UR_y").intValue();
    int sv_ll_x = gameParameters.get("SV_LL_x").intValue();
    int sv_ll_y = gameParameters.get("SV_LL_y").intValue();
    int sv_ur_x = gameParameters.get("SV_UR_x").intValue();
    int sv_ur_y = gameParameters.get("SV_UR_y").intValue();
    int sh_ll_x = gameParameters.get("SH_LL_x").intValue();
    int sh_ll_y = gameParameters.get("SH_LL_y").intValue();
    int sh_ur_x = gameParameters.get("SH_UR_x").intValue();
    int sh_ur_y = gameParameters.get("SH_UR_y").intValue();

    /*
     * For the first waypoint, there are 4 possibilities. The crossing can start on the left, top,
     * right or bottom of the red zone.
     */
    if (r_ur_x == sh_ll_x) {
      path[0][0] = sh_ll_x;
      path[0][1] = sh_ll_y + 0.5;
    } else if (r_ll_x == sh_ur_x) {
      path[0][0] = sh_ur_x;
      path[0][1] = sh_ur_y - 0.5;
    } else if (r_ll_y == sv_ur_y) {
      path[0][0] = sv_ur_x - 0.5;
      path[0][1] = sv_ur_y;
    } else if (r_ur_y == sv_ll_y) {
      path[0][0] = sv_ur_x + 0.5;
      path[0][1] = sv_ur_y;
    } else {
      Sound.buzz(); // clearly indicate that there is a missing possibility
      System.out.println("fisrt waypoint missing");
    }

    /*
     * For the second waypoint, there are 4 possibilities. Both LL corners are equal. Both UR
     * corners are equal. Difference between SV_LL and SH_UR is one. Difference between SV_UR and
     * SH_LL is one
     */
    if (sh_ll_x == sv_ll_x && sh_ll_y == sv_ll_y) {
      path[1][0] = sh_ll_x + 0.5;
      path[1][1] = sh_ll_y + 0.5;
    } else if (sh_ur_x == sv_ur_x && sh_ur_y == sv_ur_y) {
      path[1][0] = sh_ur_x - 0.5;
      path[1][1] = sh_ur_y - 0.5;
    } else if (Math.abs(sv_ll_x - sh_ur_x) == 1 && Math.abs(sv_ll_y - sh_ur_y) == 1) {
      path[1][0] = (sv_ll_x + sh_ur_x) / 2.0;
      path[1][1] = (sv_ll_y + sh_ur_y) / 2.0;
    } else if (Math.abs(sv_ur_x - sh_ll_x) == 1 && Math.abs(sv_ur_y - sh_ll_y) == 1) {
      path[1][0] = (sh_ll_x + sv_ur_x) / 2.0;
      path[1][1] = (sh_ll_y + sv_ur_y) / 2.0;
    } else {
      Sound.buzz(); // clearly indicate there is a missing possibility
      System.out.println("second waypoint missing");
    }

    /*
     * For the third waypoint, there are 4 possibilities. The crossing can end at the left, top,
     * right or bottom of green zone.
     */
    if (g_ur_x == sh_ll_x) {
      path[2][0] = sh_ll_x;
      path[2][1] = sh_ll_y + 0.5;
    } else if (g_ll_x == sh_ur_x) {
      path[2][0] = sh_ur_x;
      path[2][1] = sh_ur_y - 0.5;
    } else if (g_ll_y == sv_ur_y) {
      path[2][0] = sv_ur_x - 0.5;
      path[2][1] = sv_ur_y;
    } else if (g_ur_y == sv_ll_y) { // good
      path[2][0] = sv_ur_x - 0.5;
      path[2][1] = sv_ll_y;
    } else {
      Sound.buzz(); // clearly indicate that there is a missing possibility
      System.out.println("third waypoint missing");
    }

    return path;

  }

  /**
   * Performs a light localization before crossing the bridge. The method will compute the point
   * around which it should localize using the path provided.
   * 
   * @param path the path to follow whilst on the crossing. This array should be generated using
   *        {@link NavigationStrategy#getCrossingPath()}.
   */
  private void localizeBeforeCrossing(double[][] path) {

    /* Compute direction in which the robot will be going on the crossing's 1st leg */
    int dx = 0;
    int dy = 0;

    /* Compute dx */
    if (path[1][0] - path[0][0] != 0) {
      dx = (int) ((path[1][0] - path[0][0]) / Math.abs(path[1][0] - path[0][0]));
    }

    /* Compute dy */
    if (path[1][1] - path[0][1] != 0) {
      dy = (int) ((path[1][1] - path[0][1]) / Math.abs(path[1][1] - path[0][1]));
    }

    /* Travel to a point before the crossing */
    travelTo((path[0][0] - dx) * Robot.GRID_SIZE, (path[0][1] - dy) * Robot.GRID_SIZE, false);

    Robot.getLightLocalizer().localize(); // localize

  }

  /*
   * This method drives the robot forward until the last grid line of the crossing is detected.
   * Because the lines position is known (provided by server), it will also correct the odometer
   * accordingly.
   */
  private void clearCrossing(double[][] path) {

    /* Continue until black line is crossed, to make sure that the shallow crossing is cleared */
    while (LightLocalizer.getRedIntensity() > Robot.LINE_RED_INTENSITY) {
      Robot.getDrivingMotors()[0].forward();
      Robot.getDrivingMotors()[1].forward();
      try {
        Thread.sleep(25);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

    /* Stop motors once line is crossed */
    Robot.getDrivingMotors()[0].stop(true);
    Robot.getDrivingMotors()[1].stop();

    /*
     * Decide what the line tells us on the position of the robot (i.e. does it confirm a y or a x
     * coordinate
     */
    int dx = 0;
    int dy = 0;

    if (path[2][0] - path[1][0] != 0) {
      dx = (int) ((path[2][0] - path[1][0]) / Math.abs(path[2][0] - path[1][0]));
    }

    if (path[2][1] - path[1][1] != 0) {
      dy = (int) ((path[2][1] - path[1][1]) / Math.abs(path[2][1] - path[1][1]));
    }

    /* Correct odo */
    Robot.getOdo().setX((path[2][0] * 30.48) + (dx * LightLocalizer.LS_TO_CENTER));
    Robot.getOdo().setY((path[2][1] * 30.48) + (dy * LightLocalizer.LS_TO_CENTER));

  }
}
