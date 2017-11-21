package ca.mcgill.ecse211.team4.strategy;

import java.util.Map;
import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.drivers.ZipLineTraversal;
import ca.mcgill.ecse211.team4.robot.Helper;
import ca.mcgill.ecse211.team4.robot.Robot;

/**
 * This class implements a strategy to get to the objective and back to starting point.
 * 
 * @author Kevin Laframboise
 *
 */
public class NavigationStrategy {

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
    travelTo((Z0_x - 0.1) * Robot.GRID_SIZE, (Z0_y - 0.1) * Robot.GRID_SIZE);
    Robot.getLightLocalizer().localize();
    travelTo(Robot.getLightLocalizer().getGridX(), Robot.getLightLocalizer().getGridY());

    /* Traverse zip line */
    traversal.traverse();

    /* Travel to search area */
    int closestCorner = findClosestCorner(objCoords); // find the closest corner of the search area
    travelTo(objCoords[closestCorner][0] * Robot.GRID_SIZE,
        objCoords[closestCorner][1] * Robot.GRID_SIZE); // travel to closest corner

  }


  /**
   * Navigates to the objective using the shallow crossing.
   */
  private void navToObjectiveByCrossing() {

    /* Determine which direction to travel in on the shallow crossing first */
    char firstDirection = findFirstDirection();
    char secondDirection = (firstDirection == 'H') ? 'V' : 'H';

    /* Initialize game parameters variables */
    int SC1_x = gameParameters.get("S" + firstDirection + "_LL_x").intValue();
    int SC1_y = gameParameters.get("S" + firstDirection + "_LL_y").intValue();
    int SC2_x = gameParameters.get("S" + firstDirection + "_UR_x").intValue();
    int SC2_y = gameParameters.get("S" + firstDirection + "_UR_y").intValue();
    int SC3_x = gameParameters.get("S" + secondDirection + "_LL_x").intValue();
    int SC3_y = gameParameters.get("S" + secondDirection + "_LL_y").intValue();
    int[][] objCoords =
        Helper.convertToFourCorners(gameParameters.get("S" + otherTeam + "_UR_x").intValue(),
            gameParameters.get("S" + otherTeam + "_UR_y").intValue(),
            gameParameters.get("S" + otherTeam + "_LL_x").intValue(),
            gameParameters.get("S" + otherTeam + "_LL_y").intValue()); // convert the objective
                                                                       // search region to a set of
                                                                       // 4 points

    /* Travel to start of crossing */
    travelTo((SC1_x + 0.5) * Robot.GRID_SIZE, SC1_y * Robot.GRID_SIZE);

    /* Travel to turn in crossing */
    travelTo((SC2_x - 0.5) * Robot.GRID_SIZE, (SC2_y - 0.5) * Robot.GRID_SIZE);

    /* Travel to end of crossing */
    travelTo(SC3_x * Robot.GRID_SIZE, (SC3_y + 0.5) * Robot.GRID_SIZE);

    /* Travel to search area */
    int closestCorner = findClosestCorner(objCoords); // find the closest corner of the search area
    travelTo(objCoords[closestCorner][0] * Robot.GRID_SIZE,
        objCoords[closestCorner][1] * Robot.GRID_SIZE); // travel to closest corner
    
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
    int startX = findStart()[0]; // x coord of our team's starting pos
    int startY = findStart()[1]; // y coord of our team's starting pos

    /* Initialize zip line driver */
    ZipLineTraversal traversal = new ZipLineTraversal(Robot.getLineMotor(),
        Robot.getDrivingMotors()[0], Robot.getDrivingMotors()[1], ZC_x * Robot.GRID_SIZE,
        ZC_y * Robot.GRID_SIZE, ZF0_x * Robot.GRID_SIZE, ZF0_y * Robot.GRID_SIZE);
    
    /* Travel to start of zip line */
    travelTo((Z0_x - 0.1) * Robot.GRID_SIZE, (Z0_y - 0.1) * Robot.GRID_SIZE);
    Robot.getLightLocalizer().localize();
    travelTo(Robot.getLightLocalizer().getGridX(), Robot.getLightLocalizer().getGridY());

    /* Traverse zip line */
    traversal.traverse();

    /* Travel to start */
    travelTo(startX * Robot.GRID_SIZE, startY * Robot.GRID_SIZE);

  }

  /**
   * Navigates back to the start using the shallow crossing.
   */
  private void navBackByCrossing() {

    /* Determine which direction to travel in on the shallow crossing first */
    char firstDirection = findFirstDirection();
    char secondDirection = (firstDirection == 'H') ? 'V' : 'H';

    /* Initialize game parameters variables */
    int SC1_x = gameParameters.get("S" + secondDirection + "_LL_x").intValue();
    int SC1_y = gameParameters.get("S" + secondDirection + "_LL_y").intValue();
    int SC2_x = gameParameters.get("S" + secondDirection + "_UR_x").intValue();
    int SC2_y = gameParameters.get("S" + secondDirection + "_UR_y").intValue();
    int SC3_x = gameParameters.get("S" + firstDirection + "_LL_x").intValue();
    int SC3_y = gameParameters.get("S" + firstDirection + "_LL_y").intValue();
    int startX = findStart()[0];
    int startY = findStart()[1];

    /* Travel to start of crossing */
    travelTo((SC1_x + 0.5) * Robot.GRID_SIZE, SC1_y * Robot.GRID_SIZE);

    /* Travel to turn in crossing */
    travelTo((SC2_x - 0.5) * Robot.GRID_SIZE, (SC2_y - 0.5) * Robot.GRID_SIZE);

    /* Travel to end of crossing */
    travelTo(SC3_x * Robot.GRID_SIZE, (SC3_y + 0.5) * Robot.GRID_SIZE);

    /* Travel to start area */
    travelTo(startX * Robot.GRID_SIZE, startY * Robot.GRID_SIZE);
  }

  /**
   * This method divides the given path in shorter paths where the robot will localize after
   * reaching the end of each shorter path. This method calls itself recursively until the original
   * x and y are reached.
   * 
   * @param x destination-x, in cm
   * @param y destination-y, in cm
   */
  private void travelTo(double x, double y) {

    /* Initialize local variables */
    final double LOCALIZATION_FREQ = 5 * Robot.GRID_SIZE; // localize every LOCALIZATION_FREQ cm in
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
    if (Math.abs(currentX - x) > LOCALIZATION_FREQ) {
      waypointX = currentX + (LOCALIZATION_FREQ * posX);
      localizationRequired = true;
    }

    if (Math.abs(currentY - y) > LOCALIZATION_FREQ) {
      waypointY = currentY + (LOCALIZATION_FREQ * posY);
      localizationRequired = true;
    }

    /* Travel to waypoint and wait for nav to end */
    nav.travelTo(waypointX, waypointY, false);

    /* Localize if required and continue navigating to original waypoint */
    if (localizationRequired) {
      Robot.getLightLocalizer().localize();
      travelTo(x, y);
    }

  }

  /**
   * This method returns, in an array of integers, the waypoints at the end of the zipline. The array is structured as follows:
   * <pre>
   * Index   Value
   * 0       x of zip line end point
   * 1       y of zip line end point
   * 2       x of "other" end point
   * 3       y of "other" end point
   * </pre>
   * @param z0_x x of "other" start point.
   * @param z0_y y of "other" start point.
   * @param zC_x x of zip line start point.
   * @param zC_y y of zip line start point.
   * @return int array containing zip line endpoints
   */
  public static int[] getZipLineEndpoints(int z0_x, int z0_y, int zC_x, int zC_y) {
    
    int[] result = new int[4]; // initialize result array
    
    /* Get direction of the zip line by comparing start point and "other" start point */
    int dx = ((zC_x - z0_x) != 0)? (zC_x - z0_x)/Math.abs(zC_x - z0_x) : zC_x - z0_x; // difference in x between start and "other" start
    int dy = ((zC_y - z0_y) != 0)? (zC_y - z0_y)/Math.abs(zC_y - z0_y) : zC_y - z0_y; // difference in y between start and "other" start
    
    /* Case where zip line is along x or y-axis*/
    if(dx == 0 || dy == 0) {
      result[0] = zC_x + (dx * 4);
      result[1] = zC_y + (dy * 4);
    }
    
    /* Case where zip line is diagonal */
    else {
      result[0] = zC_x + (dx * 3);
      result[1] = zC_y + (dy * 3);
    }
    
    /* Compute other endpoints, one away in the direction of the zipline */
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

      distance = Math.sqrt(Math.pow((currentX - rectArea[i][0] * Robot.GRID_SIZE), 2)
          + Math.pow(currentY, rectArea[i][1] * Robot.GRID_SIZE)); // calculate distance to corner

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
  private int[] findStart() {

    int[] start = new int[2];

    switch (gameParameters.get(teamColor + "Corner").intValue()) {
      case 1:
        start[0] = 1;
        start[1] = 1;
        break;
      case 2:
        start[0] = Robot.MAX_COORD;
        start[1] = 1;
        break;
      case 3:
        start[0] = Robot.MAX_COORD;
        start[1] = Robot.MAX_COORD;
        break;
      case 4:
        start[0] = 1;
        start[1] = Robot.MAX_COORD;
        break;
    }

    return start;
  }

  /**
   * Finds the orientation of the shallow crossing connected to the robot's starting zone.
   * 
   * @return 'H' is the starting zone is connected to the horizontal portion of the crossing. 'V' is
   *         the starting zone is connected to the vertical portion of the crossing.
   */
  private char findFirstDirection() {

    /*
     * If the vertical portion is connected to our team's zone, then it is the first direction in
     * which to travel
     */
    if (gameParameters.get("SV_LL_y").intValue() == gameParameters.get(teamColor + "_UR_y")) {
      return 'V';
    } else {
      return 'H';
    }
  }
}
