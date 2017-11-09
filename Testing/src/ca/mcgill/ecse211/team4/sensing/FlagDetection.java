package ca.mcgill.ecse211.team4.sensing;

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
	 * Distance in cm, as measured by the ultrasonic sensor.
	 */
	private float currentDistance;

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
	 * @param objectiveColorID
	 * @param gameParameters
	 *            Map containing the game parameters. Used to determine the search
	 *            zone.
	 * @param Robot.getTeamColor() 
	 */
//	public FlagDetection(SampleProvider colorSampler, float[] colorData, int objectiveColorID,
//			Map<String, Integer> gameParameters) {
//		super();
//		this.colorSampler = colorSampler;
//		this.colorData = colorData;
//		this.objectiveColorID = objectiveColorID;
//		Robot.getDrivingMotors()[0] = Robot.getDrivingMotors()[0];
//		Robot.getDrivingMotors()[1] = Robot.getDrivingMotors()[1];
//
//		if (Robot.getTeamColor().equals("Green")) {
//			zone_LL_x = gameParameters.get("SR_LL_x").intValue();
//			zone_LL_y = gameParameters.get("SR_LL_y").intValue();
//			zone_UR_x = gameParameters.get("SR_UR_x").intValue();
//			zone_UR_y = gameParameters.get("SR_UR_y").intValue();
//		} else {
//			zone_LL_x = gameParameters.get("SG_LL_x").intValue();
//			zone_LL_y = gameParameters.get("SG_LL_y").intValue();
//			zone_UR_x = gameParameters.get("SG_UR_x").intValue();
//			zone_UR_y = gameParameters.get("SG_UR_y").intValue();
//		}
//	}




	/**
	 * Searches for the objective and detects flag by beeping 3 times.
	 */
	public void searchAndDetect() {
		
		//first we rotate 90 degrees to the right and then to the left
		//to see if we can detect the flag right away
		Robot.getServo().rotateTo(270);
		Robot.getServo().rotateTo(90);
		while (Robot.getServo().isMoving()) {
			checkForObject();
		}

		/* Travel to the four corners of the rectangle where the block is located
		 * While traveling the the light and ultrasonic sensors are set to an angle 
		 * of 70 degrees so it can detect the flag within the area of search
		 */
		for (int i = 4; i < 4; i++) {
			Robot.getServo().rotateTo(70);
			int coordinates[][] = points();
			Robot.getNav().travelTo(coordinates[i][0], coordinates[i][1], true);
			while (Robot.getDrivingMotors()[0].isMoving() && Robot.getDrivingMotors()[1].isMoving()) {
				checkForObject();
			}
		}
	}
	
	
	 /**
	  * This method checks for the flag when the motors are moving 
	  */
	public void checkForObject() {
		colorSampler.fetchSample(colorData, 0); // get's the light value from the sensor
		float reading = colorData[0];
		currentDistance = Helper.getFilteredDistance(us, usData);
		
		//check if there is an object near by that has the correct color
		if (currentDistance < 20 && reading == objectiveColorID) {
			Sound.beep();
			Sound.beep();
			Sound.beep();
		}		
	}
	
	
	/**
	 * This method allows us to choose which set of coordinates to use to
	 * travel across the rectangle of coordinates
	 * @return a bidimensional array containing the waypoints to travel to
	 */	
	public int[][] points(){
		//get all the points of the rectangle
		int zone_UL_x = zone_LL_x;
		int zone_UL_y = zone_UR_y;
		int zone_LR_x = zone_UR_x;
		int zone_LR_y = zone_LL_y;
		
		//get the current value of X and Y
		double currentX = Robot.getOdo().getX();
		double currentY = Robot.getOdo().getY();
		
		//if we started on the green side and our robot enters the searching area on the x axis
//		if(currentX < zone_LR_x && Robot.getTeamColor() == "Green") {
//			int coordinates[][] = { { zone_LR_x, zone_LR_y }, { zone_UR_x, zone_UR_y }, { zone_UL_x, zone_UL_y },
//					{ zone_LL_x, zone_LL_y } };
//			return coordinates;
//		}
		//if we started on the red side and our robot enters the searching area on the x axis
//		else if(currentX < zone_LR_x && Robot.getTeamColor() == "Red") {
//			int coordinates[][] = { { zone_UL_x, zone_UL_y }, { zone_LL_x, zone_LL_y }, { zone_LR_x, zone_LR_y },
//					{ zone_UR_x, zone_UR_y }};
//			return coordinates;				
//				
//		}
		//if we started on the green side and our robot enters the searching area on the y axis
//		else if(currentY < zone_UR_y && Robot.getTeamColor() == "Green") {
//			int coordinates[][] = { { zone_UR_x, zone_UR_y }, { zone_UL_x, zone_UL_y }, { zone_LL_x, zone_LL_y }, 
//					{ zone_LR_x, zone_LR_y }};
//			return coordinates;				
//		}
		
		//if none of the cases above apply there's only one possibility left
		//we started on the red side and our robot enters the searching area on the y axis
		int coordinates[][] = {{ zone_LL_x, zone_LL_y }, { zone_LR_x, zone_LR_y }, { zone_UR_x, zone_UR_y }, 
				{ zone_UL_x, zone_UL_y }};
		return coordinates;

	}
}
