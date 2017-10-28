package ca.mcgill.ecse211.team4.sensing;

import java.util.Map;

import ca.mcgill.ecse211.team4.robot.Robot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class searches and detects the objective flag.
 * @author Kevin Laframboise
 *
 */
public class FlagDetection {
	
	/**
	 * Sample provider for the color sensor. Sample Provider must be in color ID mode.
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
	 * x-coordinate of the upper right corner of the search zone.
	 */
	private int zone_UR_y;
	
	/**
	 * Motors driving the robot.
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	/**
	 * Creates a FlagDetection object with given parameter
	 * @param colorSampler
	 * @param colorData
	 * @param objectiveColorID
	 * @param gameParameters Map containing the game parameters. Used to determine the search zone.
	 */ 
	public FlagDetection(SampleProvider colorSampler, float[] colorData, int objectiveColorID, Map<String, Integer> gameParameters) {
		super();
		this.colorSampler = colorSampler;
		this.colorData = colorData;
		this.objectiveColorID = objectiveColorID;
		leftMotor = Robot.getDrivingMotors()[0];
		rightMotor = Robot.getDrivingMotors()[1];
		
		if(Robot.getTeamColor().equals("green")) {
			zone_LL_x = gameParameters.get("SR_LL_x").intValue();	
			zone_LL_y = gameParameters.get("SR_LL_y").intValue();	
			zone_UR_x = gameParameters.get("SR_UR_x").intValue();
			zone_UR_y = gameParameters.get("SR_UR_y").intValue();
		}
		else {
			zone_LL_x = gameParameters.get("SG_LL_x").intValue();	
			zone_LL_y = gameParameters.get("SG_LL_y").intValue();	
			zone_UR_x = gameParameters.get("SG_UR_x").intValue();
			zone_UR_y = gameParameters.get("SG_UR_y").intValue();
		}
	}

	/**
	 * Searches for the objective and detects flag by beeping 3 times.
	 */
	public void searchAndDetect() {
		
	}
}
