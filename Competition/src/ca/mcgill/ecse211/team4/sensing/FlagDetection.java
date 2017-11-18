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
	 * Color mapping from sensor return values (index) to server color values (element).
	 */
	private static final int[] colorMapping = {0,1,2,3,0,0,4,0};
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
	 * @param us
	 * @param usData
	 * @param objectiveColorID
	 * @param gameParameters
	 *            Map containing the game parameters. Used to determine the search
	 *            zone.
	 * @param Robot.getTeamColor() 
	 */
	public FlagDetection(SampleProvider colorSampler, float[] colorData, SampleProvider us, float[] usData, int objectiveColorID,
			Map<String, Long> gameParameters) {
		super();
		this.colorSampler = colorSampler;
		this.colorData = colorData;
		this.objectiveColorID = objectiveColorID;
		this.us = us;
		this.usData = usData;
		Robot.getDrivingMotors()[0] = Robot.getDrivingMotors()[0];
		Robot.getDrivingMotors()[1] = Robot.getDrivingMotors()[1];

		if (Robot.getTeamColor().equals("Green")) {
			zone_LL_x = gameParameters.get("SR_LL_x").intValue();
			zone_LL_y = gameParameters.get("SR_LL_y").intValue();
			zone_UR_x = gameParameters.get("SR_UR_x").intValue();
			zone_UR_y = gameParameters.get("SR_UR_y").intValue();
		} else {
			zone_LL_x = gameParameters.get("SG_LL_x").intValue();
			zone_LL_y = gameParameters.get("SG_LL_y").intValue();
			zone_UR_x = gameParameters.get("SG_UR_x").intValue();
			zone_UR_y = gameParameters.get("SG_UR_y").intValue();
		}
		
		Robot.getServo().setSpeed(45);
	}




	/**
	 * Searches for the objective and detects flag by beeping 3 times.
	 */
	public void searchAndDetect() {
		
		/* Initializing local variables */
		int currentDistance;
		int coordinates[][] = points();
		
		System.out.println("Corner coords: \n"
				+ coordinates[0][0] + ", " + coordinates[0][1] + "\n"
				+ coordinates[1][0] + ", " + coordinates[1][1] + "\n"
				+ coordinates[2][0] + ", " + coordinates[2][1] + "\n"
				+ coordinates[3][0] + ", " + coordinates[3][1]);		//debug
		
		
		Robot.getServo().rotateTo(45, false); //set servo speed
		
		/* Travel to the four corners of the rectangle where the block is located of the search zone 
		 * while checking ultrasonic for close by object
		 */
		for (int i = 0; i < 4; i++) {
			/* Start navigating around search zone and return immediately */
			Robot.getNav().travelTo(coordinates[i][0] * Robot.GRID_SIZE, coordinates[i][1] * Robot.GRID_SIZE, true);
			System.out.println("Travelling to " + coordinates[i][0] + ", " + coordinates[i][1]);
			
			/* Poll ultrasonic sensor */
			currentDistance = Helper.getFilteredDistance(us, usData);
			while (currentDistance > 30 && Robot.getDrivingMotors()[0].isMoving()) {
				currentDistance = Helper.getFilteredDistance(us, usData);
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
			/* Check if loop broke because object was detected */
			if(currentDistance < 30) {
				/* Stop motors */
				Robot.getDrivingMotors()[0].stop(true);
				Robot.getDrivingMotors()[1].stop(true);
				Robot.getServo().rotateTo(0);
				
				locateFlag();
			}
		}

	}
	
	/**
	 * This method will locate the flag by sweeping the ultrasonic sensor. The angle at which
	 * the measured distance is smallest is the heading towards which the object is. It will then
	 * drive the robot twoards the flag and poll the color sensor to see if the color of the falg
	 * is the one we are looking for.
	 * @return	true if our flag is located
	 */
	private boolean locateFlag() {
		
		/* Initialize local variable */
		double relHeadingToObject = getHeadingToObject();
		int colorReading = getColorId(colorSampler, colorData);
		System.out.println("Heading to object: " + relHeadingToObject);	//debug
		
		/* Turn towards the detected object */
		Robot.getNav().turn(relHeadingToObject, "left");
		Robot.getServo().rotateTo(0, false);
		
		/* Set the motor speed very slow so as to not push the blocks out of the box */
		Robot.getDrivingMotors()[0].setSpeed(60);
		Robot.getDrivingMotors()[1].setSpeed(60);
		
		/* Drive robot forward until one of the four colors are detected.
		 * This will happen when the color sensor is very close to the flag
		 */
		while(true) {
			Robot.getDrivingMotors()[0].forward();
			Robot.getDrivingMotors()[1].forward();
			colorReading = getColorId(colorSampler, colorData);
			if(colorReading >= 0 && colorReading <= 7) {
				if(colorMapping[colorReading] != 0) {
					Robot.getDrivingMotors()[0].stop(true);
					Robot.getDrivingMotors()[1].stop(true);
					break;
				}
			}
		}
		
		/* Testing if blue is detected */
		if(colorMapping[colorReading] == 2) {
			Sound.beep();
			Sound.beep();
			Sound.beep();
			
		}
		
		return true;
		
	}
	
	/**
	 * This method returns, in degrees, the angle, as reported by the getPosition() method of the servo motor
	 * at which the smallest distance was detected. This angle should be the heading required to go towards the object.
	 * @return heading to the object, relative to the current heading of the robot, in degrees.
	 */
	private double getHeadingToObject() {
		
		Map<Float, Integer> sweepResults = new HashMap<Float, Integer>();
		Iterator<Float> it;
		int minDistance = Integer.MAX_VALUE;
		int tmpDistance;
		float angleToMinDistance = 0;
		Float tmpAngle;
	
		Robot.getServo().rotateTo(0, false);
		Robot.getServo().setSpeed(10); //set a slow speed for distance reading to occur
		
		/* Get data */
		for(int i = 0; i < 7; i++) {
			sweepResults.put(Robot.getServo().getPosition(), Helper.getFilteredDistance(us, usData));
			Robot.getServo().rotate(10);
		}
		
		
		/* Find minimum distance */
		it = sweepResults.keySet().iterator();
		while(it.hasNext()) {
			tmpAngle = it.next();
			tmpDistance = sweepResults.get(tmpAngle).intValue();
			System.out.println("Distance at: " + tmpAngle.floatValue() + " is " + tmpDistance);
			if(tmpDistance < minDistance) {
				minDistance = tmpDistance;
				angleToMinDistance = tmpAngle.floatValue();
			}
		}
		
		Robot.getServo().setSpeed(45);
		Robot.getServo().rotateTo(0);
		
		return angleToMinDistance;
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
		if(currentX < zone_LR_x && Robot.getTeamColor() == "Green") {
			int coordinates[][] = { { zone_LR_x, zone_LR_y }, { zone_UR_x, zone_UR_y }, { zone_UL_x, zone_UL_y },
					{ zone_LL_x, zone_LL_y } };
			return coordinates;
		}
		//if we started on the red side and our robot enters the searching area on the x axis
		else if(currentX < zone_LR_x && Robot.getTeamColor() == "Red") {
			int coordinates[][] = { { zone_UL_x, zone_UL_y }, { zone_LL_x, zone_LL_y }, { zone_LR_x, zone_LR_y },
					{ zone_UR_x, zone_UR_y }};
			return coordinates;				
				
		}
		//if we started on the green side and our robot enters the searching area on the y axis
		else if(currentY < zone_UR_y && Robot.getTeamColor() == "Green") {
			int coordinates[][] = { { zone_UR_x, zone_UR_y }, { zone_UL_x, zone_UL_y }, { zone_LL_x, zone_LL_y }, 
					{ zone_LR_x, zone_LR_y }};
			return coordinates;				
		}
		
		//if none of the cases above apply there's only one possibility left
		//we started on the red side and our robot enters the searching area on the y axis
		int coordinates[][] = {{ zone_LL_x, zone_LL_y }, { zone_LR_x, zone_LR_y }, { zone_UR_x, zone_UR_y }, 
				{ zone_UL_x, zone_UL_y }};
		return coordinates;

	}
	
	public static int getColorId(SampleProvider colorSampler, float[] colorData) {

		for(int i = 0; i < colorData.length; i+= colorSampler.sampleSize()) {
			colorSampler.fetchSample(colorData, i * colorSampler.sampleSize()); // acquire data
		}
		Arrays.sort(colorData);	// sort array
		return (int)colorData[colorData.length/2]; // return median
	}
}
