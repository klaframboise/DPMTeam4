package ca.mcgill.ecse211.team4.test.sensing;

import ca.mcgill.ecse211.team4.robot.Display;
import ca.mcgill.ecse211.team4.robot.Robot;
import ca.mcgill.ecse211.team4.sensing.FlagDetection;
import lejos.hardware.Button;

/*
 * A test clas for flag detection
 */
public class TestFlagDetection {

	public static void main(String[] args) {
	
		Robot robot = new Robot();
		
		Robot.getOdo().start();
		Robot.getNav().start();
		Display display = new Display(Robot.getOdo());
		display.start();
		
		int LL_x;
		int LL_y;
		if (Robot.getTeamColor().equals("Green")) {
			LL_x = Robot.getGameParameters().get("SR_LL_x").intValue();
			LL_y = Robot.getGameParameters().get("SR_LL_y").intValue();
		} else {
			LL_x = Robot.getGameParameters().get("SG_LL_x").intValue();
			LL_y = Robot.getGameParameters().get("SG_LL_y").intValue();
		}
		Robot.getServo().rotateTo(0, false);

		Robot.getOdo().setX(LL_x);
		Robot.getOdo().setY(LL_y);
		Robot.getOdo().setTheta(0);
//		System.out.println("Travelling to corner of search zone");
//		Robot.getNav().travelTo((LL_x - 0.1) * Robot.GRID_SIZE, (LL_y - 0.1) * Robot.GRID_SIZE, false);
		System.out.println("Searching and detecting");
		Robot.getFlagDetection().searchAndDetect();
		Robot.getNav().travelTo(5 * Robot.GRID_SIZE, 3 * Robot.GRID_SIZE, false);

	}

}
