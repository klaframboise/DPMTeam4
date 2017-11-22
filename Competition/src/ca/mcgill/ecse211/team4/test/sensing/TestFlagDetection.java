package ca.mcgill.ecse211.team4.test.sensing;

import ca.mcgill.ecse211.team4.robot.Display;
import ca.mcgill.ecse211.team4.robot.Robot;
import ca.mcgill.ecse211.team4.sensing.FlagDetection;
import lejos.hardware.Button;

public class TestFlagDetection {

	public static void main(String[] args) {
	
		Robot robot = new Robot();
		
		Robot.getOdo().start();
		Robot.getNav().start();
		Robot.getOdo().setX(30.48);
		Robot.getOdo().setY(30.48);
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
//		System.out.println("Travelling to corner of search zone");
//		Robot.getNav().travelTo((LL_x - 0.1) * Robot.GRID_SIZE, (LL_y - 0.1) * Robot.GRID_SIZE, false);
		System.out.println("Searching and detecting");
		Robot.getFlagDetection().searchAndDetect();
		Robot.getNav().travelTo(5 * Robot.GRID_SIZE, 3 * Robot.GRID_SIZE, false);

	}

}
