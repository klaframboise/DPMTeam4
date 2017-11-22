/**
 * 
 */
package ca.mcgill.ecse211.team4.test.strategy;

//import static org.junit.Assert.*;
//import org.junit.Before;
//import org.junit.Test;
import ca.mcgill.ecse211.team4.drivers.Navigation;
import ca.mcgill.ecse211.team4.robot.Robot;
import ca.mcgill.ecse211.team4.strategy.NavigationStrategy;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author lafra
 *
 */
public class UnitTestNavigationStrategy {

  int[][] inputs = {{0,0,1,0},{0,0,0,1},{0,0,1,1},{0,0,1,-1},{0,0,-1,1},{0,0,-1,-1},{0,0,2,2},{0,0,3,-3}};
  int[][] expectedResults = {{5,0,6,0},{0,5,0,6},{4,4,5,5},{4,-4,5,-5},{-4,4,-5,5},{-4,-4,-5,-5},{5,5,6,6},{6,-6,7,-7}};

  //@Test
  public void test() {
    int results[][] = new int[inputs.length][inputs[0].length];
    
    /* Get results */
    for(int i = 0; i < inputs.length; i++) {
      results[i] = NavigationStrategy.getZipLineEndpoints(inputs[i][0], inputs[i][1], inputs[i][2], inputs[i][3]);
    }
    
    /* Compare results */
    for(int i = 0; i < results.length; i++) {
      //assertArrayEquals(expectedResults[i], results[i]);
    }
  }

}
