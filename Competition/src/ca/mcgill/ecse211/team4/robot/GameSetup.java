package ca.mcgill.ecse211.team4.robot;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * Gets a Map object containing game parameters from the game server. <br>
 * Key legend: <br>
 * RedTeam (i=1,20) –Team starting out from red zone <br>
 * GreenTeam (i=1,20) –Team starting out from green zone <br>
 * RedCorner (i=1,4) –Starting corner for red team <br>
 * GreenCorner (i=1,4) –Starting corner for green team <br>
 * OG (i=1,5) –color of green opponent flag <br>
 * OR (i=1,5) –color of red opponent flag <br>
 * Red_LL (x,y) –lower left hand corner of Red Zone <br>
 * Red_UR (x,y) –upper right hand corner of Red Zone <br>
 * Green_LL (x,y) –lower left hand corner of Green Zone <br>
 * Green_UR (x,y) –upper right hand corner of Green Zone <br>
 * ZC_R (x,y) –end point corresponding to zip line in Red Zone <br>
 * ZO_R (x,y) –end point together with ZC_R indicates direction of zip line <br>
 * ZC_G (x,y) –end point corresponding to zip line in Green Zone <br>
 * ZO_G (x,y) –end point together with ZC_G indicates direction of zip line <br>
 * SH_LL (x,y) –lower left hand corner of horizontal shallow water zone <br>
 * SH_UR (x,y) –upper right hand corner of horizontal shallow water zone <br>
 * SV_LL (x,y) –lower left hand corner of vertical shallow water zone <br>
 * SV_UR (x,y) –upper right hand corner of vertical shallow water zone <br>
 * SR_LL (x,y) –lower left hand corner of search region in red player zone <br>
 * SR_UR (x,y) –upper right hand corner of search region in red player zone <br>
 * SG_LL (x,y) –lower left hand corner of search region in green player zone <br>
 * SG_UR (x,y) –upper right hand corner of search region in green player zone
 */
public class GameSetup {
	
	/**
	 * IP address of the server from which game parameters are received
	 */
	private static final String SERVER_IP = "192.168.137.1";
	
	/**
	 * @param debugPrint dictates whether debug information is printed to default output.
	 * @return Map containing game parameters
	 * @throws UnknownHostException
	 * @throws IOException
	 * @throws ParseException
	 */
	public static Map<String, Long> getGameParameters(boolean debugPrint) throws UnknownHostException, IOException, ParseException {
		WifiConnection conn = new WifiConnection(SERVER_IP, Robot.TEAM_NUMBER, debugPrint);
		@SuppressWarnings("unchecked")
		Map<String, Long> data = conn.getData(); 
		return data;
	}

}
