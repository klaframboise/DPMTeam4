package ca.mcgill.ecse211.team4.robot;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;

/**
 * This class will be used to setup the game parameters. It will be provided later.
 */
public class GameSetup {
	
	/**
	 * IP address of the server from which game parameters are received
	 */
	private static final String SERVER_IP = "192.168.2.3";
	
	/**
	 * Team number
	 */
	private static final int TEAM_NUMBER = 4;
	
	/**
	 * @param debugPrint dictates whether debug information is printed to default output.
	 * @return Map containing game parameters
	 * @throws UnknownHostException
	 * @throws IOException
	 * @throws ParseException
	 */
	public static Map<String, Integer> getGameParameters(boolean debugPrint) throws UnknownHostException, IOException, ParseException {
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, debugPrint);
		@SuppressWarnings("unchecked")
		Map<String, Integer> data = conn.getData(); 
		return data;
	}

}
