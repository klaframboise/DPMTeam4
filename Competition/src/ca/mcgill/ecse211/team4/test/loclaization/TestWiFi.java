package ca.mcgill.ecse211.team4.test.loclaization;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.Iterator;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.team4.robot.GameSetup;
import ca.mcgill.ecse211.team4.robot.Robot;

/**
 * Simple wifi test. Connect to Robot using the EV3WifiServer on MyCourses (don't forget to change SERVER_IP
 * in GameSetup.java). Pass some values through the server and verify in the console that the values match.
 * It also writes the received data to a text file on the EV3.
 * @author Kevin Laframboise
 *
 */
public class TestWiFi {
	
	public static void main(String[] args) {
		Robot robot = new Robot();
		Map<String, Long> params = null;
		
		/*The following code writes the received data to a file,
		 * which I am not sure is supported in LeJOS. Delete if causing
		 * problems.
		 */
		File file = new File("server_data.txt");
		PrintWriter out = null;
		if(file.exists()) file.delete();
		try {
			file.createNewFile();
			out = new PrintWriter(file);
		} catch (IOException e1) {
			e1.printStackTrace();
		}
		out.println("Wifi test " + new Date(System.currentTimeMillis()));
		out.println("Data received: ");
		/* End of code to comments if problems with File or IO. */
		
		/* Get server data */
		try {
			params = GameSetup.getGameParameters(true);
		} catch (IOException | ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		/* Iterate through server data */
		Iterator<String> it = params.keySet().iterator();
		String key;
		String line;
		while(it.hasNext()) {
			key = it.next();
			line = "Key: " + key + ", Value: " + params.get(key).intValue();
			System.out.println(line);
			out.println(line);	//also comment this if problems with IO
		}
	}

}
