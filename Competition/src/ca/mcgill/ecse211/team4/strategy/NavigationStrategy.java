package ca.mcgill.ecse211.team4.strategy;

import java.util.Map;

import ca.mcgill.ecse211.team4.drivers.Navigation;

/**
 * This class implements a strategy to get to the objective and back to starting point.
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
	private Map<String, Integer> gameParameters;
	
	/**
	 * Navigation object used by strategy
	 */
	private Navigation nav;

	/**
	 * Creates a NavigationStrategy object with the given parameters.
	 * @param teamColor "red" or "green"
	 * @param gameParameters
	 * @param nav
	 */
	public NavigationStrategy(String teamColor, Map<String, Integer> gameParameters, Navigation nav) {
		this.teamColor = teamColor;
		this.gameParameters = gameParameters;
		this.nav = nav;
	}
	
	/**
	 * Uses the navigation strategy dictated by team color to navigate to the objective.
	 */
	public void navigateToObjectiveZone() {
		
		if(teamColor.equals("Green")) navToObjectiveByZip();
		else navToObjectiveByCrossing();
	}
	
	/**
	 * Uses the navigation strategy dictated by team color to navigate back to the starting point.
	 */
	public void navigateBack() {
		
		if(teamColor.equals("Green")) navBackByCrossing();
		else navBackByZip();
	}
	
	/**
	 * Navigates to the objective using the zipline.
	 */
	private void navToObjectiveByZip() {
		
	}
	
	/**
	 * Navigates to the objective using the shallow crossing.
	 */
	private void navToObjectiveByCrossing() {
		
	}
	
	/**
	 * Navigates back to the start using the zipline.
	 */
	private void navBackByZip() {
		
	}
	
	/**
	 * Navigates back to the start using the shallow crossing.
	 */
	private void navBackByCrossing() {
		
	}
	
}
