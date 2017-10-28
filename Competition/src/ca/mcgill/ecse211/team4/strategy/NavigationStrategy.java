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
	 * Must be "red" or "green". This will change the strategy used.
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
	public void NavigateToObjectiveZone() {
		
		if(teamColor.equals("green")) NavToObjectiveByZip();
		else NavToObjectiveByCrossing();
	}
	
	/**
	 * Uses the navigation strategy dictated by team color to navigate back to the starting point.
	 */
	public void NavigateBack() {
		
		if(teamColor.equals("green")) NavBackByCrossing();
		else NavBackByZip();
	}
	
	/**
	 * Navigates to the objective using the zipline.
	 */
	private void NavToObjectiveByZip() {
		
	}
	
	/**
	 * Navigates to the objective using the shallow crossing.
	 */
	private void NavToObjectiveByCrossing() {
		
	}
	
	/**
	 * Navigates back to the start using the zipline.
	 */
	private void NavBackByZip() {
		
	}
	
	/**
	 * Navigates back to the start using the shallow crossing.
	 */
	private void NavBackByCrossing() {
		
	}
	
}
