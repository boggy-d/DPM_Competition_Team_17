/**
 * The CompetitionDemo class contains the main method of the 
 * robot's program. It prints the display onto the robot's
 * LCD screen and checks if the Enter button has been pressed
 * before giving control to the ActionController class
 * 
 *  @author Bogdan Dumitru
 *  @version 0.1.0
 */


package userInterface;

import component.ActionController;
import component.Constants;
import lejos.hardware.Button;

public class CompetitionDemo {

	public static void main(String[] args) {

		while (Button.waitForAnyPress() != Button.ID_ENTER);
		
		ActionController ac = new ActionController(Constants.DEFAULT_TIMEOUT_PERIOD, true);
		
		while(true)
		{
			ac.doRoutine();
		}
		
		//final boolean TIME_TO_GO_TO_SLEEP = true; //zzz	
	}
	
	
	/**
	 * Loops until either the Enter or Escape button is pressed
	 * on the EV3 Brick. Pressing the Escape button will cause
	 * the program to terminate
	 */
	public static void isEnter(int buttonChoice)
	{
		while(buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_ESCAPE);
		
		if(buttonChoice == Button.ID_ESCAPE)
		{
			System.exit(0);
		}
		else{
			return;
		}
	
		
		//while(Button.waitForAnyPress() != Button.ID_ENTER && Button.waitForAnyPress() != Button.ID_ESCAPE);
	}
}
