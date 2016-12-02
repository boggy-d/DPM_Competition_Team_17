/**
 * The CompetitionDemo class contains the main method of the 
 * robot's program. It prints the display onto the robot's
 * LCD screen and checks if the Enter button has been pressed
 * before giving control to the ActionController class
 * 
 *  @author Bogdan Dumitru
 */


package userInterface;

import component.ActionController;
import component.Constants;
import lejos.hardware.Button;

public class PLZDONTBREAK {

	/**
	 * Main Class of the program. Waits for the Enter button to be 
	 * pressed and then loops the routine
	 * @param args
	 */
	public static void main(String[] args) {
		isEnter();
		ActionController ac = new ActionController();
		
		while(true)
		{
			ac.doRoutine();
		}
		
	}
	
	
	/**
	 * Loops until either the Enter or Escape button is pressed
	 * on the EV3 Brick. Pressing the Escape button will cause
	 * the program to terminate
	 */
	public static void isEnter()
	{
		int buttonChoice = Button.waitForAnyPress();
		
		if(buttonChoice == Button.ID_ESCAPE)
		{
			System.exit(0);
		}
		else{
			return;
		}
	}
}
