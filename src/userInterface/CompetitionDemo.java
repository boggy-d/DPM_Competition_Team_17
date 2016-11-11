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
import lejos.hardware.Button;

public class CompetitionDemo {

	public static void main(String[] args) {
		//TODO Print to the LCD. Set up ActionController thread
		
		isEnter();
		ActionController ac = new ActionController();
		while(Button.waitForAnyPress() != Button.ID_LEFT);
	}
	
	
	/**
	 * Loops until either the Enter or Escape button is pressed
	 * on the EV3 Brick. Pressing the Escape button will cause
	 * the program to terminate
	 */
	public static void isEnter()
	{
		while(Button.waitForAnyPress() != Button.ID_ENTER || Button.waitForAnyPress() != Button.ID_ESCAPE);
		if(Button.waitForAnyPress() != Button.ID_ESCAPE)
		{
			System.exit(0);
		}
	}
}
