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
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;

public class CompetitionDemo {
	
	
	

	public static void main(String[] args) {
		//TODO Print to the LCD. Set up ActionController thread
		
		//isEnter();
		ActionController ac = new ActionController(Constants.DEFAULT_TIMEOUT_PERIOD, true);
		
		
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
