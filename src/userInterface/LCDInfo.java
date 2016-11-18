/**
 * The LCDInfo class gets the position and angle from the Odometer
 * class , the block status (block, obstacle, or none) from the 
 * LightPoller class, and the distance from the USPoller class and
 * prints them on the LCD screen of the EV3
 * 
 * @author Bogdan Dumitru
 * @version 0.1.0
 */

package userInterface;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import component.ActionController;
import component.Odometer;



public class LCDInfo implements TimerListener{
	
	public static final int LCD_REFRESH = 100;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();;

	// arrays for displaying data
	private double [] pos;
	private double dist;
	private float[] usData;
	private SampleProvider usSensor;
	
	public LCDInfo() {
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		
		// initialise the arrays for displaying data
		pos = new double [3];
		
		// start the timer
		lcdTimer.start();
	}
	
	/**
	 * Prints the position and heading of the robot, the
	 * block status and the distance to an object
	 * 
	 * @see Odometer
	 * @see LightPoller
	 * @see USPoller
	 */
	public void printDisplay()
	{
		//TODO Draw String
		
	}

	public void timedOut() { 
		ActionController.odometer.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString("D: ", 0, 3);
		LCD.drawInt((int)(pos[0] * 10), 3, 0);
		LCD.drawInt((int)(pos[1] * 10), 3, 1);
		LCD.drawInt((int)pos[2], 3, 2);
		LCD.drawInt((int)ActionController.usPoller.getClippedData(255), 3, 3);
	}
	
}
