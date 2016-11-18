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
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class CompetitionDemo {
	// get motors
//	private final static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
//	private final static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
//	private final static EV3LargeRegulatedMotor clawLift = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
//	private final static EV3LargeRegulatedMotor clawClose = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
//	
//	// get sensors
//	private static final EV3UltrasonicSensor frontUsSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
//	private static final EV3UltrasonicSensor sideUsSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
//	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
//	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));

	public static void main(String[] args) {
		//TODO Print to the LCD. Set up ActionController thread
		System.out.println("Start");
//		isEnter();
//		ActionController ac = new ActionController(leftMotor,rightMotor, clawLift, clawClose, frontUsSensor, sideUsSensor, lightSensor, colorSensor);
		ActionController ac = new ActionController(2000, true);
//		ac.start();
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
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
