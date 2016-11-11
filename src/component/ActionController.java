/**
 * The ActionController class acts as a finite state
 * machine for the system as it controls the flow
 * of the routine.
 * 
 * @author Bogdan Dumitru
 * @version 0.2.0
 * 
 */

package component;

import algorithm.ClawController;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.TimerListener;

public class ActionController implements TimerListener {
	private final static EV3LargeRegulatedMotor clawLift = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private final static EV3LargeRegulatedMotor clawClose = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	private final int CLAW_LIFT_FULL = 775;
	private final int CLAW_LIFT_ONE_BLOCK = -700;
	private final int CLAW_LIFT_TWO_BLOCK = 600;
	private final int CLAW_LIFT_THREE_BLOCK = 500;

	private Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;

	/**
	 * Class constructor
	 */
	public ActionController() {

	}

	/**
	 * Turns the left and right motors at
	 * the specified speeds
	 * 
	 * @param lSpd	the speed of the left motor
	 * @param rSpd	the speed of the right motor
	 */
	public static void setSpeeds(int lSpd, int rSpd) {

		leftMotor.setSpeed(lSpd);
		rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			leftMotor.backward();
		else
			leftMotor.forward();
		if (rSpd < 0)
			rightMotor.backward();
		else
			rightMotor.forward();
	}

	/**
	 * Halts the motors
	 */
	public static void stopMotors() {

		setSpeeds(0,0);

		leftMotor.stop();
		rightMotor.stop();
	}

	/**
	 * Makes the robots advance or back up by a distance at a
	 * specified speed
	 * 
	 * @param distance the distance (in cm) to travel
	 * @param speed the speed of the wheels
	 */
	public void goForward(float distance, int speed) {

		setSpeeds(speed, speed);

		double initial_X = odometer.getX();
		double initial_Y = odometer.getY();
		
		while ((Math.sqrt(Math.pow(initial_X - odometer.getX(), 2))
				+ Math.sqrt(Math.pow(initial_Y - odometer.getY(), 2))) < distance) {
			// do nothing / keep moving
		}

		stopMotors();
	}

	/**
	 * Gets the competition information provided by WiFi
	 * and stores it into fields
	 * 
	 * @see WifiConnection
	 */
	public void setWifiInfo() {
		
		//TODO Figure out how to get the wifi info
	}

	/**
	 * Sets up the odometer thread
	 * 
	 * @see Odometer
	 */
	public void startOdometer() {
		//TODO Instantiate odometer. Start odometer thread
	}

	/**
	 * Calculates the remaining time and returns if there is
	 * enough time to continue the routine
	 * 
	 * @return	<code>true</code> if the time is almost up, otherwise returns <code>false</code>
	 */
	public boolean isTimeShort() {

		//TODO Figure out how to get the time. Write algorithm to calculate remaining time
		
		return false;
	}

	/**
	 * Reads the <code>BTN</code> and <code>CTN</code> and returns the
	 * robots job for the heat
	 * @return <code>true</code> if the robot is a builder or <code>false</code> if it is a collector
	 */
	public boolean jobAssigned() {
		//TODO Write simple if-else. Create fields
		
		return false;

	}

	/**
	 * Moves the robot to the starting corner
	 */
	public void goToStart() {
		//TODO Write algorithm to go back to starting position while avoiding blocks
	}

	@Override
	public void timedOut() {
		// TODO EVERYTHING!!!
		
		startOdometer();

		ClawController claw = new ClawController(clawLift, clawClose);

		claw.grab();
		claw.lift(CLAW_LIFT_FULL);
		while (Button.waitForAnyPress() != Button.ID_LEFT)
			;
		claw.lift(CLAW_LIFT_ONE_BLOCK);
		claw.release();
		claw.lift(CLAW_LIFT_ONE_BLOCK - CLAW_LIFT_FULL);
	}
}
