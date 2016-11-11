/**
 * The USLocaliztion class uses the Rising Edge algorithm
 * with the USS to orient the robot to a zero heading
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 * @author Eva Suska
 */

package algorithm;

import component.ActionController;
import component.Odometer;
import component.USPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalization {
	final static int ROTATE_SPEED = 100, CLIP = 55, WALL_DIST = 50, US_MARGIN = 3;

	// for constructor
	private Odometer odometer;
	private USPoller usPoller;

	private EV3LargeRegulatedMotor leftMotor;

	private EV3LargeRegulatedMotor rightMotor;

	/**
	 * Class Constructor specifying the Odometer and USPoller used
	 * 
	 * @param odometer the Odometer object used
	 * @param usPoller the USPoller object used
	 */
	public USLocalization(Odometer odometer, USPoller usPoller) {
		
		this.odometer = odometer;
		this.usPoller = usPoller;
		
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
	}

	/**
	 * Localize the robot to coordinate (0 , 0) with a heading of 0 by using the
	 * USPoller to detect Rising Edge
	 */
	public void localize() {
		// TODO Use Eric's code with Bogdan's architecture for this method

		double[] position = new double[3];
		position[0] = odometer.getX();
		position[1] = odometer.getY();

		// array for setPosition method (it will only update the angle) given in
		// lab4
		boolean[] update = { false, false, true };
		double cw_angle, ccw_angle;

		ActionController.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		while (usPoller.getClippedData(CLIP) == WALL_DIST) {

			leftMotor.forward();
			rightMotor.backward();
		}

		while (usPoller.getClippedData(CLIP) < WALL_DIST + US_MARGIN) {

			leftMotor.forward();
			rightMotor.backward();
		}

		ActionController.stopMotors();
		cw_angle = odometer.getAng();
		ActionController.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		while (usPoller.getClippedData(CLIP) >= WALL_DIST) {

			leftMotor.backward();
			rightMotor.forward();
		}

		while (usPoller.getClippedData(CLIP) < WALL_DIST + US_MARGIN) {

			leftMotor.backward();
			rightMotor.forward();
		}

		ActionController.stopMotors();
		ActionController.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		ccw_angle = odometer.getAng();

		if (cw_angle > ccw_angle) {
			position[2] = (odometer.getAng() + (45 - (cw_angle + ccw_angle) / 2));
		}

		else {
			position[2] = (odometer.getAng() + (225 - (cw_angle + ccw_angle) / 2));
		}

		// set positions below
		odometer.setPosition(position, update);

	}

	private float getFilteredData() {

		// TODO Remove this and use the USPoller class

//		usSensor.fetchSample(usData, 0);
//		float distance = usData[0] * 100;
//
		return 0;
	}
}