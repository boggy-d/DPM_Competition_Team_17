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
import component.Constants;
import component.Odometer;
import component.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	
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
	public USLocalizer(Odometer odometer, USPoller usPoller, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		
		this.odometer = odometer;
		this.usPoller = usPoller;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	/**
	 * Localize the robot to coordinate (0 , 0) with a heading of 0 by using the
	 * USPoller to detect Rising Edge
	 */
	
	//Renamed in order to differentiate the type of localization: US vs Light
	public void usLocalize() {
		// TODO Use Eric's code with Bogdan's architecture for this method

		double[] position = new double[3];
		position[0] = odometer.getX();
		position[1] = odometer.getY();

		// array for setPosition method (it will only update the angle) given in
		// lab4
		boolean[] update = { false, false, true };
		double cw_angle, ccw_angle;

		ActionController.setSpeeds(Constants.ROTATE_SPEED, -Constants.ROTATE_SPEED, true);

		while (usPoller.getClippedData(Constants.CLIP) == Constants.WALL_DIST) {

			//already moving cw
		}

		while (usPoller.getClippedData(Constants.CLIP) < Constants.WALL_DIST + Constants.US_MARGIN) {

			//already moving cw
		}

		ActionController.stopMotors();
		cw_angle = odometer.getAng();
		Sound.beep();
		ActionController.setSpeeds(-Constants.ROTATE_SPEED, Constants.ROTATE_SPEED, true);

		while (usPoller.getClippedData(Constants.CLIP) >= Constants.WALL_DIST) {

			//already moving ccw
		}

		while (usPoller.getClippedData(Constants.CLIP) < Constants.WALL_DIST + Constants.US_MARGIN) {

			//already moving ccw
		}

		ActionController.stopMotors();
		ccw_angle = odometer.getAng();
		Sound.beep();

		if (cw_angle > ccw_angle) {
			position[2] = (odometer.getAng() + (45 - (cw_angle + ccw_angle) / 2));
		}

		else {
			position[2] = (odometer.getAng() + (225 - (cw_angle + ccw_angle) / 2));
		}

		// set positions below
		odometer.setPosition(position, update);
		
		//rotate robot to 45 degrees in order for light localization to be performed
		//where should this be handled?
		
	}

}
