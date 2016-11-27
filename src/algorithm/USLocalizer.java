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
import lejos.hardware.Sound;

public class USLocalizer {

	/**
	 * Localize the robot to coordinate (0 , 0) with a heading of 0 by using the
	 * USPoller to detect Rising Edge
	 */
	
	//Renamed in order to differentiate the type of localization: US vs Light
	public void usLocalize() {

		double[] position = new double[3];
		position[0] = ActionController.odometer.getX();
		position[1] = ActionController.odometer.getY();

		// array for setPosition method (it will only update the angle) given in
		// lab4
		boolean[] update = { false, false, true };
		double cw_angle, ccw_angle;

		//rotates the robot clockwise at speed: FAST_ROTATION_SPEED
		ActionController.setSpeeds(Constants.FAST_ROTATION_SPEED, -Constants.FAST_ROTATION_SPEED, true);
		
		//keep turning while distance is large
		while (!ActionController.usPoller.isFrontBlock()) {

			//already moving cw
		}

		//keep turning after seeing the wall until we reach the 2nd wall, then stop
		while (ActionController.usPoller.getClippedData(Constants.frontUsSensor, Constants.CLIP) < Constants.OBSTACLE_DISTANCE_THRESHOLD + Constants.US_MARGIN) {

			//already moving cw
		}

		ActionController.stopMotors();
		cw_angle = ActionController.odometer.getAng();
		Sound.beep();
		ActionController.setSpeeds(-Constants.FAST_ROTATION_SPEED, Constants.FAST_ROTATION_SPEED, true);

		//turn counterclockwise until its not at the "edge" of the wall anymore
		while (!ActionController.usPoller.isFrontBlock()) {

			//already moving ccw
		}

		//keep turning until it sees the other edge of the wall
		while (ActionController.usPoller.getClippedData(Constants.frontUsSensor, Constants.CLIP) < Constants.OBSTACLE_DISTANCE_THRESHOLD + Constants.US_MARGIN) {

			//already moving ccw
		}

		ActionController.stopMotors();
		ccw_angle = ActionController.odometer.getAng();
		Sound.beep();
		
		//TODO add constants
		if (cw_angle > ccw_angle) {
			position[2] = (ActionController.odometer.getAng() + (45 - (cw_angle + ccw_angle) / 2));
		}

		else {
			position[2] = (ActionController.odometer.getAng() + (225 - (cw_angle + ccw_angle) / 2));
		}

		// set positions below
		ActionController.odometer.setPosition(position, update);
		
	}

}
