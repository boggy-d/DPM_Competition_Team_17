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
		// TODO Use Eric's code with Bogdan's architecture for this method

		double[] position = new double[3];
		position[0] = ActionController.odometer.getX();
		position[1] = ActionController.odometer.getY();

		// array for setPosition method (it will only update the angle) given in
		// lab4
		boolean[] update = { false, false, true };
		double cw_angle, ccw_angle;

		ActionController.setSpeeds(Constants.ROTATE_SPEED, -Constants.ROTATE_SPEED, true);

		while (ActionController.usPoller.getClippedData(Constants.CLIP) == Constants.WALL_DIST) {

			//already moving cw
		}

		while (ActionController.usPoller.getClippedData(Constants.CLIP) < Constants.WALL_DIST + Constants.US_MARGIN) {

			//already moving cw
		}

		ActionController.stopMotors();
		cw_angle = ActionController.odometer.getAng();
		Sound.beep();
		ActionController.setSpeeds(-Constants.ROTATE_SPEED, Constants.ROTATE_SPEED, true);

		while (ActionController.usPoller.getClippedData(Constants.CLIP) >= Constants.WALL_DIST) {

			//already moving ccw
		}

		while (ActionController.usPoller.getClippedData(Constants.CLIP) < Constants.WALL_DIST + Constants.US_MARGIN) {

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
		
		//rotate robot to 45 degrees in order for light localization to be performed
		//where should this be handled?
		
	}

}
