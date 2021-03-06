/**
 * The Navigation class moves the robot around the
 * competition field, updates the odometer, and checks
 * for obstacles
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 * @author Fiona Hang
 */

package algorithm;

import component.ActionController;
import component.Constants;
import component.Odometer;
import component.USPoller;

public class Navigator {
	public double dR;
	public int movementCounter;
	
	/**
	 * Class constructor setting the acceleration of the motors
	 * 
	 * @see Odometer
	 * @see USPoller
	 */

	public Navigator() {
		//set accelerations of motors
		Constants.leftMotor.setAcceleration(Constants.ACCELERATION);
		Constants.rightMotor.setAcceleration(Constants.ACCELERATION);
	}

	/**
	 * Wraps the angle to fit within 0 and 360 degrees
	 * 
	 * @param angle
	 *            the angle to be wrapped
	 * @return the new angle value between 0 and 360
	 */
	public double wrapAngle(double angle) {

		if (angle < 0) {
			angle += 360;
		}

		if (angle >= 360) {
			angle -= 360;
		}

		return angle;
	}

	/**
	 * Calculates rotations needed to be performed by both wheels in order to
	 * rotate by a certain angle. Rotations needed are calculated by the
	 * relative dimensions of the bot and accuracy is clipped by casting actual
	 * value to an integer value
	 * 
	 * @param radius
	 *            the radius of the wheels
	 * @param width
	 *            the distance between the wheels
	 * @param angle
	 *            the angle to convert
	 * @return the converted angle
	 */
	public int convertAngle(double radius, double width, double angle) {

		return (int) ((width * angle / 2.0) / (radius));
	}

	/**
	 * Calculates the angle to turn to based on the robot's position and the
	 * desired destination position
	 * 
	 * @param initial_x
	 *            the robot's X coordinate
	 * @param initial_y
	 *            the robot's Y coordinate
	 * @param final_x
	 *            the destination's X coordinate
	 * @param final_y
	 *            the destination's Y coordinate
	 * @return the angle, in radians, to turn to
	 */
	public double calculateAngle(double initial_x, double initial_y, double final_x, double final_y) {

		double delta_x = (final_x - initial_x);
		double delta_y = (final_y - initial_y);
		double return_angle = Math.atan(delta_y / delta_x); // first quadrant
															// angle wrt x-axis

		if (delta_x < 0) {

			if (delta_y > 0) // 2nd quadrant angle wrt x-axis
			{
				return_angle += Math.PI;
			}

			else // 3rd quadrant angle wrt x-axis
			{
				return_angle += Math.PI;
			}
		}

		if (delta_y < 0 && delta_x > 0) // 4th quadrant wrt x-axis --> will be
										// handles by wraping angle
		{
			return_angle += 2 * Math.PI;
		}

		return Math.toDegrees(return_angle);
	}

	/**
	 * Rotates the robot to the desired angle
	 * 
	 * @param target_angle
	 *            the angle to rotate to
	 */
	public void turnTo(double target_angle) {

		ActionController.stopMotors();
		ActionController.setSpeeds(Constants.FAST_ROTATION_SPEED, Constants.FAST_ROTATION_SPEED, false);
		
		
		//calculate angle required to turn one direction
		double delta_theta = target_angle - ActionController.odometer.getAng();
		int turning_angle = convertAngle(Constants.RADIUS, Constants.TRACK, delta_theta);
		
		//calculate angle required to turn in other direction
		double adjusted_theta = 360 - Math.abs(delta_theta);
		int adjusted_turning_angle = convertAngle(Constants.RADIUS, Constants.TRACK, adjusted_theta);

		//turn by the minimal angle
		if (-180 < delta_theta && delta_theta < 180) {

			Constants.rightMotor.rotate(turning_angle, true);
			Constants.leftMotor.rotate(-turning_angle, false);
		}

		else if (delta_theta < -180) {

			Constants.rightMotor.rotate(adjusted_turning_angle, true);
			Constants.leftMotor.rotate(-adjusted_turning_angle, false);
		}

		else {
			Constants.rightMotor.rotate(-adjusted_turning_angle, true);
			Constants.leftMotor.rotate(adjusted_turning_angle, false);
		}

	}

	/**
	 * Moves the robot to the desired coordinate in the grid field by making the
	 * motors advance
	 * 
	 * @param destination_x
	 *            the X coordinate to travel to
	 * @param destination_y
	 *            the Y coordinate to travel to
	 */
	public void travelTo(double final_x, double final_y) {

		turnTo(calculateAngle(ActionController.odometer.getX(), ActionController.odometer.getY(), final_x, final_y));
		
		//calculate component differnce in current poition relative to final position
		double delta_x = Math.abs(final_x - ActionController.odometer.getX());																
		double delta_y = Math.abs(final_y - ActionController.odometer.getY());
		
		//calculate total scalr distance between two points
		double total_distance = Math.sqrt((Math.pow(delta_x, 2) + Math.pow(delta_y, 2)));
		
		//calculate rotations require by wheels to travel a set distance
		double total_rotations = calculateForwardRotation(total_distance);
	
		//rotate wheels
		ActionController.setSpeeds(Constants.FAST_FORWARD_SPEED, Constants.FAST_FORWARD_SPEED, false);
		Constants.leftMotor.rotate((int) total_rotations, true);
		Constants.rightMotor.rotate((int) total_rotations, false);

	}

	
	/**
	 * Moves the robot to the desired coordinate in the grid field by making the
	 * motors advance in partitions instead of one piece
	 * 
	 * @param destination_x
	 *            the X coordinate to travel to
	 * @param destination_y
	 *            the Y coordinate to travel to
	 * @param partitions
	 *            number of iterations to travel to
	 */
	public void partitionedPathTravelTo(double final_x, double final_y, int partitions) {

		turnTo(calculateAngle(ActionController.odometer.getX(), ActionController.odometer.getY(), final_x, final_y)); // now
																														// move
																														// straight

		double delta_x = Math.abs(final_x - ActionController.odometer.getX()); 									// dist
		double delta_y = Math.abs(final_y - ActionController.odometer.getY()); 
		double total_distance = Math.sqrt((Math.pow(delta_x, 2) + Math.pow(delta_y, 2)));
		double dP = total_distance / partitions; // incremental piece of distance in cm
		dR = this.calculateForwardRotation(dP); // incremental piece of rotation in degrees
		
		movementCounter = 0; // hold how many incremental pieces have been covered
		
		// for timed out
		ActionController.setSpeeds(Constants.FAST_FORWARD_SPEED, Constants.FAST_FORWARD_SPEED, false);

	}
	
	/**
	 * Calculates rotations of a wheel for a given distance
	 * @param distance distance to be travelled in cm
	 * @return number of wheel rotations
	 */
	public double calculateForwardRotation(double distance) {

		// circumfrence C is 2PIr which corresponds to 360 degrees
		// we travel a distance D which which contains D/C rotations
		// total rotations in dregrees is therefore 360*(D/C)


		return 360 * (distance / (2 * Math.PI * Constants.RADIUS));

	}

}
