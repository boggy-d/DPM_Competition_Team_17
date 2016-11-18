/**
 * The Navigation class moves the robot around the
 * competition field, updates the odometer, and checks
 * for obstacles
 * 
 * @author Bogdan Dumitru
 * @author Eric Zimmermann
 * @version 1.0.0
 * @since 0.1.0
 */

package algorithm;

import component.ActionController;
import component.Constants;
import component.Odometer;
import component.USPoller;

public class Navigator{
	/**
	 * Class constructor specifying the Odometer and the USPoller to be used
	 * 
	 * @param odometer the Odometer object used
	 * @param usPoller the USPoller object used
	 * 
	 * @see Odometer
	 * @see USPoller
	 */

	public Navigator(){
		Constants.leftMotor.setAcceleration(Constants.ACCELERATION);
		Constants.rightMotor.setAcceleration(Constants.ACCELERATION);
	}
	
	/**
	 * Wraps the angle to fit within 0 and 360 degrees
	 * 
	 * @param angle the angle to be wrapped
	 * @return the new angle value between 0 and 360
	 */
	public double wrapAngle(double angle){
		
		//TODO Might be deprecated because of the fixDegreeAngle method
		
		if (angle < 0)
		{ angle += 360;	}

		if (angle >= 360)
		{ angle -= 360; }

		return angle;
	}
	
	/**
	 * Calculates rotations needed to be performed by both wheels in order to rotate by a certain angle. 
	 * Rotations needed are calculated by the relative dimensions of the bot
	 * accuracy is clipped by casting actual value to an integer value
	 * @param radius the radius of the wheels
	 * @param width the distance between the wheels
	 * @param angle the angle to convert
	 * @return the converted angle
	 */
	public int convertAngle( double radius, double width, double angle){

		return (int) ((width * angle / 2.0) / (radius));
	}

	/**
	 * Calculates the angle to turn to based on the robot's
	 * position and the desired destination position
	 * 
	 * @param initial_x the robot's X coordinate
	 * @param initial_y the robot's Y coordinate
	 * @param final_x the destination's X coordinate
	 * @param final_y the destination's Y coordinate
	 * @return the angle, in radians, to turn to
	 */
	public double calculateAngle(double initial_x, double initial_y, double final_x, double final_y){

		double delta_x = (final_x - initial_x);
		double delta_y = (final_y - initial_y);
		double return_angle = Math.atan(delta_y / delta_x);  //edited for efficiency such that minimal work is required
		
		if (delta_x < 0){

			if (delta_y > 0)
				{ return_angle += Math.PI; }

			else
				{ return_angle -= Math.PI; }
		}
		
		return Math.toDegrees(return_angle);
	}

	/**
	 * Rotates the robot to the desired angle
	 * 
	 * @param target_angle the angle to rotate to
	 */
	public void turnTo(double target_angle){

		ActionController.stopMotors();
		ActionController.setSpeeds(Constants.FAST_ROTATION_SPEED, Constants.FAST_ROTATION_SPEED, false); 


		double delta_theta = target_angle - ActionController.odometer.getAng();
		int turning_angle =convertAngle(Constants.RADIUS, Constants.TRACK, delta_theta);
		System.out.println("target_angle:" + target_angle);
		System.out.println("delta_theta:"+ delta_theta);
		System.out.println("odo ang:"+ ActionController.odometer.getAng());
		System.out.println("turning angle:"+ turning_angle);
		
		
		double adjusted_theta = 360 - delta_theta;
		int adjusted_turning_angle = convertAngle(Constants.RADIUS, Constants.TRACK, adjusted_theta);
		
		
		System.out.println("adjusted theta:"+ adjusted_theta);
		System.out.println("adjusted turning angle:" + adjusted_turning_angle);

		if (-180 < delta_theta && delta_theta < 180){

			System.out.println("1");

			Constants.rightMotor.rotate(-turning_angle, true);
			Constants.leftMotor.rotate(turning_angle, false);
		}

		else if (delta_theta < -180){
			System.out.println("2");

			Constants.rightMotor.rotate(-adjusted_turning_angle, true);
			Constants.leftMotor.rotate(adjusted_turning_angle, false);
		}

		else{

			System.out.println("3");

			Constants.rightMotor.rotate(adjusted_turning_angle, true);
			Constants.leftMotor.rotate(-adjusted_turning_angle, false);
		}
		
		
	}
	
	/**
	 * Moves the robot to the desired coordinate in the grid field
	 * by making the motors advance
	 * 
	 * @param destination_x the X coordinate to travel to
	 * @param destination_y the Y coordinate to travel to
	 */
	public void travelTo(double destination_x, double destination_y){

		turnTo(calculateAngle(ActionController.odometer.getX(), ActionController.odometer.getY(), destination_x, destination_y));
		ActionController.setSpeeds(Constants.FAST_FORWARD_SPEED, Constants.FAST_FORWARD_SPEED, true);

		while ((Math.abs(ActionController.odometer.getX() - destination_x) > Constants.DIST_ERR) || (Math.abs(ActionController.odometer.getY() - destination_y) > Constants.DIST_ERR)){

			//already moving forward -- do nothing
		}
		
		ActionController.stopMotors();
	}


	//TODO might have to convert all methods to work in degrees
}
