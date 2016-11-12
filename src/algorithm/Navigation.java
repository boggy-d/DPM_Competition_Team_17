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
import component.Odometer;
import component.USPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.TimerListener;

public class Navigation implements TimerListener{

	//Speed variables
	
	final static int ACCELERATION = 3000, FORWARD_SPEED = 200, ROTATION_SPEED = 100;
	final static int TRACK = 0000;
	final static int RADIUS = 0000;
	final static int DIST_ERR = 0000;
	
	private Odometer odometer;
	private USPoller usPoller;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	/**
	 * Class constructor specifying the Odometer and the USPoller to be used
	 * 
	 * @param odometer the Odometer object used
	 * @param usPoller the USPoller object used
	 * 
	 * @see Odometer
	 * @see USPoller
	 */
	public Navigation(Odometer odometer, USPoller usPoller){
		
		this.odometer = odometer;
		this.usPoller = usPoller;
		
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
		
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
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

		return (int)Math.toDegrees(((width * angle / 2.0) / (radius)));
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
		ActionController.setSpeeds(ROTATION_SPEED, ROTATION_SPEED); //SETSPEEDS CAUSES BOT TO MOVE FORWARD *ERROR*

		double delta_theta = target_angle - odometer.getAng();
		int turning_angle =convertAngle(RADIUS, TRACK, delta_theta);

		double adjusted_theta = 360 - delta_theta;
		int adjusted_turning_angle = convertAngle(RADIUS, TRACK, adjusted_theta);


		if (-180 < delta_theta && delta_theta < 180){

			rightMotor.rotate(-turning_angle, true);
			leftMotor.rotate(turning_angle, false);
		}

		else if (delta_theta < -180){

			rightMotor.rotate(-adjusted_turning_angle, true);
			leftMotor.rotate(adjusted_turning_angle, false);
		}

		else{

			rightMotor.rotate(adjusted_turning_angle, true);
			leftMotor.rotate(-adjusted_turning_angle, false);
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

		turnTo(calculateAngle(odometer.getX(), odometer.getY(), destination_x, destination_y));
		ActionController.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

		while ((Math.abs(odometer.getX() - destination_x) > DIST_ERR) || (Math.abs(odometer.getY() - destination_y) > DIST_ERR)){

			leftMotor.forward();
			rightMotor.backward();
		}
		
		ActionController.stopMotors();
	}

	@Override
	public void timedOut() {
		// TODO Auto-generated method stub
		
	}

	//might have to convert all methods to work in degrees
}
