/**
 * The Obstacle avoidance class runs when an object that is not
 * blue is detected by the robot's USS. The robot runs a specific
 * routine to avoid either wooden blocks or walls.
 * 
 * @author Bogdan Dumitru
 */

package algorithm;

import component.ActionController;
import component.Constants;

public class ObstacleAvoider {

	/**
	 * Checks how long the USS has been detecting values
	 * under the threshold and returns if the object is a wall
	 * 
	 * @return <code>true</code> if the USS has been detecting values under the threshold for n (TBD) seconds, <code>false</code> otherwise
	 */
	public boolean isWall()
	{
		return false;
		
	}
	
	/**
	 * Turns the robot 90deg and breaks out of obstacle avoidance (exact algorithm still TBD)
	 * 
	 * @see component.TimeRecorder
	 * @see component.Odometer
	 */
	public void avoidWall()
	{
		
	}
	
	/**
	 * Turns the robot 90deg and makes it go forward until the second
	 * USS does not detect an obstacle anymore (exact algorithm still TBD)
	 * 
	 * @see USPoller
	 * @see TimeRecorder
	 * @see Odometer
	 */
	public void avoidObstacle()
	{
		//ASSUMING THAT THE 2ND US IS ON THE LEFT
		
		//second us sensor is at 90 degrees
		//problem with this method: doesn't work if we are close to the corner of the block
		//after probing is done, back up
		ActionController.goForward(5, Constants.FORWARD_SPEED);
		
		//rotate (both direction) until we get the smallest distance on front US (forms a 90 degree with block)
		//like rising/falling edge
		
		//rotates the robot clockwise at speed: ROTATE_SPEED
		ActionController.setSpeeds(Constants.ROTATE_SPEED, -Constants.ROTATE_SPEED, true);
		
		//rotate clockwise until don't see block
		while (ActionController.frontUsPoller.getClippedData(Constants.CLIP) < Constants.WALL_DIST) {
			//save the smallest us distance we see
			
		}
		ActionController.stopMotors();
		
		//rotates the robot counterclockwise at speed: ROTATE_SPEED
		ActionController.setSpeeds(-Constants.ROTATE_SPEED, Constants.ROTATE_SPEED, true);
		
		//rotate counterclockwise until don't see block
		while (ActionController.frontUsPoller.getClippedData(Constants.CLIP) < Constants.WALL_DIST) {
			//save the smallest us distance we see
		
		}
		ActionController.stopMotors();
		
		//rotate to the smallest distance we got
		
		
		//turn 90 degrees clockwise
		int turn90 = ActionController.navigator.convertAngle(Constants.RADIUS, Constants.TRACK, 90);
		Constants.rightMotor.rotate(turn90, true);
		Constants.leftMotor.rotate(-turn90, false);
		
		//move forward until 2nd US doesn't see the obstacle and stop
		
		
		/////////////////////////////////////////////////////////////////////////////////
		//another way of doing it: US might need to be at 45 degrees
		//rotate a little clockwise so the second US can see
		int turn45 = ActionController.navigator.convertAngle(Constants.RADIUS, Constants.TRACK, 45);
		Constants.rightMotor.rotate(turn45, true);
		Constants.leftMotor.rotate(-turn45, false);
		
		//BangBang (make a new class or write it here?)
		
		
		//have an angle condition to stop BangBang
		
		
	}
}
