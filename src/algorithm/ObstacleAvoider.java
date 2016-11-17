/**
 * The Obstacle avoidance class runs when an object that is not
 * blue is detected by the robot's USS. The robot runs a specific
 * routine to avoid either wooden blocks or walls.
 * 
 * @author Bogdan Dumitru
 */

package algorithm;

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
		
	}
}
