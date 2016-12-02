/**
 * The OdometerCorrection class uses an algorithm
 * to correct the Odometer object's values
 * 
 * @author Eric Zimmermann
 * @author Fiona Hang
 */

package algorithm;
import lejos.utility.Timer;
import lejos.utility.TimerListener;
import component.ActionController;
import component.Constants;

//to be started after localization!!!
public class OdometerCorrection implements TimerListener{

private double position[] = new double[3]; 
private boolean update[] = {true, true, false};
private double norm_x, norm_y;
private double delta_x, delta_y;
private Timer ocTimer;
private boolean isOnLine = false;

/**
 * Class contructor creating OdometerCorrection thread
 * 
 * @param INTERVAL the refresh time of the Timer
 * @param autostart the flag that start the Timer on startup or not
 */
public OdometerCorrection(int INTERVAL, boolean autostart)
{
	if (autostart) {
		// if the timeout interval is given as <= 0, default to 20ms timeout 
		ocTimer = new Timer((INTERVAL <= 0) ? INTERVAL : Constants.DEFAULT_TIMEOUT_PERIOD, this);
		ocTimer.start();
	} else
		ocTimer = null;
}

/**
 * Modifies the Odometer based on if the light sensor
 * sees a line or not
 */
public void timedOut(){

		//if on a line, continue to set line to true
		while(ActionController.lightPoller.isLine())
		{ isOnLine = true; }
	
		// no longer on line so correct
		if(isOnLine){
		
			isOnLine = false; //stop correcting since no longer on line (used to prevent multiple corrections)
			
			//fetch parameters from odometer 
			position[0] = ActionController.odometer.getX();
			position[1] = ActionController.odometer.getY();
			position[2] = ActionController.odometer.getAng();


			//transform location of point to find position of color sensor
			transformPosition(position);
			
			
			//"normalize" positions by tile lengths
			norm_x = (position[0] / Constants.TILE_LENGTH);
			norm_y = (position[1] / Constants.TILE_LENGTH);
			


			//hold distance (non integer between) position relative to nearest gridline
			delta_x = Math.abs((int)norm_x - norm_x);
			delta_y = Math.abs((int)norm_y - norm_y);
		
			//rescale for minimum distances to closest gridline
			if (delta_x > 0.5)
			{ delta_x = 1 - delta_x ; }
			
			if (delta_y > 0.5)
			{ delta_y = 1 - delta_y ; }
			
			//create margin to avoid correction near grid line intersections
			if(Math.abs(delta_y - delta_x) > Constants.CORNER_MARGIN){

				//closest to horizontal gridline therefore y must be corrected 
				if(delta_y < delta_x){
				
					if(norm_y + delta_y <= (int)norm_y + 1) // implies norm_y < position of horizontal gridline
						{ position[1] = ((int)norm_y + 1) * Constants.TILE_LENGTH; } //correct by updating value to nearest gridline
				
					else //implies norm_y > position of horizontal gridline
						{ position[1] = ((int)norm_y) * Constants.TILE_LENGTH; } //correct by updating value to nearest gridline
				}

			//closest to vertical gridline therefore x must be corrected
				else{

					if(norm_x + delta_x <= (int)norm_x + 1) // implies norm_x < position of vertical gridline
						{ position[0] = ((int)norm_x + 1) * Constants.TILE_LENGTH; } //correct by updating value to nearest gridline
				
					else //implies norm_x > position of vertical gridline
						{ position[0] = ((int)norm_x) * Constants.TILE_LENGTH; } //correct by updating value to nearest gridline
				}
			

				//inverse transorm to scale correction from lightsensor position back to wheel centering
				inverseTransfrom(position);
			

				//update odometer with corrected values
				ActionController.odometer.setPosition(position, update);	
			}
		}
}

/**
 * Shifts position between wheels to position of color sensor
 * @param position the position of the wheels
 */
public static void transformPosition(double position[]){

	position [0] -= Constants.SENSOR_DISTANCE * Math.cos(Math.toRadians(position[2]));
	position [1] -= Constants.SENSOR_DISTANCE * Math.sin(Math.toRadians(position[2]));
}

//shifts position of color sensor to position between wheels
public static void inverseTransfrom(double position[]){

	position [0] += Constants.SENSOR_DISTANCE * Math.cos(Math.toRadians(position[2]));
	position [1] += Constants.SENSOR_DISTANCE * Math.sin(Math.toRadians(position[2]));
}

}
