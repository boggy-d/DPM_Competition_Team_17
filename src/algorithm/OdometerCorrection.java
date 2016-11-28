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
	
		// no longer on line so correct!
	
		if(isOnLine){
		
			isOnLine = false; //stop correcting since no longer on line!
			
			position[0] = ActionController.odometer.getX();
			position[1] = ActionController.odometer.getY();
			position[2] = ActionController.odometer.getAng();

//			System.out.println("position 0: "+position[0]);
//			System.out.println("position 1: "+position[1]);
//			System.out.println("position 2: "+position[2]);

			//transform location of point for correction from wheel center to color sensor
			transformPosition(position);
			
//			System.out.println("position 0*: "+position[0]);
//			System.out.println("position 1*: "+position[1]);
//			System.out.println("position 2*: "+position[2]);
//
//		
			//"normalize" positions by tile lengths
			norm_x = (position[0] / Constants.TILE_LENGTH);
			norm_y = (position[1] / Constants.TILE_LENGTH);
			
//			System.out.println("norm x: "+ norm_x);
//			System.out.println("norm y: "+ norm_y);

			//hold distance between position relative to nearest gridline
			delta_x = Math.abs((int)norm_x - norm_x);
			delta_y = Math.abs((int)norm_y - norm_y);
			
//			System.out.println("delta x: "+ delta_x);
//			System.out.println("delta y: "+ delta_y);
//			
			//rescale for minimum distances
			if (delta_x > 0.5)
			{ delta_x = 1 - delta_x ; }
			
			if (delta_y > 0.5)
			{ delta_y = 1 - delta_y ; }
			
//			System.out.println("delta x*: "+ delta_x);
//			System.out.println("delta y*: "+ delta_y);
//			
			if(Math.abs(delta_y - delta_x) > Constans.CORNER_MARGIN){

				//closest to horizontal gridline therefore y must be corrected 
				//add in additional and statement with a margin to reduce errors (test)
				if(delta_y < delta_x){
				
					if(norm_y + delta_y <= (int)norm_y + 1) // implies norm_y < position of horizontal gridline
						{ position[1] = ((int)norm_y + 1) * Constants.TILE_LENGTH; }
				
					else //implies norm_y > position of horizontal gridline
						{ position[1] = ((int)norm_y) * Constants.TILE_LENGTH; }
				}

			//closest to vertical gridline therefore x must be corrected
				else{

					if(norm_x + delta_x <= (int)norm_x + 1) // implies norm_x < position of vertical gridline
						{ position[0] = ((int)norm_x + 1) * Constants.TILE_LENGTH; }
				
					else //implies norm_x > position of vertical gridline
						{ position[0] = ((int)norm_x) * Constants.TILE_LENGTH; }
				}
			
				// may break down near gridline intersections

				//inverse transorm to scale correction from lightsensor position back to wheel contering
				inverseTransfrom(position);
			
//				System.out.println("position 0**: "+position[0]);
//				System.out.println("position 1**: "+position[1]);
//				System.out.println("position 2**: "+position[2]);

				//update odometer
				ActionController.odometer.setPosition(position, update);	
			}
		}
}


// shifts position between wheels to position of color sensor
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
