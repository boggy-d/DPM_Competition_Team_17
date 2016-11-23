/**
 * The OdometerCorrection class uses an algorithm
 * to correct the Odometer object's values
 * 
 * @author Eric Zimmermann
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

public OdometerCorrection(int INTERVAL, boolean autostart)
{
	if (autostart) {
		// if the timeout interval is given as <= 0, default to 20ms timeout 
		ocTimer = new Timer((INTERVAL <= 0) ? INTERVAL : Constants.DEFAULT_TIMEOUT_PERIOD, this);
		ocTimer.start();
	} else
		ocTimer = null;
}


public void timedOut(){

		//if line is detected -> correct ---> make entire while an if statement?
		if(ActionController.lightPoller.isLine()){

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
			
//			System.out.println("position 0**: "+position[0]);
//			System.out.println("position 1**: "+position[1]);
//			System.out.println("position 2**: "+position[2]);

		

			//update odometer
			ActionController.odometer.setPosition(position, update);
			
			//sleep thread quickly to avoid adjusting multiple times when passing a single gridline
			
			//verify it doesn't pass the same gridline twice
		}
}


// shifts position between wheels to position of color sensor
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
