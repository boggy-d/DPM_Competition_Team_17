/**
 * The OdometerCorrection class uses an algorithm
 * to correct the Odometer object's values
 * 
 * @author Eric Zimmermann
 */

package algorithm;
import component.Odometer;
import component.LightPoller;
import component.Constants;

//to be started after localization!!!
public class OdometerCorrection extends Thread{

private Odometer odometer;
private LightPoller lightPoller;

double position[] = new double[3]; 
boolean update[] = {true, true, false};

//constructor
public OdometerCorrection(Odometer odometer, LightPoller lightPoller){
	
	this.odometer = odometer;
	this.lightPoller = lightPoller;
}


public void Run(){

	while(true){
		
		double prevLightData = lightPoller.getLightData();

		//if line is detected -> correct ---> make entire while an if statement?
		if(lightPoller.isLine(prevLightData)){

			position[0] = odometer.getX();
			position[1] = odometer.getY();
			position[2] = odometer.getAng();


			//transform location of point for correction from wheel center to color sensor
			transformPosition(position);

			//"normalize" positions by tile lengths
			double norm_x = (position[0] / Constants.TILE_LENGTH);
			double norm_y = (position[1] / Constants.TILE_LENGTH);

			//hold distance between position relative to nearest gridline
			double delta_x = Math.abs((int)norm_x - norm_x);
			double delta_y = Math.abs((int)norm_y - norm_y);
			
			//rescale for minimum distances
			if (delta_x > 0.5)
			{ delta_x = 1 - delta_x ; }
			
			if (delta_y > 0.5)
			{ delta_y = 1 - delta_y ; }

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

			//update odometer
			odometer.setPosition(position, update);
			
			//sleep thread quickly to avoid adjusting multiple times when passing a single gridline
		}
	}
}


// shifts position between wheels to position of color sensor
public static void transformPosition(double position[]){

	position [0] -= Constants.SENSOR_DISTANCE * Math.cos(Math.toRadians(position[2]));
	position [0] -= Constants.SENSOR_DISTANCE * Math.sin(Math.toRadians(position[2]));
}

//shifts position of color sensor to position between wheels
public static void inverseTransfrom(double position[]){

	position [0] += Constants.SENSOR_DISTANCE * Math.cos(Math.toRadians(position[2]));
	position [0] += Constants.SENSOR_DISTANCE * Math.sin(Math.toRadians(position[2]));
}

}
