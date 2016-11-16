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

		//if line is detected -> correct ---> make entire while an if statement?
		while(lightPoller.isLine()){

			position[0] = odometer.getX();
			position[1] = odometer.getY();
			position[2] = odometer.getAng();


			//transform loction of point for correction from wheel center to color sensor
			transformPosition(position);

			//"normalize" positions by tile lengths
			double norm_x = (position[0] / Constants.TILE_LENGTH);
			double norm_y = (position[1] / Constants.TILE_LENGTH);

			//hold distance between position relative to nearest gridline
			double delta_x = Math.abs((int)norm_x - norm_x);
			double delta_y = Math.abs((int)norm_y - norm_y);


			//closest to horizontal gridline therefore y must be corrected
			if(delta_y > delta_x){
				
				if(norm_x + delta_x <= (int)norm_x + 1) // implies norm_x < position of horizontal gridline
					{ position[1] = ((int)norm_x + 1) * Constants.TILE_LENGTH; }
				
				else //implies norm_x > position of horizontal gridline
					{ position[1] = ((int)norm_x) * Constants.TILE_LENGTH; }
			}

			//closest to vertical gridline therefore x must be corrected
			else{

				if(norm_y + delta_y <= (int)norm_y + 1) // implies norm_y < position of vertical gridline
					{ position[0] = ((int)norm_y + 1) * Constants.TILE_LENGTH; }
				
				else //implies norm_x > position of vertical gridline
					{ position[0] = ((int)norm_y) * Constants.TILE_LENGTH; }

			}

			//inverse transorm to scale correction from lightsensor position back to wheel contering
			inverseTransfrom(position);

			//update odometer
			odometer.setPosition(position, update);

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
