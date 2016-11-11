

//DPM generic code

//Description:
/* The following fuctions will be generic methods that will be implemented as needed. 
   All fuctions will have relevent variable information above them such that the user may 
   declare all needed constants when resuing these methods.
*/


/* GENERAL COMMENTS:

all general movement code should be placed in the action controller and the odometer should
be made available to all other classes to that vales of x,y,theta can be accessed and used

*/

// Ultrasonic filters:

//clipping filter with adjustable clipper

private SampleProvider usSensor;
private float[] usData;

public float getClippedData(int maxValue){

	usSensor.fetchSample(usData,0)
	float distance = fetchSample(usData,0)*100;

	if(distance > maxValue)
	{ distance = maxValue; }
	
	return distance;
}

//Median Filter
EV3 API --> public MedianFilter(SampleProvider source, int bufferSize);


// Thread sleep (quick)

public void sleepThread(int time_millisecond){

	try
	{ Thread.sleep(time_milliseconds); } 

	catch (InterruptedException e) 
	{ /*nothing */ }
}

//GENERAL MOVEMENT:

// Speed adjustments


//note: left and right motors are declared

public void setSpeeds(int speed){

	leftMotor.setSpeed(speed);
	rightMotor.setSpeed(speed);
}

//stop motors smoothly
// uses "setSpeeds"

public void stopMotors(){

	setSpeeds(0);

	leftMotor.stop();
	rightMotor.stop();
}

// move forward a desired amount

//uses "setSpeeds" & "odometer" to fetch position

final static int FORWARD_SPEED;

public void goForward(float distance){

	setSpeeds(FORWARD_SPEED);

	double initial_X = odometer.getX();
	double initial_Y = odometer.getY();

	leftMotor.forward();
	rightMotor.forward();

	while((Math.sqrt(Math.pow(initial_X - odometer.getX(),2) ) + Math.sqrt( Math.pow(initial_Y - odometer.getY(),2) )) < distance){
	//do nothing / keep moving
	}

	stopMotors();

}


// Move backwards a set amount
final static int FORWARD_SPEED;

public void goBackward(float distance){

	setSpeeds(FORWARD_SPEED);

	double initial_X = odometer.getX();
	double initial_Y = odometer.getY();

	leftMotor.backward();
	rightMotor.backward();

	while((Math.sqrt(Math.pow(initial_X - odometer.getX(),2) ) + Math.sqrt( Math.pow(initial_Y - odometer.getY(),2) )) < distance){
	//do nothing / keep moving
	}

	stopMotors();
}


// NAVIGATION CODE & ANGLE CONTROL

// wrap angle so that θ ∈ [0,360)

public double wrapAngle(double angle){

	if (angle < 0)
		{ angle += 360;	}

	if (angle >= 360)
		{ angle -= 360; }

	return angle;
}

// calculate desired angle of wheel movment wrt robot dimensions 

public double convertAngle( double radius, double width, double angle){

	return ((Math.PI * width * angle / 2.0) / (Math.PI * radius));
}

// Calculate angle of current position relative to desired position (works given a cartesian angle map)
//must be tested ---> original code was changed to model lecture slides (delta_x and delta_y were swapped in this version)
//computations in rads not degrees but other methods take in rads
public double calculateAngle(double initial_x, double initial_y, double final_x, double final_y){

	double delta_x = (final_x - initial_x);
	double delta_y = (final_y - initial_y);

	if (delta_x < 0){

		if (delta_y > 0)
			{ return Math.atan(delta_y / delta_x) + Math.PI; }

		else
			{ return Math.atan(delta_y / delta_x) - Math.PI}
	}

	else
		{ return Math.atan(delta_y / delta_x); }
}

// TWO TYPES OF TURN TO: 

//TURN TO VERSION 1:

// Turn to a desired angle with single speed format using physical dimentions
// uses "stopMotors" & "calculateAngle"

/* for variable turn speed let speed be a function of θ such that 
   speed S(θ) = (1-f(θ))*ROTATE_SPEED where f(θ) ∈ [0,.75] to ensure smoothness&completion
	& do not use physical dimentions
	final int ERR_ANG; would be used to stop when in range
*/

final int ROTATE_SPEED;
final int TRACK;
final int RADIUS;

public void turnTo(double target_angle){

	stopMotors();
	setSpeeds();

	double delta_theta = Math.toDegrees(target_angle - odometer.getTheta());
	double turning_angle =convertAngle(RADIUS, TRACK, delta_theta;

	double adjusted_theta = 360 - delta_theta;
	double adjusted_turning_angle = convertAngle(RADIUS, TRACK, adjusted_theta);


	if (-180 < delta_theta && delta_theta < 180){

		rightMotor.rotate(-turning_angle, true);
		leftMotor.rotate(turning_angle, false)
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


//TRAVEL TO VERSION 1:

// uses "calculateTheta", "TurnTo", "odometer", "setSpeeds", "stopMotor"
final int DIST_ERR;
final int FORWARD_SPEED;

public void travelTo(double destination_x, destination_y){

	turnTo(calculateAngle(odometer.getX(), odometer.getY(), destination_x, destination_y);
	setSpeeds(FORWARD_SPEED);

	while ((Math.abs(odometer.getX() - destination_x) > DIST_ERR) || (Math.abs(odometer.getY() - destination_y) > DIST_ERR)){

		leftMotor.forward();
		rightMotor.backward();

	}

	stopMotors();

}

//NOTE: ALL ANGLES CAN BE CONVERTED TO DEGREES, WRAPPED AND PASSED BACK TO THE ODOMETER USING THE WRAP ANGLE METHOD


// RISING EDGE DELOCALIZATION

// uses "setSpeeds" and "stopMotors" & "odometer"


// NOTE: ASSUMES READINGS ARE IN ANGLES, OTHER METHODS ABOVE ARE IN RADS
final int US_MARGIN;
final int WALL_DIST;
final int CLIP;
final int ROTATE_SPEED;
/* using */ getClippedData(CLIP); // try median filter too!!

public void doLocalization(){

double [] position = new double [3];
		pos[0] = odometer.getX();
		pos[1] = odometer.getY();
		
		//array for setPosition method (it will only update the angle) given in lab4
		boolean[] update = {false, false, true};

	double cw_angle, ccw_angle;

	setSpeeds(ROTATE_SPEED);

	while (getClippedData(CLIP) == WALL_DIST){

		leftMotor.forward();
		rightMotor.backward();
	}

	while (getClippedData(CLIP) < WALL_DIST + US_MARGIN){

		leftMotor.forward();
		rightMotor.backward();
	}

	stopMotors();
	cw_angle = odometer.getTheta();
	setSpeeds(ROTATE_SPEED);

	while (getClippedData(CLIP) >= WALL_DIST){

		leftMotor.backward();
		rightMotor.forward();
	}

	while (getClippedData(CLIP) < WALL_DIST + US_MARGIN){

		leftMotor.backward();
		rightMotor.forward();
	}

	stopMotors();
	setSpeeds(ROTATE_SPEED);
	ccw_angle = odometer.getTheta

	if(clockwiseAngle > counterClockwiseAngle)
			{
				position[2] = (odometer.getTheta() + (45 - (clockwiseAngle + counterClockwiseAngle) / 2));
				
			}
			else{
				pos[2] = (odometer.getAng() + (225 - (clockwiseAngle + counterClockwiseAngle) / 2));
			}
			
			//set positions below
			odometer.setPosition(pos, update);

}

//set positions used in stock odometry code provided by ta's

public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}


