package parkingRobot.hsamr8;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.nxt.NXTMotor;
import parkingRobot.INavigation;

/**
 * Main class for control module
 *
 */
public class ControlRST implements IControl {
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	NXTMotor leftMotor = null;
	NXTMotor rightMotor = null;
	
	IPerception perception = null;
	INavigation navigation = null;
	ControlThread ctrlThread = null;	

    int leftMotorPower = 0;
	int rightMotorPower = 0;
	
	double velocity = 0.0;
	double angularVelocity = 0.0;
	
	Pose startPosition = new Pose();
	Pose currentPosition = new Pose();
	Pose destination = new Pose();
	
	ControlMode currentCTRLMODE = null ;
	
	EncoderSensor controlRightEncoder    = null;
	EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    
    
    double PI = 3.14159265359;
    double esum = 0; //für linefollow
	double e = 0;
	double ealt = 0;
	double kp = 0.39;
	double ki =0.05; //0.0025
	double kd = 0.0; //0.000001
	double y = 0;
	
	
	double radius = 0;   //für methode drive(V;W-Control)
	double wheelDiameter = 5.6;
	double trackWidth = 14;
	double distancePerTurn = PI*wheelDiameter;
	double distancePerDegree = distancePerTurn/360;
	double rightSpeed = 0;
	double leftSpeed = 0;
	double dleft = 0;
	double dright = 0;
	double dleftsum =0;
	double drightsum =0;
	double dleftalt =0;
	double drightalt =0;
	double yleft =0;
	double yright =0;
	double kp1 =0;
	double ki1 =0;
	double kd1 =0;
	
	
	
	
	/**
	 * provides the reference transfer so that the class knows its corresponding navigation object (to obtain the current 
	 * position of the car from) and starts the control thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param navigation corresponding main module Navigation class object
	 * @param leftMotor corresponding NXTMotor object
	 * @param rightMotor corresponding NXTMotor object
	 */
	public ControlRST(IPerception perception, INavigation navigation, NXTMotor leftMotor, NXTMotor rightMotor){
		this.perception = perception;
        this.navigation = navigation;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.currentCTRLMODE = ControlMode.INACTIVE;
			
		this.encoderLeft  = perception.getControlLeftEncoder();
		this.encoderRight = perception.getControlRightEncoder();
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		
		this.ctrlThread = new ControlThread(this);
		
		ctrlThread.setPriority(Thread.MAX_PRIORITY - 1); //legt 2.höhste priorität fest
		ctrlThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		ctrlThread.start();
	}
	

	// Inputs
	
	/**
	 * set velocity
	 * @see parkingRobot.IControl#setVelocity(double velocity)
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * set angular velocity
	 * @see parkingRobot.IControl#setAngularVelocity(double angularVelocity)
	 */
	public void setAngularVelocity(double angularVelocity) {
		this.angularVelocity = angularVelocity;

	}
	
	/**
	 * set destination
	 * @see parkingRobot.IControl#setDestination(double heading, double x, double y)
	 */
	public void setDestination(double heading, double x, double y){ //ist heading der winkel????????!!!!!!!
		this.destination.setHeading((float) heading);
		this.destination.setLocation((float) x, (float) y);
	}
	

	
	/**
	 * sets current pose
	 * @see parkingRobot.IControl#setPose(Pose currentPosition)
	 */
	public void setPose(Pose currentPosition) {
		// TODO Auto-generated method stub
		this.currentPosition = currentPosition;
	}
	

	/**
	 * set control mode
	 */
	public void setCtrlMode(ControlMode ctrl_mode) {
		this.currentCTRLMODE = ctrl_mode;
	}
	
	public ControlMode getCtrlMode(){
		return currentCTRLMODE;
	}
	/**
	 * set start time
	 */
	public void setStartTime(int startTime){
		this.lastTime = startTime;
	}
	
	/**
	 * selection of control-mode
	 * @see parkingRobot.IControl#exec_CTRL_ALGO()
	 */
	public void exec_CTRL_ALGO(){
		
		switch (currentCTRLMODE)
		{
		  case LINE_CTRL	: update_LINECTRL_Parameter();
		                      //exec_LINECTRL_ALGO();
		                      exec_LINECTRL_ALGO_PID();
		                      break;
		  case VW_CTRL		: //update_VWCTRL_Parameter();
		   					  exec_VWCTRL_ALGO();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO();
		                      break;
		  case PARK_CTRL	: update_PARKCTRL_Parameter();
		  					  exec_PARKCTRL_ALGO();
		  					  break;		  					  
		  case INACTIVE 	: exec_INACTIVE();
			                  break;
		}

	}
	
	// Private methods
	
	/**
	 * update parameters during VW Control Mode
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during SETPOSE Control Mode
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
	}
	
	/**
	 * update parameters during PARKING Control Mode
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
	}

	/**
	 * update parameters during LINE Control Mode
	 */
	/**private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();		
	}*/
	
	//für 3.1.3 liniensensordaten von 0 bis 100
	
    private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensorValue();
		this.lineSensorLeft  		= perception.getLeftLineSensorValue();		
	}
	
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity); 
		}
	
    private void exec_SETPOSE_ALGO(){
    	//Aufgabe 3.3
	}
	
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
	}
	
    private void exec_INACTIVE(){
    	this.stop();
	}
	
	/**
	 * DRIVING along black line
	 * Minimalbeispiel
	 * Linienverfolgung fuer gegebene Werte 0,1,2
	 * white = 0, black = 2, grey = 1
	 */
    
	private void exec_LINECTRL_ALGO(){
		leftMotor.forward();
		rightMotor.forward();
		int lowPower = 1;
		int midPower = 20; //mittlere Power eingeführt
		int highPower = 40; //maximale geschwindigkeiten bei den keine linienüberschreitung stattfindet
		

        if(this.lineSensorLeft == 2 && (this.lineSensorRight == 1)){
			
			// when left sensor is on the line, turn left
    	    leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
		} 
        else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 1)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
		}
		else if(this.lineSensorLeft == 2 && (this.lineSensorRight == 0)){
			
			// when left sensor is on the line, turn left
			leftMotor.setPower(lowPower);
			rightMotor.setPower(highPower);
			
		} 
		else if(this.lineSensorRight == 2 && (this.lineSensorLeft == 0)){
		
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(lowPower);
		}
		else if(this.lineSensorLeft == 1 && this.lineSensorRight == 0) {
				
			// when left sensor is on the line, turn left
			leftMotor.setPower(midPower); //mittlere Power eingesetzt
			rightMotor.setPower(highPower);
				
		} 
		else if(this.lineSensorRight == 1 && this.lineSensorLeft == 0) {
			
			// when right sensor is on the line, turn right
			leftMotor.setPower(highPower);
			rightMotor.setPower(midPower);   //mittlere Power eingesetzt
		}
		else if (this.lineSensorLeft == 0 && this.lineSensorRight == 0) { 
			
			//wenn beide Sensoren auf weiß sind, geradeaus fahren
			//fehleranfällig bei verlassen der führungslinie!!!!!!!! TESTEN!!!!
			leftMotor.setPower(highPower);
			rightMotor.setPower(highPower);
		}
	}
	
	//für 3.1.3 methode mit PID-Regler
	private void exec_LINECTRL_ALGO_PID() {  
		leftMotor.forward();
		rightMotor.forward();
		//int lowPower = 1;
		//int midPower = 20; //mittlere Power eingeführt
		//int highPower = 40; //maximale geschwindigkeiten bei den keine linienüberschreitung stattfindet
		
		
		
		e = (this.lineSensorRight - this.lineSensorLeft);  //(double)
		
		if(e > 93){                       //linkskurve
			leftMotor.setPower(-20);
			rightMotor.setPower(30);
			esum =0;
			ealt=0;
			
		}
		else if (e < -93 ){
			leftMotor.setPower(30);
			rightMotor.setPower(-20);
			esum =0;
			ealt=0;
		}
		else{  
		//if(e < 100 && e > -100){ 	
			esum = esum + (double)(e); //integrationsanteil
			y = kp*(double)(e) + ki*esum + kd*((double)(e) - ealt);
			ealt = (double)(e);
		
			rightMotor.setPower(38+(int)(y));
			leftMotor.setPower(38-(int)(y));
		
		
		}
	}
		
		
		

	
	
	
	
	
	private void stop(){
		this.leftMotor.stop();
		this.rightMotor.stop();
	}
		
    /**
     * calculates the left and right angle speed of the both motors with given velocity 
     * and angle velocity of the robot
     * @param v velocity of the robot
     * @param omega angle velocity of the robot
     */
	private void drive(double v, double omega){
		//Aufgabe 3.2
		
		
		/**if( omega != 0){                   //wenn Winkelgeschwindigkeit gegeben
			radius = v/omega;
			if(v != 0){
				leftSpeed = v - (trackWidth/2*v/radius);
				rightSpeed = v + (trackWidth/2*v/radius);
			}
			else{                            //falls v=0
				leftSpeed = trackWidth/2*omega;
				rightSpeed = -trackWidth/2*omega;
			};
				
		}
		else{                                      
			leftSpeed = v;
			rightSpeed = v;
		};
		leftMotor.setPower((int)leftSpeed);
		rightMotor.setPower((int)rightSpeed);
		
		*/
		
		AngleDifferenceMeasurement rightEncoder = perception.getControlRightEncoder().getEncoderMeasurement();    //mit ludwig absprechen an welcher stelle zu platzieren
		AngleDifferenceMeasurement leftEncoder = perception.getControlLeftEncoder().getEncoderMeasurement();      //mit ludwig absprechen an welcher stelle zu platzieren
		
		//mit regelung
		if( omega != 0){                   //wenn Winkelgeschwindigkeit gegeben
			radius = v/omega;
			if(v != 0){
				leftSpeed = v - (trackWidth/2*v/radius);
				rightSpeed = v + (trackWidth/2*v/radius);
			}
			else{                            //falls v=0
				leftSpeed = trackWidth/2*omega;
				rightSpeed = -trackWidth/2*omega;
			};
				
		}
		else{                                      
			leftSpeed = v;
			rightSpeed = v;
		};
		
		dleft = leftSpeed - wheelDiameter/2*(leftEncoder.getAngleSum())/(leftEncoder.getDeltaT());                //Fehler des linken Rades
		dright = rightSpeed - wheelDiameter/2*(rightEncoder.getAngleSum())/(rightEncoder.getDeltaT());			  //Fehler der rechten Rades
		
		dleftsum = dleftsum + dleft; //integrationsanteil
		yleft = kp1*dleft + ki1*dleftsum + kd1*(dleft - dleftalt);
		dleftalt = dleft;
		
		drightsum = drightsum + dright; //integrationsanteil
		yright = kp1*dright + ki1*drightsum + kd1*(dright - drightalt);
		drightalt = dright;
		
		
		leftMotor.setPower((int)(leftSpeed+dleft));
		rightMotor.setPower((int)(rightSpeed+dright));
	
		
	}
	
}