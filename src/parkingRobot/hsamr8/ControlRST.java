package parkingRobot.hsamr8;


import lejos.robotics.navigation.Pose;
import parkingRobot.IControl;
import parkingRobot.IPerception;
import parkingRobot.IPerception.*;
import lejos.geom.Point;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
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
	
	
	IPerception.AngleDifferenceMeasurement left_encoder;
	IPerception.AngleDifferenceMeasurement right_encoder;
	
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
	
	//EncoderSensor controlRightEncoder    = null;
	//EncoderSensor controlLeftEncoder     = null;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    
    
    //double PI = 3.14159265359;  //besreits definiert durch Math.PI
    double esum = 0; //für linefollow
	double e = 0;
	double ealt = 0;
	double kp = 0.387;
	double ki =0.0042; //0.0025
	double kd = 0.00008; //0.000001
	double y = 0;
	
	

	double wheelDiameter = 5.6;  //in cm
	double trackWidth = 14;   //in cm
	double distancePerTurn = Math.PI*wheelDiameter; //in cm
	double distancePerDegree = distancePerTurn/360; //in cm

	double dleftsum =0;
	double drightsum =0;
	double dleftalt =0;
	double drightalt =0;

	
	
	float olddestx=0;
	float olddesty=0;
	float olddestphi=0;
	float xvect;
	Point idealpose=null;
	float olddistx1=0;
    float yvect;
	double starttime=0;
	float oldtime=0;
	float startx=0;
	float starty=0;
	float length=0;
	
	//Trajektorie-einparken
	double a0=0;
	double a1=0;
	double a2=0;
	double a3=0;
	double b0=0;
	double b1=0;
	double b2=0;
	double b3=0;
	double currenttime=0;
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
		  case VW_CTRL		: update_VWCTRL_Parameter();
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
//		this.encoderLeft  = perception.getControlLeftEncoder();
//		this.encoderRight = perception.getControlRightEncoder();
		left_encoder=encoderLeft.getEncoderMeasurement();
		right_encoder=encoderRight.getEncoderMeasurement();
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
		setPose(navigation.getPose());
		currenttime=(double)System.currentTimeMillis()/1000;
		left_encoder=encoderLeft.getEncoderMeasurement();
		right_encoder=encoderRight.getEncoderMeasurement();
		
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
    	 
    	//float currentx=currentPosition.getX();
    	//float currenty=currentPosition.getY();
    	//float currentphi=currentPosition.getHeading();
    	
    	//float newdestx=destination.getX();
    	//float newdesty=destination.getY();
    	//float newdestphi=destination.getHeading();
    	
    	
    	
    	float distx1=0;
    	float idealposex=0;
    	float idealposey=0;
    	float t=0;  //Parameter für Streckenfortschritt
    	double w=0;   //Winkelgeschwindigkeit
    	//float currentidealposex=0;
    	//float currentidealposey=0;
    	double kp=1;
    	double kd=0.1;
    	
    	
    	
    	if((destination.getX()!=olddestx) ||(destination.getY()!= olddesty)){     //Neue Startposition
    		starttime=System.currentTimeMillis()/1000;//    Startzeit
    		startPosition.setHeading(currentPosition.getHeading());
    		startPosition.setLocation(currentPosition.getX(), currentPosition.getY());
    		//startx=currentPosition.getX();
    		//starty=currentPosition.getY();
    		xvect=(destination.getX()-currentPosition.getX())*100;   //in cm
    		yvect=(destination.getY()-currentPosition.getY())*100;   //in cm
    		length=(float) Math.sqrt(xvect*xvect+yvect*yvect);   //Länge der Wegstrecke in cm
    		
    				
    				}
    
    	
    if(currentPosition.relativeBearing(destination.getLocation())<(-5)){
    	
    	drive(0,0.5);   //drehen mit  ca 30°/sec
    	}
    else if(currentPosition.relativeBearing(destination.getLocation())<(5)){
    	
    	drive(0,-0.5);
    	}
  
    else{
    	//Ausrichtung erreicht
    	t=(float)(20*(System.currentTimeMillis()/1000-starttime)/length);  //20standartspeed
    	idealposex=startPosition.getX()*100+(t*xvect);  //in cm
    	idealposey=startPosition.getY()*100+(t*yvect); //in cm
    	
    	idealpose.setLocation(idealposex/100, idealposey/100);  //muss noch Winkel in Poseobjekt geschrieben werden?!!!???!!!!!!
    	distx1=currentPosition.distanceTo(idealpose)*100;   //Abstand zur ideallinie in cm
    	
    	w=ki*distx1+kd*((distx1-olddistx1))/((System.currentTimeMillis()/1000-oldtime));
    			
    	olddistx1=distx1;
    	oldtime=System.currentTimeMillis()/1000;
    	
    	drive(20,w);
    	}
    	
    		
    
    	//currentidealposex=;
    	//currentidealposey=;
    	//idealpose.setLocation(currentidealposex, currentidealposey);
    	
    	
    	olddestx=destination.getX()/100; //in cm
    	olddesty=destination.getY()/100;  //in cm
    	
	}
	
    
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
		double T=8; //dauer der einparkzeit in s 
		double vstart=0.7; 
		double vend=0.5;   
		double theta_i=0;
		double theta_f=0;
		double t=0;
		double dots=0;
		double s=0;
		double x1=0;
		double dx1=0;
		double ddx1=0;
		double x2=0;
		double dx2=0;
		double ddx2=0;
		double v=0;
		double w=0;
		
	//	X;
		
		if((destination.getX()!=olddestx) ||(destination.getY()!= olddesty)){     //Neue Startposition ->koeffizientenberechnung alles in cm und s
//    		Sound.beep();
			starttime=currenttime;//    Startzeit in s
			theta_i=currentPosition.getHeading();  //in °
    		theta_f=destination.getHeading();  //in °			
//			theta_i=currentPosition.getHeading()*180/Math.PI;  //in °
//    		theta_f=destination.getHeading()*180/Math.PI;  //in °
    		a0 = currentPosition.getX()*1;
    		a1 = vstart*Math.cos(theta_i);
    		a2 = 3*(destination.getX()-currentPosition.getX())*1 - vend*Math.cos(destination.getHeading());
    		a3 = 2*(currentPosition.getX()-destination.getX())*1 + vend*Math.cos(destination.getHeading());
//    		a2 = 3*(destination.getX()-currentPosition.getX())*1 - 2*a1 - vend*Math.cos(destination.getHeading());
//    		a3 = 2*(currentPosition.getX()-destination.getX())*1 + a1 + vend*Math.cos(destination.getHeading());


    		b0 = currentPosition.getY()*1;
    		b1 = vstart*Math.sin(theta_i);
    		b2 = 3*(destination.getY()-currentPosition.getY())*1 - vend*Math.sin(destination.getHeading());
    		b3 = 2*(currentPosition.getY()-destination.getY())*1 + vend*Math.sin(destination.getHeading());
//    		b2 = 3*(destination.getY()-currentPosition.getY())*1 - 2*b1 - vend*Math.sin(destination.getHeading());
//    		b3 = 2*(currentPosition.getY()-destination.getY())*1 + b1 + vend*Math.sin(destination.getHeading());
		
    		olddestx=destination.getX();
    		olddesty=destination.getY();
		}
		
//		a0=a0*1;
//		a1=a1*1;
//		a2=a2*1;
//		a3=a3*1;
//		b0=b0*1;
//		b1=b1*1;
//		b2=b2*1;
//		b3=b3*1;


		
		//t = linspace(0,T,500);
		t=(System.currentTimeMillis()/1000)-starttime;
		
		s = (t*t)/(T*T)*(3-2*t/T);
//		s = (t*t)/(T*T)*(3-2*t/T);
		dots = -6*t/(T*T)*(1 - t/T);
//		dots = 6*t/(T*T)*(1 - t/T);

		
		x1 = a0 + a1*s + a2*s*s + a3*s*s*s;
		dx1 = a1 + 2*a2*s + 3*a3*s*s;
		ddx1 = 2*a2 + 6*a3*s;
		x2 = b0 + b1*s + b2*s*s + b3*s*s*s;
		dx2 = b1 + 2*b2*s + 3*b3*s*s;
		ddx2 = 2*b2 + 6*b3*s;
		

//		olddestx=destination.getX();
//		olddesty=destination.getY();
		if (t<T){
		v = dots*(Math.sqrt(dx1*dx1 + dx2*dx2))*100; // Geschwindigkeit
//		w = dots*(dx2*ddx1 - ddx2*dx1)/(dx1*dx1 + dx2*dx2);
		w = dots*(ddx2*dx1 - dx2*ddx1)/(dx1*dx1 + dx2*dx2);
		} 
//		else 
//			if 	(currentPosition.getHeading()!=theta_f){
//				double current_heading = currentPosition.getHeading();
//				if current_heading>270){
//					current_heading = current_heading - Math.PI;
//				}
//			if (currentPosition.getHeading()-theta_f<0.05*Math.PI){
//					w=2;
//				} else if (currentPosition.getHeading()-theta_f>0.05*Math.PI){
//					w=-2;
//				} else {
//					w=0;
//				}
//			v=0;
//		} 
	else {
			v=0;
			w=0;
		}
		drive(v,w);
		
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
			leftMotor.setPower(highPower);
			rightMotor.setPower(highPower);
		}
	}
	
	
	//für 3.1.3 methode mit PID-Regler
	private void exec_LINECTRL_ALGO_PID() {  
		int speedconst = 0;
		leftMotor.forward();
		rightMotor.forward();

		e = (this.lineSensorRight - this.lineSensorLeft);  //(double)
		
		if(e > 80){                       //Linkskurve
			leftMotor.setPower(-20);
			rightMotor.setPower(30);
			esum =0;
			ealt=0;
			
		}
		else if (e < -80 ){   //-90, Rechtskuve
			leftMotor.setPower(30);
			rightMotor.setPower(-20);
			esum =0;
			ealt=0;
		}
		else{     //Geradeausfahrt mit PIDRegler
		
			esum = esum + (double)(e); //integrationsanteil
			y = kp*(double)(e) + ki*esum + kd*((double)(e) - ealt);
			ealt = (double)(e);
			
			//if(currentPosition.getX())
			
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
		double radius = 0;   //für methode drive(V;W-Control)
		double rightSpeed = 0;
		double leftSpeed = 0;
		double dleft = 0;
		double dright = 0;
		double yleft =0;
		double yright =0;
		double KP1 =0.2;    //für v-w-control
		double KI1 =0.016;//für v-w-control
		double KD1 =0.0001;//für v-w-control
		double MOTORKONSTRIGHT=2.5806452+0.087;//für Power/speed-verhältnis
		double MOTORKONSTLEFT=2.272727272+0.15;//für Power/Speed-Verhältnis


				
		if( omega != 0){                   //wenn Winkelgeschwindigkeit gegeben, in rad/s
			radius = v/omega;         //Radius in cm, v in cm/s
			if(v != 0){
				leftSpeed = v - (trackWidth*v/radius/2);
				rightSpeed = v + (trackWidth*v/radius/2);
			}
			else{                            //falls v=0
				leftSpeed = trackWidth*omega/2;
				rightSpeed = -trackWidth*omega/2;
			};
				
		}
		else{                                      
			leftSpeed = v;
			rightSpeed = v;
		};
		
		double left_length=left_encoder.getAngleSum()*1000*distancePerDegree;
		double right_length=right_encoder.getAngleSum()*1000*distancePerDegree;
		double left_deltat=left_encoder.getDeltaT();
		double right_deltat=right_encoder.getDeltaT();
		double left_rspeed=0;
		double right_rspeed=0;
		if(left_deltat==0){
			left_rspeed=0;
		}
		else{
			left_rspeed=left_length/left_deltat;
		}
		if(right_deltat==0){
			right_rspeed=0;
		}
		else{
			right_rspeed=right_length/right_deltat;
		}
		
		//PIDRegler für jedes Rad
		dleft = leftSpeed - (left_rspeed);                //Fehler des linken Rades
		dright = rightSpeed - (right_rspeed);			  //Fehler der rechten Rades
		
		dleftsum = dleftsum + dleft; //integrationsanteil
		yleft = KP1*dleft + KI1*dleftsum + KD1*(dleft - dleftalt);
		dleftalt = dleft;
		
		drightsum = drightsum + dright; //integrationsanteil
		yright = KP1*dright + KI1*drightsum + KD1*(dright - drightalt);
		drightalt = dright;
		
		if(drightsum > 1000){
			drightsum=0;
		}
		if(dleftsum > 1000){
			dleftsum=0;
		}
		
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.setPower((Math.round((float)((leftSpeed+yleft)*MOTORKONSTLEFT))));   //berechneter Powerwert*Reglerausgang mit Power-Speed-Verhältnis
		rightMotor.setPower((Math.round((float)((rightSpeed+yright)*MOTORKONSTRIGHT))));
	
		
	}
	
}