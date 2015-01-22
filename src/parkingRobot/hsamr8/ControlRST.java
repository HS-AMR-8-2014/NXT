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
	
	/**
	 * encoder objects that hold the encoder measurements
	 */
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
	Pose interncurrentPosition=new Pose();
	Pose idealPose=new Pose();
	
	ControlMode currentCTRLMODE = null ;

	int lastTime = 0;
	
    double currentDistance = 0.0;
    double Distance = 0.0;
    
    /**
     * variables für Line-Follow
     */
    double esum = 0; //für linefollow
	double e = 0;
	double ealt = 0;
	double y = 0;
	
	

	double wheelDiameter = 5.6;  //in cm
	double trackWidth = 14;   //in cm
	double distancePerTurn = Math.PI*wheelDiameter; //in cm
	double distancePerDegree = distancePerTurn/360; //in cm
	/**
	 * variables for v/w-control
	 */
	double dleftsum =0;
	double drightsum =0;
	double dleftalt =0;
	double drightalt =0;
	double left_rspeed=0;
	double right_rspeed=0;

	
	/**
	 * variables for Setpose
	 */
	double olddestx=0;
	double olddesty=0;
	double olddestphi=0;
	double xvect;
	Point idealpose=null;
	double olddistx1=0;
	double yvect;
	double starttime=0;
	double oldtime=0;
	double startx=0;
	double starty=0;
	double endx=0;
	double endy=0;
	double length=0;
	double oldangle=0;
	
	/**
	 * Variables for Park-Ctrl
	 */
	double a0=0;
	double a1=0;
	double a2=0;
	double a3=0;
	double b0=0;
	double b1=0;
	double b2=0;
	double b3=0;
	double currenttime=0;
	private boolean destination_reached;
	private boolean drive_backwards;
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
		   					  exec_VWCTRL_ALGO_V2();
		   					  break; 
		  case SETPOSE      : update_SETPOSE_Parameter();
			  				  exec_SETPOSE_ALGO_V2();
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
	 * current Position; right, left Encoder
	 */
	private void update_VWCTRL_Parameter(){
		setPose(navigation.getPose());
		left_encoder=encoderLeft.getEncoderMeasurement();
		right_encoder=encoderRight.getEncoderMeasurement();
	}
	
	
	/**
	 * update parameters during SETPOSE Control Mode
	 * current Position; time; right, left Encoder
	 */
	private void update_SETPOSE_Parameter(){
		setPose(navigation.getPose());
		currenttime=(double)System.currentTimeMillis()/1000;
		left_encoder=encoderLeft.getEncoderMeasurement();
		right_encoder=encoderRight.getEncoderMeasurement();
	}
	
	
	/**
	 * update parameters during PARKING Control Mode
	 * current Position; time; right, left Encoder
	 */
	private void update_PARKCTRL_Parameter(){
		//Aufgabe 3.4
		setPose(navigation.getPose());
		currenttime=(double)System.currentTimeMillis()/1000;
		left_encoder=encoderLeft.getEncoderMeasurement();
		right_encoder=encoderRight.getEncoderMeasurement();
	}

	
//	/**
//	 * update parameters during LINE Control Mode
//	 */
//	private void update_LINECTRL_Parameter(){
//		this.lineSensorRight		= perception.getRightLineSensor();
//		this.lineSensorLeft  		= perception.getLeftLineSensor();		
//	}

	//für 3.1.3 liniensensordaten von 0 bis 100
	/**
	 * Update for parameter: left, right line sensor and current position
	 */
    private void update_LINECTRL_Parameter(){
		this.lineSensorRight		= perception.getRightLineSensorValue();
		this.lineSensorLeft  		= perception.getLeftLineSensorValue();	
		setPose(navigation.getPose());
	}
	
	
	/**
	 * The car can be driven with velocity in m/s or angular velocity in grade during VW Control Mode
	 * optionally one of them could be set to zero for simple test.
	 */
    private void exec_VWCTRL_ALGO(){  
		this.drive(this.velocity, this.angularVelocity); 
		}
    
    
    /**
     * execute method for reworked v/w-control
     */
    private void exec_VWCTRL_ALGO_V2(){  
		this.drive_V2(this.velocity, this.angularVelocity); 
		}
	
    
    /**
     * method to lead the robot to a destination given
     * works with the distance to the ideal line
     */
    private void exec_SETPOSE_ALGO_V1(){
   	 //Aufgabe 3.3 
   	double v=10;
   	double w=0;   //Winkelgeschwindigkeit
   	double kp=4;   //auf 4 lassen
   	double kd=0.0000001;
   	double x=0;
   	double y=0;
   	double angle=0;
   	double destx=0;
   	double desty=0;
   	double destangle=0;
   	double rel_angle=0;
   	x=currentPosition.getX();
   	y=currentPosition.getY();
   	angle=currentPosition.getHeading()*180/Math.PI;  //in °
   	destx=destination.getX();
   	desty=destination.getY();
   	destangle=destination.getHeading()*180/Math.PI;
   	interncurrentPosition.setLocation((float)x, (float)y);
   	interncurrentPosition.setHeading(currentPosition.getHeading()*180/(float)Math.PI);     //neue aktuelle Position mit Heading in °
   	
   	if((Math.abs(destx-x)>0.05)||(Math.abs(desty-y)>0.05)){
   		if(interncurrentPosition.relativeBearing(destination.getLocation())<(-10)){
			Sound.beep();
   			w=0.7;
			v=0;
			oldangle=0;
		}
		else if(interncurrentPosition.relativeBearing(destination.getLocation())>10){
			w=-0.7;
			v=0;
			oldangle=0;
		}
		else{
			rel_angle=interncurrentPosition.relativeBearing(destination.getLocation());
			w=kp*rel_angle+kd*(rel_angle-oldangle);
   			w=w*Math.PI/180;
   			oldangle=rel_angle;
		}
   	}
   	else if((Math.abs(x-destx)<0.05)||(Math.abs(y-desty)<0.05)){
   		if((Math.abs(destangle-angle)<5)||(Math.abs(destangle-angle)>355)){
   			Sound.buzz();
   			v=0;
   			w=0;
   		}
   		else if((angle<180)&&((angle-destangle)>5)){
			w=-0.6;
			v=0;
		}
		else if((angle>180)&&(angle-destangle)>340){
			w=0.6;
			v=0;
		}
		else{
			w=0;
			v=0;
		
		}
   	}
   	else{
   		v=0;
   		w=0;
   		
   	}
   	
   	drive(v,w);
   	}

    
    /**
     * method to lead the robot to a destination given
     * works with the angle to the destination
     */
    private void exec_SETPOSE_ALGO_V2(){
   	 //Aufgabe 3.3 mit querabweichung
   	double x;
   	double y;
    double distx1=0;
   	double idealposex=0;
   	double idealposey=0;
   	double t=0;  //Parameter für Streckenfortschritt
   	double v=15;
   	double w=0;   //Winkelgeschwindigkeit
   	double kp=0.1;    //laut Matllab größer 0.4
   	double kd=0.0000000001;
   	
   	int direction=0;
   	
   	interncurrentPosition.setLocation(currentPosition.getX(), currentPosition.getY());
   	interncurrentPosition.setHeading(currentPosition.getHeading()*180/(float)Math.PI);     //neue aktuelle Position mit Heading in °
   	x=currentPosition.getX();
   	y=currentPosition.getY();
   	
   if(interncurrentPosition.relativeBearing(destination.getLocation())<(-5)){
   	v=0;
   	w=0.7;//drehen mit  ca 30°/sec
   	}
   else if(interncurrentPosition.relativeBearing(destination.getLocation())>(5)){
   	v=0;
   	w=-0.7;
   	}
   else{                //Ausrichtung erreicht
   	
   	if((destination.getX()!=olddestx) ||(destination.getY()!= olddesty)){     //Neue Startposition
   		startx=currentPosition.getX();
   		starty=currentPosition.getY();
   		endx=destination.getX();
   		endy=destination.getY();
   		xvect=endx-startx;   //in m
   		yvect=endy-starty;   //in m
   		length=(float) Math.sqrt(xvect*xvect+yvect*yvect);   //Länge der Wegstrecke in m
       	olddestx=destination.getX(); //in m
       	olddesty=destination.getY();  //in m		
   		}
   	
       t=(Math.sqrt(xvect*(x-startx)+yvect*(y-starty)))/length; 
       idealposex=startx*+(t*xvect);  //in m
       idealposey=starty*+(t*yvect); //in m

       distx1=Math.sqrt((x-idealposex)*(x-idealposex)+(y-idealposey)*(y-idealposey));
   	
       //Abfrage ob distx1 positiv oder negativ
       if(endx>startx && endy>starty){
       	direction=1;
       	if(currentPosition.getX()>idealposex && currentPosition.getY()<idealposey){
       		distx1=-distx1;
       	}
       }
       else if(endx<startx && endy>starty){
       	direction=2;
       	if(currentPosition.getX()>idealposex && currentPosition.getY()>idealposey){
       		distx1=-distx1;
       	}
       }
       else if(endx<startx && endy<starty){
       	direction=3;
       	if(currentPosition.getX()<idealposex && currentPosition.getY()>idealposey){
       		distx1=-distx1;
       	}
       }
       else if(endx>startx && endy<starty){
       	direction=4;
       	if(currentPosition.getX()<idealposex && currentPosition.getY()<idealposey){
       		distx1=-distx1;
       	}
       }
       else if(endx==startx && endy>starty){
       	direction=5;
       	if(currentPosition.getX()>idealposex){
       		distx1=-distx1;
       	}
       }
       else if(endx<startx && endy==starty){
       	direction=6;
       	if(currentPosition.getY()>idealposey){
       		distx1=-distx1;
       	}
       }
       else if(endx==startx && endy<starty){
       	direction=7;
       	if(currentPosition.getX()<idealposex){
       		distx1=-distx1;
       	}
       }
       else if(endx>startx && endy==starty){
       	direction=8;
       	if(currentPosition.getY()<idealposey){
       		distx1=-distx1;
       		Sound.buzz();
       	}
       }
       else{
       	direction=0;
       	
       }
       
       
   	w=kp*distx1+kd*(distx1-olddistx1);   //formel wie im matlabskript
   	w=-w;    //*Math.PI/180

   	olddistx1=distx1;

   	
//   	if(t>=1){            //Ziel theoretisch erreicht
//   		w=0;
//   		v=0;
//   	}
   	
   	}
   if(Math.sqrt((x-destination.getX())*(x-destination.getX())+(y-destination.getY())*(y-destination.getY()))<0.05){
	   destination_reached=true;
	   double currentangle=currentPosition.getHeading();
	   if((currentangle-destination.getHeading())*180/Math.PI>3){
		   w=-0.5;
		   v=0;
	   }
	   else if((destination.getHeading()-currentangle)*180/Math.PI<(-357)){
		   w=0.5;
		   v=0;
	   }
	   else{
		   w=0;
		   v=0;
	   }
   }
   
   drive(v,w);
	}

    
	/**
	 * PARKING along the generated path
	 */
	private void exec_PARKCTRL_ALGO(){
		//Aufgabe 3.4
		double T=5; //dauer der einparkzeit in s 
		double vstart = -0.7; 
		double vend = -0.7;     
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
		//Reglerparameter
		double kp_v=0.4;
		double kp_w=0.005;
		double delta_v=0;
		double delta_w=0;
		
		if((destination.getX()!=olddestx) ||(destination.getY()!= olddesty)){     //Neue Startposition ->koeffizientenberechnung alles in cm und s
    		Sound.beep();
			starttime=currenttime;//    Startzeit in s
			theta_i=currentPosition.getHeading();
			theta_f=destination.getHeading();    		
    		a0 = currentPosition.getX()*1;
    		a1 = vstart*Math.cos(theta_i);
    		a2 = 3*(destination.getX()-currentPosition.getX())*1 - 2*a1 - vend*Math.cos(destination.getHeading());
    		a3 = 2*(currentPosition.getX()-destination.getX())*1 + a1 + vend*Math.cos(destination.getHeading());


    		b0 = currentPosition.getY()*1;
    		b1 = vstart*Math.sin(theta_i);
    		b2 = 3*(destination.getY()-currentPosition.getY())*1 - 2*b1 - vend*Math.sin(destination.getHeading());
    		b3 = 2*(currentPosition.getY()-destination.getY())*1 + b1 + vend*Math.sin(destination.getHeading());
		
    		olddestx=destination.getX();
    		olddesty=destination.getY();
    		destination_reached = false;
		}

		
		//t = linspace(0,T,500);
		t=(System.currentTimeMillis()/1000)-starttime;
		
		s = (t*t)/(T*T)*(3-2*t/T);
		dots = 6*t/(T*T)*(1 - t/T);
		
		x1 = a0 + a1*s + a2*s*s + a3*s*s*s;
		dx1 = a1 + 2*a2*s + 3*a3*s*s;
		ddx1 = 2*a2 + 6*a3*s;
		x2 = b0 + b1*s + b2*s*s + b3*s*s*s;
		dx2 = b1 + 2*b2*s + 3*b3*s*s;
		ddx2 = 2*b2 + 6*b3*s;

		
		
		
		if (t<T){
			v = dots*(Math.sqrt(dx1*dx1 + dx2*dx2))*100; // Geschwindigkeit
			w = dots*(ddx2*dx1 - dx2*ddx1)/(dx1*dx1 + dx2*dx2);
			if (drive_backwards){
				v = -1*v; // drive backwards
			
			//Regler
			idealPose.setLocation((float)x1, (float)x2);
//			idealPose.setHeading(heading);
			delta_v=kp_v* currentPosition.distanceTo(idealPose.getLocation());
			delta_w=kp_w*currentPosition.relativeBearing(idealPose.getLocation());
			}
		} else {
			//catch special case: phi_dest = 0 -> phi_current might be higher than 180°
			if ((Math.abs(theta_f)<Math.PI*5/180)&&(currentPosition.getHeading()>Math.PI)){
				 theta_f = Math.PI*2;
				 }
			double delta_phi = currentPosition.getHeading()-theta_f;
			if 	(Math.abs(delta_phi)>Math.PI*4/180){
				if (delta_phi<0){
					w=-0.7;
				} else if (delta_phi>0){
					w=+0.7;
				} 
			v=0;
			} 
			else {
			v=0;
			w=0;
			destination_reached=true;
			}
		}
		drive(v+delta_v,w+delta_w);
		
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
	/**
	 * line-follow with PID control
	 */
	private void exec_LINECTRL_ALGO_PID() {  
		double kp = 0.387;
		double ki =0.00001;//0.0042; 1.Einstellung//0.0025
		double kd = 0.0000001;//0.00008; 2.Einstellung//0.000001
		int speedconst = 38;
		leftMotor.forward();
		rightMotor.forward();

		e = this.lineSensorRight - this.lineSensorLeft;  //(double)
		
		double x=currentPosition.getX();
		double y=currentPosition.getY();
		if((x>1.7 && x<1.85 && y>-0.05 && y<0.05)||(y>0.5 && y<0.65 && x>1.75 && x<1.85)||(x<1.6 && x>1.45 && y>0.55 && y<0.65)||(y<0.4 && y>0.25 && x>1.45 && x<1.55)||(x<0.16 && x>0.145 && y>0.25 && y<0.35)||(y>0.5 && y<0.65 && x>0.25 && x<0.35)||(x<0.1 && x>-0.05 && y<0.65 && y>0.55)||(y<0.1 && y>-0.05 && x>-0.05 && x<0.05)){   //10cm vor Kurve konst Geschw verlangsamen
			speedconst=25;
		}
		
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
			rightMotor.setPower(speedconst+(int)(y));
			leftMotor.setPower(speedconst-(int)(y));
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
		
		
		leftMotor.setPower((Math.round((float)((leftSpeed+yleft)*MOTORKONSTLEFT))));   //berechneter Powerwert*Reglerausgang mit Power-Speed-Verhältnis
		rightMotor.setPower((Math.round((float)((rightSpeed+yright)*MOTORKONSTRIGHT))));
		leftMotor.forward();
		rightMotor.forward();
		
	}
    
	
	//Aufgabe 3.2
	/**
     * reworked drive method
     * @param v in cm/s velocity of the robot
     * @param omega in rad/s angle velocity of the robot
     */
	private void drive_V2(double v, double omega){

		double radius = 0;   //für methode drive(V;W-Control)
		double rightSpeed = 0;
		double leftSpeed = 0;
		double dleft = 0;
		double dright = 0;
		double yleft =0;
		double yright =0;
		double KPl1 =2.39;    //für v-w-control
		double KIl1 =0.57;//für v-w-control
		double KDl1 =0.0004921;//für v-w-control
		double KPr1 =2.16;    //für v-w-control
		double KIr1 =0.5;//für v-w-control
		double KDr1 =0.000498;//für v-w-control
		double left_length=left_encoder.getAngleSum()*1000*distancePerDegree;
		double right_length=right_encoder.getAngleSum()*1000*distancePerDegree;
		double left_deltat=left_encoder.getDeltaT();
		double right_deltat=right_encoder.getDeltaT();

		
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

		
		//PIDRegler für jedes Rad
		dleft = leftSpeed - left_rspeed;                //Fehler des linken Rades
		dright = rightSpeed - right_rspeed;			  //Fehler der rechten Rades
		
		dleftsum = dleftsum + dleft; //integrationsanteil
		yleft = KPl1*dleft + KIl1*dleftsum + KDl1*(dleft - dleftalt);
		dleftalt = dleft;
		
		drightsum = drightsum + dright; //integrationsanteil
		yright = KPr1*dright + KIr1*drightsum + KDr1*(dright - drightalt);
		drightalt = dright;
		
		if(drightsum > 1000){
			drightsum=0;
		}
		if(dleftsum > 1000){
			dleftsum=0;
		}
		
		leftMotor.setPower((Math.round((float)yleft)));  
		rightMotor.setPower((Math.round((float)yright)));
		leftMotor.forward();
		rightMotor.forward();
	}
	
	
	/**
	 * method to give back the state of destination reached
	 */
	public boolean destination_reached(){
		return destination_reached;
	}
	
	
	/**
	 * method to back out of a parking space
	 * @param isON activates the state of driving backwards
	 */
	public void drive_backwards(boolean isOn)
		{ 
		drive_backwards = isOn;
		starttime=(double)System.currentTimeMillis()/1000;
		destination_reached = false;
		Sound.beep();
		dleftsum=0;
		drightsum=0;
		}
	
	
	/**
	 * test method for datalogger to get the actual velocity of the left wheel
	 */
	public double get_rvelocity(){
		return left_rspeed;
	}
	/**
	 * test method for datalogger to get the actual velocity of the left wheel
	 */
	public double get_lvelocity(){
		return right_rspeed;
	}
	
