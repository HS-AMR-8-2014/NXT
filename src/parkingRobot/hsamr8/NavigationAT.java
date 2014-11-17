package parkingRobot.hsamr8;

import lejos.geom.Line;
import lejos.robotics.navigation.Pose;
import lejos.geom.Point;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.hsamr8.NavigationThread;


/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
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
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;


	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	0.028; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.114; // only rough guess, to be measured exactly and maybe refined by experiments

	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	Line[] map 								= null;
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();
    int id_aktuell =0;
    boolean x_fix=false;
    boolean y_fix=true;
    int fix_value=0;
	ParkingSlot[] list_ParkingSlot;
	int abstand_sens_band=130; //Abstand von Sensor zur Bande

	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 */
	public NavigationAT(IPerception perception){
		this.perception   = perception;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo = perception.getNavigationOdo();		
		
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = map;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
	}
	
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		return null;
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance = perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx 		= 0;
		double ICCy 		= 0;

		double xResult 		= 0;
		double yResult 		= 0;
		double angleResult 	= 0;
		int abstandtriang=50;
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		if (R.isNaN()) { //robot don't move
			if(x_fix==false){xResult= this.pose.getX();}
			else{xResult=fix_value;}
			if(y_fix==false){
				yResult			= this.pose.getY();
			}else{
				yResult=fix_value; 
			}
			angleResult 	= this.pose.getHeading();
			if(angleResult>360){
				angleResult=angleResult-360;
			}
		} else if (R.isInfinite()||(this.backSideSensorDistance==this.frontSideSensorDistance )) { //robot moves straight forward/backward, vLeft==vRight
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
				angleResult 	= this.pose.getHeading();
				if(angleResult>360){
					angleResult=angleResult-360;
				}
			
			
		} else {			
			
			if(this.frontSideSensorDistance<60 &&  this.backSideSensorDistance<60){
				ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
				ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
				xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
				yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
				angleResult 	= this.pose.getHeading() + w * deltaT;
				if(angleResult>360){
					angleResult=angleResult-360;
				}
				
			}
			
			else {
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
		/**	angleResult 	= this.pose.getHeading() + w * deltaT;    */
			angleResult     = this.pose.getHeading()+ Math.sin((this.frontSideSensorDistance-this.backSideSensorDistance)/abstandtriang);
			if(angleResult>360){
				angleResult=angleResult-360;
			}
		}
		}
		
		/**		if(this.lineSensorLeft==2 || this.lineSensorRight==2){
			int links=this.lineSensorLeft;
			int rechts=this.lineSensorRight;
		}
			switch(links){
	        case 2:
	            if(rechts==0){
	            	angleResult=
	            };
	            break;
	        case 0:
	            if(rechts==2){
	            	
	            };
	            break;
	        	
            switch(rechts){
		        case 2:
		            if(links==0){
		            	
		            };
		            break;
		        case 0:
		            if(links==2){
		            	
		            };
		            break;
            
            }
		} */
			
			
			
		this.pose.setLocation((float)xResult, (float)yResult);
		this.pose.setHeading((float)angleResult);		 
	}

	/**
	 * Wird von Guidance aufgerufen, um fixe Werte zu übergeben. Dies vereinfacht Schätzung der Pose.
	 * 
	 * @param a ist der x-Wert fix?
	 * @param b ist der y-Wert fix?
	 * @param c fixer Wert dessen Berechnung eingespart werden kann
	 */
	public void update_nav_line(boolean a, boolean b, int c){
	x_fix=a;
	y_fix=b;
	fix_value=c;
		
	}
	
	
	
	
	
	
	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
//		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
//		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds
	//	double firstcheckx=	0;
//		double secondcheckx=0;
//		double xweg 		= 0;
//		double yweg 		= 0;
//		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
//		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s	
//		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		boolean detectionactive = false;
		Point anfang =new Point(0,0);
		Point ende =new Point(0,0);
		boolean bekannt=false;
		int id_bekannte_luecke=0;
		
		
		ParkingSlotStatus slotStatus=ParkingSlotStatus.BAD;
		
		if(this.frontSideSensorDistance>abstand_sens_band || this.backSideSensorDistance>abstand_sens_band){
			
			
	 		if(this.frontSideSensorDistance>abstand_sens_band && (detectionactive==false)){
	 		
	 		//pruefe, ob Parkluecke schon vorhanden
	 		for(int i=0; i==list_ParkingSlot.length;i++){	
	 		double	deltax=this.pose.getX()-list_ParkingSlot[i].getFrontBoundaryPosition().getX();
	 		double	deltay=this.pose.getY()-list_ParkingSlot[i].getFrontBoundaryPosition().getY();
	 			if ((deltax<5) && (deltay<5)){
	 			bekannt=true;
	 			id_bekannte_luecke=i;
	 			//Parklücken später direkt neu vermessen an dieser Stelle und mit alten Daten abgleichen
	 			
	 			}else{
	 				anfang.setLocation(this.pose.getX(),this.pose.getY());
	 				detectionactive=true;
	 				
	 				//Lücke vermessen und Speichern
	 			}
	 		}
	 		
	 		
	 		
	 		 
	 		if(this.frontSideSensorDistance<abstand_sens_band && detectionactive==true){
	 			ende.setLocation(this.pose.getX(),this.pose.getY());
			detectionactive=false;
	 		//Parklückenbewertung
			
			
			list_ParkingSlot [id_aktuell] = new ParkingSlot(id_aktuell, anfang, ende, slotStatus);
			
			id_aktuell++;
	 		}
	 		
	 
	 		}	

		}else{
			
		}
			
		
		//muss abgesichert werden, dass nur positiv/gut vermessene Parklücken abgespeichert werden


	
		return ; // has to be implemented by students
	}
	public ParkingSlot getSlotById(int i){
		return list_ParkingSlot[i];
	}
	
	
}
