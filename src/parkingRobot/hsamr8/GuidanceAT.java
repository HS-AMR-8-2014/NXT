package parkingRobot.hsamr8;

import lejos.nxt.Button;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.Sound;
import parkingRobot.IControl;
import parkingRobot.IControl.*;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INxtHmi;
import parkingRobot.INavigation;
import parkingRobot.IPerception;
import lejos.robotics.navigation.Pose;
import lejos.geom.Line;
import lejos.nxt.LCD;
import parkingRobot.hsamr8.ControlRST;
import parkingRobot.hsamr8.HmiPLT;
import parkingRobot.hsamr8.NavigationAT;
import parkingRobot.hsamr8.PerceptionPMP;
//import parkingRobot.IPerception.AngleDifferenceMeasurement;
//import parkingRobot.IPerception.EncoderSensor;

/**
 * Main class for 'Hauptseminar AMR' project 'autonomous parking' for students of electrical engineering
 * with specialization 'automation, measurement and control'.
 * <p>
 * Task of the robotic project is to develop an mobile robot based on the Lego NXT system witch can perform
 * parking maneuvers on an predefined course. To fulfill the interdisciplinary aspect of this project the software
 * structure is divided in 5 parts: human machine interface, guidance, control, perception and navigation.
 * <p>
 * Guidance is to be realized in this main class. The course of actions is to be controlled by one or more finite
 * state machines (FSM). It may be advantageous to nest more than one FSM.
 * <p>
 * For the other parts there are interfaces defined and every part has to be realized in one main module class.
 * Every class (except guidance) has additionally to start its own thread for background computation.
 * <p>
 * It is important that data witch is accessed by more than one main module class thread is only handled in a
 * synchronized context to avoid inconsistent or corrupt data!
 */
public class GuidanceAT {
	
	/**
	 * states for the main finite state machine. This main states are requirements because they invoke different
	 * display modes in the human machine interface.
	 */
	public enum CurrentStatus {
		/**
		 * indicates that robot is following the line and maybe detecting parking slots
		 */
		DRIVING,
		/**
		 * indicates that robot is paused
		 */
		INACTIVE,
		/**
		 * indicates that robot is performing a parking maneuver with determined parking slot
		 */
		PARK_THIS,
		/**
		 * indicates that robot is perfroming a parking maneuver as soon as possible
		 */
		PARK_NOW,
		/**
		 * indicates that shutdown of main program has initiated
		 */
		EXIT		
	}
	
	
	/**
	 * state in which the main finite state machine is running at the moment
	 */
	protected static CurrentStatus currentStatus 	= CurrentStatus.INACTIVE;
	/**
	 * state in which the main finite state machine was running before entering the actual state
	 */
	protected static CurrentStatus lastStatus		= CurrentStatus.INACTIVE;

	
	/**
	 * Substates that are not supposed to be sent to the tablet, needed to realize submissions
	 *
	 */
	private enum CurrentSubStatus {
		/**
	 	 * SubStates for DRIVING, needed for collision avoidance
		 */
		FOLLOW_LINE,WAIT,DRIVE_BACKWARDS,FIND_BLACK_LINE,
		/**
		 * SubStates for INACTIVE
		 */
		WAITING_ON_LINE,WAITING_OFF_LINE,
		/**
		 * SubStates for PARK_THIS
		 */
		
		/**
		 * SubStates for PARK_NOW
		 */
		SEARCH,		
		/**
	 	 * SubStates for both Parking-Modes, the Robot moves to and enters a Parking Slot
		 */
		MOVE_TO_PARKING_POSITION,PARK,	
	}
	
	/**
	 * state in which the sub finite state machine is running at the moment
	 */
	protected static CurrentSubStatus currentSubStatus 	= CurrentSubStatus.WAITING_ON_LINE;
	/**
	 * state in which the sub finite state machine was running before entering the actual state
	 */
	protected static CurrentSubStatus lastSubStatus		= CurrentSubStatus.WAITING_ON_LINE;
	
	
	/**
	 * converts currentSubStatus to String. Needed to show current Status on display
	 * possibly only needed for testing
	 * @return
	 */
	private static String CurrentSubStatusAsString(){
		String currentSubStatusString = null;
		switch (currentSubStatus){
			case FOLLOW_LINE:
				currentSubStatusString = "FOLLOW_LINE";
				break;
			case WAIT:
				currentSubStatusString = "WAIT";
				break;
			case DRIVE_BACKWARDS:
				currentSubStatusString = "DRIVE_BACKWARDS";
				break;
			case FIND_BLACK_LINE:
				currentSubStatusString = "FIND_BLACK_LINE";
				break;
			case WAITING_ON_LINE:
				currentSubStatusString = "WAITING_ON_LINE";
				break;
			case WAITING_OFF_LINE:
				currentSubStatusString = "WAITING_OFF_LINE";
				break;
			case SEARCH:
				currentSubStatusString = "SEARCH";
				break;
			case MOVE_TO_PARKING_POSITION:
				currentSubStatusString = "MOVE_TO_PARKING_POSITION";
				break;
			case PARK:
				currentSubStatusString = "PARK";
				break;
			default:
				currentSubStatusString = "UNKNOWN";
				break;
 		}
		return currentSubStatusString;
	}
	
	/**
	 * one line of the map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * This documentation for line0 hold for all lines.
	 */
	static Line line0 = new Line(  0,  0, 180,  0);
	static Line line1 = new Line(180,  0, 180, 60);
	static Line line2 = new Line(180, 60, 150, 60);
	static Line line3 = new Line(150, 60, 150, 30);
	static Line line4 = new Line(150, 30,  30, 30);
	static Line line5 = new Line( 30, 30,  30, 60);
	static Line line6 = new Line( 30, 60,   0, 60);
	static Line line7 = new Line(  0, 60,   0,  0);
	/**
	 * map of the robot course. The course consists of a closed chain of straight lines.
	 * Thus every next line starts where the last line ends and the last line ends where the first line starts.
	 * All above defined lines are bundled in this array and to form the course map.
	 */
	static Line[] map = {line0, line1, line2, line3, line4, line5, line6, line7};

		
	/**
	 * main method of project 'ParkingRobot'
	 * 
	 * @param args standard string arguments for main method
	 * @throws Exception exception for thread management
	 */
	public static void main(String[] args) throws Exception {		
		
		//ORIGINAL STARTZUSTAENDE
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus    = CurrentStatus.EXIT;
				
//	//TestStates PARK NOW + SEARCH
//		currentStatus = CurrentStatus.PARK_NOW;
//		lastStatus    = CurrentStatus.PARK_NOW;
//        currentSubStatus = CurrentSubStatus.SEARCH;
//        lastSubStatus = CurrentSubStatus.FOLLOW_LINE;


//		//TestStates PARK NOW + MOVE_TO_PARKING_POSITION
//		currentStatus = CurrentStatus.PARK_NOW;
//		lastStatus    = CurrentStatus.PARK_NOW;
//        currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
//        lastSubStatus = CurrentSubStatus.SEARCH;

//		//TestStates PARK NOW + PARK
//		//also good for testing trajectory
//		currentStatus = CurrentStatus.PARK_NOW;
//		lastStatus    = CurrentStatus.PARK_NOW;
//        currentSubStatus = CurrentSubStatus.PARK;
//        lastSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;

//		//TestStates DRIVE BACKWARDS
//		currentStatus = CurrentStatus.DRIVING;
//		lastStatus    = CurrentStatus.DRIVING;
//		currentSubStatus = CurrentSubStatus.DRIVE_BACKWARDS;
//		lastSubStatus = CurrentSubStatus.WAIT;		

				
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor);
//		perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control);
		navigation.setMap(map);	

		
		while(true) {
			showData(navigation, perception, control);
			
        	switch ( currentStatus )
        	{          	
        		case DRIVING:
					
					main_state_transition_check(hmi);
										
					//Unterzustandsmaschine DRIVING
					switch ( currentSubStatus ) {
					
						case FOLLOW_LINE:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.FOLLOW_LINE ){
								navigation.setParkingActive(false);
//***************************************************								control.setCtrlMode(ControlMode.INACTIVE);
								control.setCtrlMode(ControlMode.LINE_CTRL);
								Sound.beepSequenceUp();
								navigation.setDetectionState(true);
							}
							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}else if (currentStatus == CurrentStatus.PARK_NOW){
								currentSubStatus = CurrentSubStatus.SEARCH;
							}
							break;	
							
						case WAIT:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.WAIT ){
								control.setCtrlMode(ControlMode.INACTIVE);
								navigation.setDetectionState(false);
								navigation.setParkingActive(false);
							}
							//State transition check; orders are overwritten
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.INACTIVE)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentStatus = CurrentStatus.DRIVING;
							}
							//Collision check	
							if ((perception.getFrontSensorDistance() < 70)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.DRIVE_BACKWARDS;
							}else if ((perception.getFrontSensorDistance() > 150)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							break;	
							
						case DRIVE_BACKWARDS:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.DRIVE_BACKWARDS ){
								control.setCtrlMode(ControlMode.VW_CTRL);	
								control.setVelocity(-15);
								control.setAngularVelocity(0);	
							}
							
							//State transition check; orders are overwritten
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.INACTIVE)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentStatus = CurrentStatus.DRIVING;
							}								
							//Collision check
							if ((perception.getFrontSensorDistance() > 100)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.WAIT;
							}
							break;	
							
						case FIND_BLACK_LINE:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.FIND_BLACK_LINE ){
								control.drive_backwards(false);
								navigation.setParkingActive(true);
								control.setCtrlMode(ControlMode.PARK_CTRL);
								navigation.setDetectionState(false);
							}					

							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}else if ((currentStatus == CurrentStatus.PARK_NOW)||(currentStatus == CurrentStatus.PARK_THIS)){
								currentStatus = CurrentStatus.DRIVING;
							}
							
							//check, if black line is reached
							if(control.destination_reached()){
								//try to find black line if trajectory failed
								if (!destination_reached(navigation)){
								control.setCtrlMode(ControlMode.SETPOSE);
								setDestination(control,navigation);
								} else{
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
								}
							}
							
							break;	
						default:
							break;
					}
					break;
					
				case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
						navigation.setDetectionState(false);
					}

					main_state_transition_check(hmi);
					
					//UNTERZUSTANDSMASCHINE INACTIVE				
					switch ( currentSubStatus ) {
						case WAITING_ON_LINE:
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}else if (currentStatus == CurrentStatus.PARK_NOW){
								currentSubStatus = CurrentSubStatus.SEARCH;
							}
							break;	
						case WAITING_OFF_LINE:
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if ((currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								//overwrite hmi
								currentStatus = CurrentStatus.INACTIVE;			
							}
							break;	
						default:
							break;
						}
					break;
					
				case PARK_THIS:
					
					main_state_transition_check(hmi);
					
					//UNTERZUSTANDSMASCHINE PARK_THIS					
					switch ( currentSubStatus ) {	
						case MOVE_TO_PARKING_POSITION:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION ){																		
								destination = navigation.getParkingSlots()[hmi.getSelectedParkingSlot()];
								setDestination(control, navigation);
								navigation.setParkingActive(false);
								control.setCtrlMode(ControlMode.LINE_CTRL);
								navigation.setDetectionState(true);
							}

							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_NOW){
								currentSubStatus = CurrentSubStatus.SEARCH;
							}
							
							//check, if parking position is reached
							if(destination_reached(navigation)){
								currentSubStatus = CurrentSubStatus.PARK;
							}
						
							break;	
						case PARK:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.PARK ){
								setDestination(control, navigation);
								control.drive_backwards(true);
								navigation.setParkingActive(true);
								control.setCtrlMode(ControlMode.PARK_CTRL);
								navigation.setDetectionState(false);
							}
					
							//check, if parking slot is reached
							if(control.destination_reached()){
								currentStatus = CurrentStatus.INACTIVE;
								Sound.beepSequenceUp();
							}				
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.DRIVING)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}
							
							break;	
						default:
							break;
						}
					break;
					
				case PARK_NOW:
					
					main_state_transition_check(hmi);
					
					//UNTERZUSTANDSMASCHINE PARK_NOW				
					switch ( currentSubStatus ) {
						case SEARCH:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.SEARCH ){
								control.setCtrlMode(ControlMode.LINE_CTRL);
								navigation.setDetectionState(true);
								navigation.setParkingActive(false);
							}	
																					
							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else 
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
							currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}
							
							//compare known parking slots with current position
							//move to parking position if parking slot is close							
							{
								ParkingSlot[] parkingSlots = navigation.getParkingSlots();
								Pose currentPose = navigation.getPose();
								for (int i=0; i<parkingSlots.length; i++){
				
									//test if parking slot exists
									if (parkingSlots[i]!=null){
										
										//test if parking slot has status 'GOOD'
										if (parkingSlots[i].getStatus().ordinal()==0){      
											
											lejos.geom.Point current_checking = parkingSlots[i].getBackBoundaryPosition();
											//calculate the parking position of this slot
											double distance_slot_to_line = 0.3;
											double current_checking_X = current_checking.getX() - distance_slot_to_line*Math.sin(navigation.getPhi());
											double current_checking_Y = current_checking.getY() + distance_slot_to_line*Math.cos(navigation.getPhi());
											
											double deltaX = Math.abs(current_checking_X-currentPose.getX());
											double deltaY = Math.abs(current_checking_Y-currentPose.getY());
											double tollerance = 0.1;
											
											//test if robot is close to parking slot
											if((deltaX<tollerance)&&(deltaY<tollerance)){
												destination = navigation.getParkingSlots()[i];
												currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
												Sound.buzz();
												break;
											}
										}
									}																							
								}
							}		
							break;
						case MOVE_TO_PARKING_POSITION:
							//Into action							
							if ( lastSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION ){																		
								setDestination(control, navigation);
								control.setCtrlMode(ControlMode.LINE_CTRL);
								navigation.setDetectionState(true);
								navigation.setParkingActive(false);
							}
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if (collision_detection(perception)){
								//action implemented in method
							} else 
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
								//needed for "into-actions"
								lastSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							
							//check, if parking position is reached
							if(destination_reached(navigation)){
								currentSubStatus = CurrentSubStatus.PARK;
							}
							break;	
						case PARK:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.PARK ){
								
//								//Ziel fuer Testzwecke fix setzen
//								destination = navigation.getParkingSlots()[0];
								
								setDestination(control, navigation);
								control.drive_backwards(true);
								navigation.setParkingActive(true);
								control.setCtrlMode(ControlMode.PARK_CTRL);
								navigation.setDetectionState(false);							
							}
							
							//check, if parking slot is reached
							if(control.destination_reached()){
								currentStatus = CurrentStatus.INACTIVE;
								Sound.beepSequenceUp();
							}				
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.DRIVING)||(currentStatus == CurrentStatus.PARK_THIS)){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}
							break;	
						default:
							break;
						}					
					break;
				case EXIT:
					hmi.disconnect();
					System.exit(0);
					break;
			default:
				break;
        	}
        	
        	Thread.sleep(100);        	
		}
	}
	
	
	/**
	 * State transition check. Checks hmi-orders and NXT-Buttons, used in every state except EXIT
	 * 
	 * 
	 */
	
	private static void main_state_transition_check(INxtHmi hmi)throws Exception{
		//State transition check
		lastStatus = currentStatus;
		if ( Button.ENTER.isDown() ){
			switch (currentStatus){
				case INACTIVE:
					currentStatus = CurrentStatus.DRIVING;
					break;
				default:
					currentStatus = CurrentStatus.INACTIVE;
					break;
			}
			while(Button.ENTER.isDown()){Thread.sleep(1);} //wait for button release
		}else if ( Button.ESCAPE.isDown() ){
			currentStatus = CurrentStatus.EXIT;
			while(Button.ESCAPE.isDown()){Thread.sleep(1);} //wait for button release
		}else if (hmi.getMode() == parkingRobot.INxtHmi.Mode.DISCONNECT){
			currentStatus = CurrentStatus.EXIT;
		} else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
			currentStatus = CurrentStatus.DRIVING;
		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
			currentStatus = CurrentStatus.INACTIVE;						
		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_THIS ){
			currentStatus = CurrentStatus.PARK_THIS;
		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PARK_NOW ){
			currentStatus = CurrentStatus.PARK_NOW;
		}
		Thread.sleep(30);
	}
	
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception,IControl control)throws Exception{
		LCD.clear();	
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		LCD.drawString( CurrentSubStatusAsString() , 0, 3);
		LCD.drawString("X_dest: " + (destination_pose.getX()*100), 0, 4);
		LCD.drawString("Y_dest: " + (destination_pose.getY()*100), 0, 5);
		LCD.drawString("PHI_dest: " + (destination_pose.getHeading()/Math.PI*180), 0, 6);
//		LCD.drawString("sensor " + perception.getRightLineSensorValue(), 0, 7);
		
//		perception.showSensorData();
		
//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
		Thread.sleep(20);
	}
	

	/**
	 * destination to be driven to
	 */
	//dummy-Werte fuers Testen in Navigation eingespeist!
	//bei Initialzustaenden wird ParkingSlot geladen!
	private static ParkingSlot destination = null;
	private static Pose destination_pose = new Pose();
	
/**
 * 	
 * @param control
 * @param navigation
 * @throws Exception
 */
		private static void setDestination(IControl control, INavigation navigation)throws Exception{
			//coordinates of parking slot
			double back_X = destination.getBackBoundaryPosition().getX();
			double back_Y = destination.getBackBoundaryPosition().getY();
			double front_X = destination.getFrontBoundaryPosition().getX();
			double front_Y = destination.getFrontBoundaryPosition().getY();
			//midpoint of parking slot
			double middle_X = (back_X+front_X)/2;
			double middle_Y = (back_Y+front_Y)/2;
			float phi = 0;
			
//			float phi = (float)navigation.getPhi();
			
//			switch(navigation.getLine()){
//			case 1: phi = Math.PI/2;
//					break;
//			case 4: phi = Math.PI;
//					break;
//			}
			
			if(middle_Y<0){
				phi = 0;
//********************************************************************************Nav-Werte in cm oder mm? <-HIER MM
			}else if(middle_X>1.8){
				phi = (float)Math.PI*1/2;
			}else if(middle_Y>0.6){
				phi = (float)Math.PI;
			}else if(middle_X<0){
				phi = (float)Math.PI*3/4;
			}
			
			//coordinates on black line, which are supposed to be driven to before parking
			double distance_slot_to_line = 0.3;
			double distance_to_go = 0.2;
			double line_X = middle_X + distance_to_go*Math.cos(phi) - distance_slot_to_line*Math.sin(phi);
			double line_Y = middle_Y + distance_slot_to_line*Math.cos(phi) + distance_to_go*Math.sin(phi);
			//vermeide Einparken mit schiefem Startwinkel an zweiter Ecke
			if (line_Y>0.55){
				line_Y = line_Y-0.03;
			}
			destination_pose.setHeading(phi);
			
			if(currentSubStatus == CurrentSubStatus.MOVE_TO_PARKING_POSITION){
				destination_pose.setLocation((float)line_X, (float)line_Y);
			}
			if((currentSubStatus == CurrentSubStatus.PARK)||(currentSubStatus == CurrentSubStatus.FIND_BLACK_LINE)){
				destination_pose.setLocation((float)middle_X, (float)middle_Y);
				control.setDestination(phi, middle_X, middle_Y);
			} else {
				//make sound if method fails
				Sound.buzz();;
			}
		}
		
		private static boolean destination_reached(INavigation navigation)throws Exception{
			boolean destination_reached = false;
			Pose current_position = navigation.getPose();
			float current_X = current_position.getX();
			float current_Y = current_position.getY();
			double tollerance = 0.04;
			if((Math.abs(current_X-destination_pose.getX())<tollerance)&&(Math.abs(current_Y-destination_pose.getY())<tollerance)){
				Sound.beepSequenceUp();;
				destination_reached = true;
			}
			return destination_reached;
		}	
	
		/**
		 * method to detect objects, which are less than 10cm ahead. 
		 * If so, the currentState and currentSubState are updated, the robot stops moving
		 * @param perception
		 * @return returns true, if an object is detected less than 10cm ahead
		 */
		private static boolean collision_detection(IPerception perception){
			boolean collision = false;
			if ((perception.getFrontSensorDistance() < 100) && (perception.getFrontSensorDistance()!=0)){
				Sound.twoBeeps();
				collision = true;
				currentSubStatus = CurrentSubStatus.WAIT;
				currentStatus = CurrentStatus.DRIVING;
			}
			return collision;
		}
}
