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
import parkingRobot.IPerception.AngleDifferenceMeasurement;
import parkingRobot.IPerception.EncoderSensor;
import lejos.robotics.navigation.Pose;
import lejos.geom.Line;
import lejos.nxt.LCD;
import parkingRobot.hsamr8.ControlRST;
import parkingRobot.hsamr8.HmiPLT;
import parkingRobot.hsamr8.NavigationAT;
import parkingRobot.hsamr8.PerceptionPMP;
import java.awt.Point;
import javax.sound.sampled.Control;

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
	 	 * SubStates for both Parking-Modes, the Robot enters the Parking Slot
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
	
																					/**********************************
	
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

		//ORIGINAL STARTZUSTAENDE
		currentStatus = CurrentStatus.INACTIVE;
		lastStatus    = CurrentStatus.EXIT;
		
		// Generate objects
		
		NXTMotor leftMotor  = new NXTMotor(MotorPort.B);
		NXTMotor rightMotor = new NXTMotor(MotorPort.A);
		
		IPerception perception = new PerceptionPMP(leftMotor, rightMotor);
		//perception.calibrateLineSensors();
		
		INavigation navigation = new NavigationAT(perception);
		IControl    control    = new ControlRST(perception, navigation, leftMotor, rightMotor);
		INxtHmi  	hmi        = new HmiPLT(perception, navigation, control);
		navigation.setMap(map);	

//**************************************************************************************
//**************************************************************************************		
//		destination = navigation.getParkingSlots()[0];
//		destination = navigation.getSlotById(1);
//**************************************************************************************
//**************************************************************************************
		
		while(true) {
			showData(navigation, perception, control);
			
        	switch ( currentStatus )
        	{          	
        		case DRIVING:
					//Into action
					if ( lastStatus != CurrentStatus.DRIVING ){
						//action realized in substates
					}
					
					main_state_transition_check(hmi);
										
					//Unterzustandsmaschine DRIVING
					switch ( currentSubStatus ) {
					
						case FOLLOW_LINE:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.FOLLOW_LINE ){
								control.setCtrlMode(ControlMode.LINE_CTRL);
								navigation.setDetectionState(true);
							}
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if ((perception.getFrontSensorDistance() < 150) && (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.WAIT;
							} else {sub_state_parking_check();}
							break;	
							
						case WAIT:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.WAIT ){
								control.setCtrlMode(ControlMode.INACTIVE);
								navigation.setDetectionState(false);
							}
							//State transition check; orders are overwritten
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.INACTIVE)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentStatus = CurrentStatus.DRIVING;
							}
							//Collision check	
							if ((perception.getFrontSensorDistance() < 100)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.DRIVE_BACKWARDS;
							}else if ((perception.getFrontSensorDistance() > 150)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							break;	
							
						case DRIVE_BACKWARDS:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.DRIVE_BACKWARDS ){
								control.setCtrlMode(ControlMode.VW_CTRL);	
//								X;
//								control.setCtrlMode(ControlMode.PARK_CTRL);	
								control.setVelocity(-15);
								control.setAngularVelocity(0);								
							
							}
							
							//State transition check; orders are overwritten
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.INACTIVE)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentStatus = CurrentStatus.DRIVING;
							}								
							//Collision check
							if ((perception.getFrontSensorDistance() > 120)&& (perception.getFrontSensorDistance()!=0)){
								currentSubStatus = CurrentSubStatus.WAIT;
							}
							break;	
							
						case FIND_BLACK_LINE:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.FIND_BLACK_LINE ){
								control.setCtrlMode(ControlMode.PARK_CTRL);
								navigation.setDetectionState(false);
								setDestination(control);
							}					
							
							//check, if black line is reached
							if(destination_reached(navigation)){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}
							
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}else if ((currentStatus == CurrentStatus.PARK_NOW)||(currentStatus == CurrentStatus.PARK_THIS)){
								currentStatus = CurrentStatus.DRIVING;
							}
							break;	
						default:
							break;
					}
					
					//Leave action
					if ( currentStatus != CurrentStatus.DRIVING ){
						//nothing to do here
					}
					break;
					
				case INACTIVE:
					//Into action
					if ( lastStatus != CurrentStatus.INACTIVE ){
						control.setCtrlMode(ControlMode.INACTIVE);
						navigation.setDetectionState(false);
					}
					
					//While action
					{
						//nothing to do here
					}
					

					main_state_transition_check(hmi);
					
					//UNTERZUSTANDSMASCHINE INACTIVE				
					switch ( currentSubStatus ) {
						case WAITING_ON_LINE:
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else {sub_state_parking_check();}
							break;	
						case WAITING_OFF_LINE:
//							X;
//******************************************************BEACHTE FOLGENDES:
//******************************************************im geparkten Zustand wuerde die Maschine sofort wieder in
//******************************************************FIND_BLACK_LINE springen
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
//							if ((currentStatus == CurrentStatus.DRIVING)||(currentStatus == CurrentStatus.PARK_THIS)||(currentStatus == CurrentStatus.PARK_NOW)){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
//								currentStatus = CurrentStatus.DRIVING;
							}				
							break;	
						default:
							break;
						}
					
					//Leave action
					if ( currentStatus != CurrentStatus.INACTIVE ){
						//nothing to do here
					}					
					break;
					
				case PARK_THIS:
//					//Into action
//					if ( lastStatus != CurrentStatus.PARK_THIS ){
//						//control.setCtrlMode(ControlMode.INACTIVE);
//						//enter action
//					
//						//catch the case, that PARK_NOW switches to PARK_THIS
//						lastSubStatus = CurrentSubStatus.FOLLOW_LINE;
//					}
//					
//					//While action
//					{
//						//nothing to do here
//					}
//					
//
//					main_state_transition_check(hmi);
//					
//					//UNTERZUSTANDSMASCHINE PARK_THIS					
//					switch ( currentSubStatus ) {	
//						case MOVE_TO_PARKING_POSITION:
//							//Into action
//							if ( lastSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION ){																		
//								destination = navigation.getSlotById(getSelectedParkingSlot());
//								control.setCtrlMode(ControlMode.LINE_CTRL);
//								navigation.setDetectionState(true);
//								setDestination(control);
//						}
//				
//							//While action				
//							{}					
//				
//							//State transition check
//							lastSubStatus = currentSubStatus;
//							if (currentStatus == CurrentStatus.DRIVING){
//								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
//							}else if (currentStatus == CurrentStatus.INACTIVE){
//								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
//							}else if (currentStatus == CurrentStatus.PARK_THIS){
//								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
//							}else if (currentStatus == CurrentStatus.PARK_NOW){
//								currentSubStatus = CurrentSubStatus.SEARCH;
//							}
//							
//							//Leave action
//							if ( currentSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION )
//							{}
//							break;	
//						case PARK:
//							//Into action
//							if ( lastSubStatus != CurrentSubStatus.PARK ){
//								X; // nicht aktuell!
//								control.setCtrlMode(ControlMode.PARK_CTRL);
//								navigation.setDetectionState(false);
//							}
//							
//							//While action				
//							{}					
//							
//							//State transition check
//							lastSubStatus = currentSubStatus;
//							if (currentStatus == CurrentStatus.DRIVING){
//								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
//							}else if (currentStatus == CurrentStatus.INACTIVE){
//								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
//							}else if (currentStatus == CurrentStatus.PARK_THIS){
//								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
//							}else if (currentStatus == CurrentStatus.PARK_NOW){
//								currentSubStatus = CurrentSubStatus.SEARCH;
//							}
//							
//							//Leave action
//							if ( currentSubStatus != CurrentSubStatus.PARK )
//							{}
//							break;	
//						default:
//							break;
//						}
//	
//					//Leave action
//					if ( currentStatus != CurrentStatus.PARK_THIS ){
//						//nothing to do here
//					}					
					break;
				case PARK_NOW:
					main_state_transition_check(hmi);
					
					//UNTERZUSTANDSMASCHINE PARK_NOW				
					switch ( currentSubStatus ) {
						case SEARCH:
							//Into action
//							if ( lastSubStatus != CurrentSubStatus.SEARCH ){
//							
//								//catch the case, that PARK_THIS switches to PARK_NOW
//								lastSubStatus = CurrentSubStatus.FOLLOW_LINE;
//								
//								control.setCtrlMode(ControlMode.LINE_CTRL);
//								navigation.setDetectionState(true);
//							}	
//							
//							//compare known parking slots with current position
//							//move to parking position if parking slot is close							
//							{
//								ParkingSlot[] parkingSlots = navigation.getParkingSlots();
//								Pose currentPose = navigation.getPose();
////								for (int i=0; i<parkingSlots.length; i++){
//								for (int i=0; i<parkingSlots.size(); i++){
//				
//									//test if parking slot has status 'GOOD'
//									if (parkingSlots[i].getStatus().ordinal()==0){      
//										
//										lejos.geom.Point current_checking = parkingSlots[i].getFrontBoundaryPosition();
//										double deltaX = Math.abs(current_checking.getX()-currentPose.getX());
//										double deltaY = Math.abs(current_checking.getY()-currentPose.getY());
//										
//										if((deltaX<0.3)&&(deltaY<0.3)){
//											destination = navigation.getParkingSlots()[i];
//											setDestination(control); 
//											currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
//											break;
//										}
//									}									
//															
//				LCD.drawString("SlotStatus " + parkingSlots[0].getStatus().ordinal(), 0, 0);
//				LCD.drawString("deltaX" + deltaX, 0, 1);
//				LCD.drawString("deltaY" + deltaY, 0, 2);
//				LCD.drawString( CurrentSubStatusAsString() , 0, 3);
//				LCD.drawString("X_dest: " + (destination_pose.getX()*100), 0, 4);
//				LCD.drawString("Y_dest: " + (destination_pose.getY()*100), 0, 5);
//				LCD.drawString("PHI_dest: " + (destination_pose.getHeading()/Math.PI*180), 0, 6);
////				LCD.drawString(""+,0,7);
//								
//								
//								
//								
//								}
//							}									
							//State transition check
							lastSubStatus = currentSubStatus;
							if (currentStatus == CurrentStatus.DRIVING){
								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
							}else if (currentStatus == CurrentStatus.PARK_THIS){
							currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
							}
							break;
						case MOVE_TO_PARKING_POSITION:
							//Into action
							control.setCtrlMode(ControlMode.INACTIVE);
							
//							if ( lastSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION ){																		
//									control.setCtrlMode(ControlMode.LINE_CTRL);
//									navigation.setDetectionState(true);
//									setDestination(control);
//							}
//							
//							//check, if parking position is reached
//							if(destination_reached(navigation)){
//								currentSubStatus = CurrentSubStatus.PARK;
//							}
//				
//							//State transition check
//							lastSubStatus = currentSubStatus;
//							if (currentStatus == CurrentStatus.DRIVING){
//								currentSubStatus = CurrentSubStatus.FOLLOW_LINE;
//							}else if (currentStatus == CurrentStatus.INACTIVE){
//								currentSubStatus = CurrentSubStatus.WAITING_ON_LINE;
//							}else if (currentStatus == CurrentStatus.PARK_THIS){
//								currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
//							}
//							
//							//Leave action
//							if ( currentSubStatus != CurrentSubStatus.MOVE_TO_PARKING_POSITION )
//							{}
							break;	
						case PARK:
							//Into action
							if ( lastSubStatus != CurrentSubStatus.PARK ){
								setDestination(control);
								control.setCtrlMode(ControlMode.PARK_CTRL);
								navigation.setDetectionState(false);
//								Thread.sleep(100);								
							}
							
							
							//check, if parking slot is reached
							if(destination_reached(navigation)){
////**************************************beachte: PARK_NOW ist hier immer noch gesetzt!
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
								currentStatus = CurrentStatus.INACTIVE;
							}				
				
							//State transition check
							lastSubStatus = currentSubStatus;
							if ((currentStatus == CurrentStatus.DRIVING)||(currentStatus == CurrentStatus.PARK_THIS)){
								currentSubStatus = CurrentSubStatus.FIND_BLACK_LINE;
							}else if (currentStatus == CurrentStatus.INACTIVE){
								currentSubStatus = CurrentSubStatus.WAITING_OFF_LINE;
							}
				
							//Leave action
							if ( currentSubStatus != CurrentSubStatus.PARK )
							{}
							break;	
						default:
							break;
						}
	
					//Leave action
					if ( currentStatus != CurrentStatus.PARK_NOW ){
						//nothing to do here
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
	}
	
	
	/**
	 * Sub State transition check. Checks if the robot received the order to start a parking maneuver
	 * Used in States DRIVING and INACTIVE
	 * 
	 */
	
	private static void sub_state_parking_check(){
		if (currentStatus == CurrentStatus.PARK_THIS){
			currentSubStatus = CurrentSubStatus.MOVE_TO_PARKING_POSITION;
		}else if (currentStatus == CurrentStatus.PARK_NOW){
			currentSubStatus = CurrentSubStatus.SEARCH;
		}
	}
	
	/**
	 * returns the actual state of the main finite state machine as defined by the requirements
	 * 
	 * @return actual state of the main finite state machine
	 */
	public static CurrentStatus getCurrentStatus(){
		return GuidanceAT.currentStatus;
	}
	
	/**
	 * plots the actual pose on the robots display
	 * 
	 * @param navigation reference to the navigation class for getting pose information
	 */
	protected static void showData(INavigation navigation, IPerception perception,IControl control){
		LCD.clear();	
		LCD.drawString("X (in cm): " + (navigation.getPose().getX()*100), 0, 0);
		LCD.drawString("Y (in cm): " + (navigation.getPose().getY()*100), 0, 1);
		LCD.drawString("Phi (grd): " + (navigation.getPose().getHeading()/Math.PI*180), 0, 2);
		LCD.drawString( CurrentSubStatusAsString() , 0, 3);
		LCD.drawString("X_dest: " + (destination_pose.getX()*100), 0, 4);
		LCD.drawString("Y_dest: " + (destination_pose.getY()*100), 0, 5);
		LCD.drawString("PHI_dest: " + (destination_pose.getHeading()/Math.PI*180), 0, 6);
//		LCD.drawString("front_dist"+perception.getFrontSensorDistance(),0,7);
//		LCD.drawString("status " + navigation.getParkingSlots()[0].getStatus().ordinal(), 0, 7);
		
//		perception.showSensorData();
		
//    	if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.SCOUT ){
//			LCD.drawString("HMI Mode SCOUT", 0, 3);
//		}else if ( hmi.getMode() == parkingRobot.INxtHmi.Mode.PAUSE ){
//			LCD.drawString("HMI Mode PAUSE", 0, 3);
//		}else{
//			LCD.drawString("HMI Mode UNKNOWN", 0, 3);
//		}
	}
	
	

	/**
	 * destination to be driven to
	 */
	//dummy-Werte fuers Testen in Navigation eingespeist!
	//bei Initialzustaenden wird ParkingSlot geladen!
	private static ParkingSlot destination = null;
	private static Pose destination_pose = new Pose();
	
	
	//private static void setDestination(ParkingSlot parkingslot,IControl control){
		private static void setDestination(IControl control)throws Exception{
//****************************************************************Teste Slotstatus?!			
			//<-NEEDED?
			//coordinates of ParkingSlot
			double back_X = destination.getBackBoundaryPosition().getX();
			double back_Y = destination.getBackBoundaryPosition().getY();
			double front_X = destination.getFrontBoundaryPosition().getX();
			double front_Y = destination.getFrontBoundaryPosition().getY();
			double middle_X = (back_X+front_X)/2;
			double middle_Y = (back_Y+front_Y)/2;
			float phi = 0;
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
			double line_X = middle_X + distance_slot_to_line*Math.cos(phi) - distance_slot_to_line*Math.sin(phi);
			double line_Y = middle_Y + distance_slot_to_line*Math.cos(phi) + distance_slot_to_line*Math.sin(phi);
			destination_pose.setHeading(phi);
			
			if(currentSubStatus == CurrentSubStatus.MOVE_TO_PARKING_POSITION){
				Sound.beep();
				destination_pose.setLocation((float)line_X, (float)line_Y);
			}
			if(currentSubStatus == CurrentSubStatus.PARK){
				Sound.beep();
				destination_pose.setLocation((float)middle_X, (float)middle_Y);
				control.setDestination(phi, middle_X, middle_Y);
			} else if(currentSubStatus == CurrentSubStatus.FIND_BLACK_LINE){
				Sound.beep();
				destination_pose.setLocation((float)line_X, (float)line_Y);
				control.setDestination(phi, line_X, line_Y);
			} else {
				//make sound if method fails
				Sound.buzz();
			}
			//Thread.sleep(100);
		}
		
		private static boolean destination_reached(INavigation navigation){
			boolean destination_reached = false;
			Pose current_position = navigation.getPose();
			float current_X = current_position.getX();
			float current_Y = current_position.getY();
			//if(((current_X-destination_pose.getX())<20)&&((current_Y-destination_pose.getY())<20)){
			if((Math.abs(current_X-destination_pose.getX())*100<20)&&(Math.abs(current_Y-destination_pose.getY())*100<20)){
				Sound.buzz();;
				destination_reached = true;
			}
			return destination_reached;
		}
		
//	//private static void setDestination(Pose destination, double x,double y,int phi){
//	private static void setDestination(double phi,double x,double y){
//		destination.setHeading((int)phi);
//		float x_float = (float) x;
//		float y_float = (float) y;
//		destination.setLocation(x_float, y_float);
//	}
	

}
