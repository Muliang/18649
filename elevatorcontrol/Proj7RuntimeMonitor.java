/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * 
 */


package simulator.elevatorcontrol;

/*
* 
* This monitor detects ::
* 
* 1. The number of stops where the car becomes OVERWEIGHT
* when the door are open.  Each stop counted at most once, unless the doors
* close completely and then reopen at the same floor.(that counts as two overweight instances)
*
* 2.The number of WASTED OPENINGS
* The number of times when a door opens but there is no call at that floor
* 
* 3. TIME spent dealing with DOOR REVERSALS
* Count the time from a reversal until the doors fully close, and accumulate this value over the whole test. 
* 
* @author Priya Mahajan
* priyam
* 
*/

import jSimPack.SimTime;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.CarLightPayload.ReadableCarLightPayload;
import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload.ReadableHallLightPayload;


public class Proj7RuntimeMonitor extends RuntimeMonitor {

	
		
		DoorStateMachine doorState = new DoorStateMachine();
		WeightStateMachine weightState = new WeightStateMachine();
		CallStateMachine callState = new CallStateMachine();
		atFloorStateMachine atFloorState = new atFloorStateMachine();
		Stopwatch reversal_time = new Stopwatch();
		
		boolean hasMoved = false;
		boolean wasOverweight = false;
		boolean isWastedOpening = false;
		boolean isdoorReversal = false;
		boolean floor_changed = false;
		boolean call_flag = false;
		//boolean call_flag(Hallway hallway, int floor) = false;
		
		int old_pending_calls = 0;
		int overWeightCount = 0;
		int wastedOpeningCount = 0;
		int doorOpeningCount = 0;
		int call_counter=0;
		int atFloor_counter = 0;
		
		SimTime total_reversal_time = null;
		SimTime period ;
		
		

		public Proj7RuntimeMonitor(){
		}
		
		/**************************************************************************
	     * high level event methods
	     *
	     * these are called by the logic in the message receiving methods and the
	     * state machines
	     **************************************************************************/
	    /**
	     * Called once when the door starts opening
	     * @param hallway which door the event pertains to
	     */
	    private void doorOpening(Hallway hallway) {
	    	
	    		
	    }

	    /**
	     * Called once when the door starts closing
	     * @param hallway which door the event pertains to
	     */
	    private void doorClosing(Hallway hallway) {
	        //System.out.println(hallway.toString() + " Door Closing");
	    }

	    /**
	     * Called once if the door starts opening after it started closing but before
	     * it was fully closed.
	     * @param hallway which door the event pertains to
	     */
	    private void doorReopening(Hallway hallway) {
	        //System.out.println(hallway.toString() + " Door Reopening");
	    }
	    
	    private void doorReversing(Hallway hallway) {
	    	if(doorState.anyDoorReversing(hallway)){
	    		message("Door Reversal");
	    		isdoorReversal = true;
	    		 
	    		reversal_time.start();
	    		
	    	}
	    	
	    }

	    /**
	     * Called once when the doors close completely
	     * @param hallway which door the event pertains to
	     */
	    private void doorClosed(Hallway hallway) {
	        
	        //once ALL doors are closed (both front/back hallway), check to see if the car was overweight
	        if (!doorState.anyDoorOpen()) {
	            if (wasOverweight) {
	                message("Overweight");
	                overWeightCount++;
	                wasOverweight = false;
	            }
	        }
	        if(isdoorReversal)
	        {
	        	/**
	        	 * Stop the stopwatch/timer.
	        	 * Add the accumulated time to total_reversal_time
	        	 */
	        	if(reversal_time.isRunning()){
	        		reversal_time.stop();
	        		total_reversal_time = SimTime.add(total_reversal_time, reversal_time.getAccumulatedTime());
	        	}
	        	isdoorReversal = false;
	        }
	    }
	    
	    /**
	     * Called once when the doors are fully open
	     * @param hallway which door the event pertains to
	     */
	    private void doorOpened(Hallway hallway) {
	        //System.out.println(hallway.toString() + " Door Opened");
	    	//Once all doors are open, check to see if the car is overweight
	    	if(doorState.allDoorsCompletelyOpen(hallway)){
	    	if(wasOverweight){
	    		message("Overweight");
                overWeightCount++;
                wasOverweight = false;
	    	}
	    	}
	    	
	    	if(doorState.anyDoorOpen()){
    	    	if(isWastedOpening == true) {
    	    	
    	    		message("WastedOpening");
    				wastedOpeningCount++;
    	    	}
    	    	}
	    	
	    	
	    	
	    }

	    /**
	     * Called when the car weight changes
	     * @param newWeight the new value of weight(after passengers leave or enter)
	     */
	    private void weightChanged(int newWeight){
	        if (newWeight > Elevator.MaxCarCapacity) {
	            wasOverweight = true;
	        }
	    }

	    /**
	     * Called when the car reaches a floor 
	     * @param floor 
	     * Checks for door opens for floor 
	     */
	    private void floorChanged(int f){
	    	
	    	
	    	int new_pending_calls = old_pending_calls;
	    	 
	    	for(int i=0;i<Elevator.numFloors;i++){
	    		if(callState.anyCarCall(i+1) || callState.anyHallCall(i+1)){
	    			new_pending_calls++;
	    			//message("Pending calls"+new_pending_calls);
	    		}
	    	}
	    	
	    	if(new_pending_calls == old_pending_calls){
	    		isWastedOpening = true;
	    	}
	    	else
	    		isWastedOpening = false;
	    	
	    	old_pending_calls = new_pending_calls;
	    	    
	    	
	    }
	      
		
		/**************************************************************************
	     * low level message receiving methods
	     * 
	     * These mostly forward messages to the appropriate state machines
	     **************************************************************************/
		@Override
	    public void receive(ReadableDoorClosedPayload msg) {
	        doorState.receive(msg);
	    }

	    @Override
	    public void receive(ReadableDoorOpenPayload msg) {
	        doorState.receive(msg);
	    }

	    @Override
	    public void receive(ReadableDoorMotorPayload msg) {
	        doorState.receive(msg);
	    }

	    @Override
	    public void receive(ReadableCarWeightPayload msg) {
	        weightState.receive(msg);
	    }

	    @Override
	    public void receive(ReadableDriveSpeedPayload msg) {
	        if (msg.speed() > 0) {
	            hasMoved = true;
	        }
	    }
	    
	    
	    
	    @Override
	    public void receive(ReadableCarCallPayload msg) {
	    	 callState.receive(msg);
	    }
	    
	  	    
	    @Override
	    public void receive(ReadableHallCallPayload msg) {
	    	callState.receive(msg);
	    }

	    public void receive(ReadableAtFloorPayload msg) {
	        atFloorState.receive(msg);
	    }

	    @Override
	    public void receive(ReadableDoorReversalPayload msg) {
	    	doorState.receive(msg);

	    }

		@Override
		public void timerExpired(Object callbackData) {
			// TODO Auto-generated method stub
			
		}
		
		
		private static enum DoorState {

	        CLOSED,
	        OPENING,
	        OPEN,
	        CLOSING,
	        REVERSING
	    }
		
		private static enum CallState {
			
			ON,
			OFF
		}
		
		/**
	    * Utility class to detect weight changes
	    */
	    private class WeightStateMachine {

	        int oldWeight = 0;

	        public void receive(ReadableCarWeightPayload msg) {
	            if (oldWeight != msg.weight()) {
	                weightChanged(msg.weight());
	            }
	            
	            oldWeight = msg.weight();
	        }
	    }
	    
	    /**
	     * Utility class for keeping track of the door state.
	     * 
	     * Also provides external methods that can be queried to determine the
	     * current door state.
	     */
	    private class DoorStateMachine {

	        DoorState state[] = new DoorState[2];

	        public DoorStateMachine() {
	        	//Initially all front/back doors should be closed
	            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
	            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
	        }

	        public void receive(ReadableDoorClosedPayload msg) {
	            updateState(msg.getHallway());
	        }

	        public void receive(ReadableDoorOpenPayload msg) {
	            updateState(msg.getHallway());
	        }

	        public void receive(ReadableDoorMotorPayload msg) {
	            updateState(msg.getHallway());
	        }

	        public void receive(ReadableDoorReversalPayload msg) {
	        	updateState(msg.getHallway());

		    }
	        private void updateState(Hallway h) {
	            DoorState previousState = state[h.ordinal()];

	            DoorState newState = previousState;

	            if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
	                newState = DoorState.CLOSED;
	            } else if (allDoorsCompletelyOpen(h) && allDoorMotorsStopped(h)) {
	                newState = DoorState.OPEN;
	                //} else if (anyDoorMotorClosing(h) && anyDoorOpen(h)) {
	            } else if (anyDoorMotorClosing(h)) {
	                newState = DoorState.CLOSING;
	            } else if (anyDoorMotorOpening(h)) {
	                newState = DoorState.OPENING;
	            } else if (anyDoorReversing(h)) {
	            	newState = DoorState.REVERSING;
	            }

	            if (newState != previousState) {
	                switch (newState) {
	                    case CLOSED:
	                        doorClosed(h);
	                        break;
	                    case OPEN:
	                        doorOpened(h);
	                        
	                        break;
	                    case OPENING:
	                        if (previousState == DoorState.CLOSING) {
	                            doorReopening(h);
	                        } else {
	                            doorOpening(h);
	                        }
	                        break;
	                    case CLOSING:
	                        doorClosing(h);
	                        break;
	                    case REVERSING:
	                    	doorReversing(h);
	                    	break;

	                }
	            }

	            //set the newState
	            state[h.ordinal()] = newState;
	        }

	        //door utility methods
	        public boolean allDoorsCompletelyOpen(Hallway h) {
	            return doorOpeneds[h.ordinal()][Side.LEFT.ordinal()].isOpen()
	                    && doorOpeneds[h.ordinal()][Side.RIGHT.ordinal()].isOpen();
	        }

	        public boolean anyDoorOpen() {
	            return anyDoorOpen(Hallway.FRONT) || anyDoorOpen(Hallway.BACK);

	        }

	        public boolean anyDoorOpen(Hallway h) {
	            return !doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
	                    || !doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed();
	        }

	        public boolean allDoorsClosed(Hallway h) {
	            return (doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
	                    && doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed());
	        }

	        public boolean allDoorMotorsStopped(Hallway h) {
	            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP && doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
	        }

	        public boolean anyDoorMotorOpening(Hallway h) {
	            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.OPEN || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.OPEN;
	        }

	        public boolean anyDoorMotorClosing(Hallway h) {
	            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.CLOSE || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.CLOSE;
	        }
	        
	       public boolean anyDoorReversing(Hallway h){
	    	   return doorReversals[h.ordinal()][Side.LEFT.ordinal()].isReversing() == true || doorReversals[h.ordinal()][Side.RIGHT.ordinal()].isReversing() == true;
	       }
	    }


	    /**
	     * Utility class for keeping track of all Calls to the floor.
	     * Hall Calls and Car Calls
	     *
	     */
	    private class CallStateMachine{
	    	
	    	CallState state[][] = new CallState[Elevator.numFloors][2];
	    	
	    	
	    	public CallStateMachine(){
	    		for(int i =0; i< Elevator.numFloors ; i++){
	    			state[i][Hallway.FRONT.ordinal()] = CallState.OFF;
	    			state[i][Hallway.BACK.ordinal()] = CallState.OFF;
	    			
	    			
	    		}
	    		
	    	}
	    	 
	    	public void receive(ReadableCarCallPayload msg) {
	    				update_state(msg.getHallway(),msg.getFloor());	    			
		    }
	    	
	    	 public void receive(ReadableHallCallPayload msg) {
	    		       update_state(msg.getHallway(),msg.getFloor());	    		 
	 	    }
	    	 
	    	 
	    	 
	    	 private void update_state(Hallway h, int f){
	    		 CallState previousState = state[f-1][h.ordinal()];
	    		 
		         CallState newState = previousState;
	    		 
		         if (anyCarCalls(f,h) || anyHallCalls(f,h) ){
		        	  newState = CallState.ON;
		        	  
		         }
		         else
		         {
		        	 newState = CallState.OFF;
		        	 
		         }
		         
		         //set the newState
		         state[f-1][h.ordinal()] = newState;
	    		 	 
	    	 }
	    	 
	    	//State utility methods
	    	 
	    	 public boolean anyHallCall(int floor){
	    		 return anyHallCalls(floor, Hallway.FRONT) || anyHallCalls(floor, Hallway.BACK);
	    	 }
	    	 
	    	 public boolean anyCarCall(int floor){
	    		
	    		 return anyCarCalls(floor, Hallway.FRONT) || anyCarCalls(floor, Hallway.BACK);
	    		
	    	 }
	    	 
		     public boolean anyHallCalls(int floor,Hallway h) {
		            return hallCalls[floor-1][h.ordinal()][Direction.UP.ordinal()].pressed()
		                    || hallCalls[floor-1][h.ordinal()][Direction.DOWN.ordinal()].pressed();
		                   
		        }

		        public boolean anyCarCalls(int floor, Hallway h) {
		            return carCalls[floor-1][h.ordinal()].pressed()  ;
		        }

	    }
	    
	    //atFloor State Machine
	    private class atFloorStateMachine{
	    	
	    	int old_floor = 1;
	    	
	    	
	    	public void receive(ReadableAtFloorPayload msg) {
	    		 if (msg.getFloor() == old_floor){
	    	    	 if (!atFloors[old_floor-1][Hallway.BACK.ordinal()].value() && !atFloors[old_floor-1][Hallway.FRONT.ordinal()].value()){
	    	    		 old_floor = MessageDictionary.NONE;
	    	    	 }
	    	     }
	    		 
	    		 else{
	    			 if(msg.getValue()){
	    			  floorChanged(msg.getFloor());
	    			  old_floor = msg.getFloor();
	    			  
		            }
	    		 }	 
	    	}
	    	
	    	
	   
	    	
	    }
	    
		@Override
		protected String[] summarize() {
			 String[] arr = new String[3];
		     arr[0] = "Overweight Count = " + overWeightCount;
		     arr[1] = "Total number of Wasted Openings = " + wastedOpeningCount ;
		     arr[2] = "Total time spent in Door Reversals = " + total_reversal_time;
		     return arr;
		}
		/**
	     * Keep track of time and decide whether to or not to include the last interval
	     */
	    private class ConditionalStopwatch {

	        private boolean isRunning = false;
	        private SimTime startTime = null;
	        private SimTime accumulatedTime = SimTime.ZERO;

	        /**
	         * Call to start the stopwatch
	         */
	        public void start() {
	            if (!isRunning) {
	                startTime = Harness.getTime();
	                isRunning = true;
	            }
	        }

	        /**
	         * stop the stopwatch and add the last interval to the accumulated total
	         */
	        public void commit() {
	            if (isRunning) {
	                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
	                accumulatedTime = SimTime.add(accumulatedTime, offset);
	                startTime = null;
	                isRunning = false;
	            }
	        }

	        /**
	         * stop the stopwatch and discard the last interval
	         */
	        public void reset() {
	            if (isRunning) {
	                startTime = null;
	                isRunning = false;
	            }
	        }

	        public SimTime getAccumulatedTime() {
	            return accumulatedTime;
	        }

	        public boolean isRunning() {
	            return isRunning;
	        }
	    }

	    /**
	     * Keep track of the accumulated time for an event
	     */
	    private class Stopwatch {

	        private boolean isRunning = false;
	        private SimTime startTime = null;
	        private SimTime accumulatedTime = SimTime.ZERO;

	        /**
	         * Start the stopwatch
	         */
	        public void start() {
	            if (!isRunning) {
	                startTime = Harness.getTime();
	                isRunning = true;
	            }
	        }

	        /**
	         * Stop the stopwatch and add the interval to the accumulated total
	         */
	        public void stop() {
	            if (isRunning) {
	                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
	                accumulatedTime = SimTime.add(accumulatedTime, offset);
	                startTime = null;
	                isRunning = false;
	            }
	        }

	        public SimTime getAccumulatedTime() {
	            return accumulatedTime;
	        }

	        public boolean isRunning() {
	            return isRunning;
	        }
	    }

	    /**
	     * Utility class to implement an event detector
	     */
	    private abstract class EventDetector {

	        boolean previousState;

	        public EventDetector(boolean initialValue) {
	            previousState = initialValue;
	        }

	        public void updateState(boolean currentState) {
	            if (currentState != previousState) {
	                previousState = currentState;
	                eventOccurred(currentState);
	            }
	        }

	        /**
	         * subclasses should overload this to make something happen when the event
	         * occurs.
	         * @param newState
	         */
	        public abstract void eventOccurred(boolean newState);
	    }
	
}
	    
	   