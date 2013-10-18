package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

/**
 * 
 * This dispatcher determines the elevator's future destinations
 * This dispatcher is a numb one, it stops at every floor and operates 
 * all the time.
 * Source of:
 * mDesiredFloor(f,b,d)
 * mDesiredDwell
 * 
 * Sink of:
 * mAtFloor[f,b]
 * mDoorClosed[b,r]
 * mHallCall[f,b,d] // not used
 * mCarCall[f,b] // not used
 * mCarWeight // not used
 * @author Yujia Wang
 * 
 */

public class Dispatcher extends Controller {
	//trace current state
	private int target; //desired floor, initialized to 1
	private int currentFloor;// do not update when all AtFloor are false
	private Hallway currentHallway;
	
    //network interface
	//output
    private WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    private WriteableCanMailbox networkDesiredDwellFront;
    private DesiredDwellCanPayloadTranslator mDesiredDwellFront;
    private WriteableCanMailbox networkDesiredDwellBack;
    private DesiredDwellCanPayloadTranslator mDesiredDwellBack;
    
    //input
    private Utility.AtFloorArray mAtFloor;
    
    private Utility.DoorClosedArray mDoorClosedArrayFront;
    private Utility.DoorClosedArray mDoorClosedArrayBack;
    
    private static int dwellTime = 2000; //in ms
    //add Time translator
    		
	//enumerate states
    private static enum State {
        STATE_SET_TARGET,
        STATE_SET_HALLWAY,
        STATE_EMERGENCY
    }
    
    
  //store the period for the controller
    private SimTime period;
    private State currentState;
    private int maxFloors;
    	    
	public Dispatcher(int maxFloors, SimTime period, boolean verbose) {
		super("Dispatcher", verbose);
		
		this.period = period;
		this.maxFloors = maxFloors;
		this.currentState = State.STATE_SET_TARGET;
		this.target = 1;
		this.currentFloor = 1;
		this.currentHallway = Hallway.NONE;

        //initialize network interface
		//output
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        networkDesiredDwellFront = 
        		CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID 
        				+ ReplicationComputer.computeReplicationId(Hallway.FRONT));
        networkDesiredDwellBack = 
        		CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID 
        				+ ReplicationComputer.computeReplicationId(Hallway.BACK));
        mDesiredDwellFront = new DesiredDwellCanPayloadTranslator(networkDesiredDwellFront, Hallway.FRONT);
        mDesiredDwellBack = new DesiredDwellCanPayloadTranslator(networkDesiredDwellBack, Hallway.BACK);
        canInterface.sendTimeTriggered(networkDesiredDwellFront, period);
        canInterface.sendTimeTriggered(networkDesiredDwellBack, period);
        
        //input
        
        mAtFloor = new Utility.AtFloorArray(canInterface);
                
        mDoorClosedArrayBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
        mDoorClosedArrayFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
        
        timer.start(period);
		
	}

	@Override
	public void timerExpired(Object callbackData) {
		State oldState = currentState;
        switch (currentState) {
            case STATE_SET_TARGET: 	stateSetTarget();	break;
            case STATE_SET_HALLWAY: stateSetHallway();	break;
            case STATE_EMERGENCY: stateEmergency();		break;
            default:
				throw new RuntimeException("State " + currentState + " was not recognized.");
            }
        
        if (currentState == oldState)
        	log("No transitions:", currentState);
        else
        	log("Transitions:", oldState, "->", currentState);
        
        setState(STATE_KEY, currentState.toString());
        timer.start(period);
	}

	private void stateSetTarget() {
		// stops at floor and doors not closed, set new target
		// state actions
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		//currentHallway = mAtFloor.getCurrentHallway();
		target = currentFloor % maxFloors +1;
		//#transition 'T11.1'
		if((mDoorClosedArrayFront.getBothClosed()==true && 
			mDoorClosedArrayBack.getBothClosed()==true) )
			currentState = State.STATE_SET_HALLWAY;
		//#transition 'T11.4.1'
		if ((mDoorClosedArrayFront.getBothClosed()==false || 
			mDoorClosedArrayBack.getBothClosed()==false) &&
			mAtFloor.getCurrentFloor() == MessageDictionary.NONE)
			currentState = State.STATE_EMERGENCY;			
	}

	private void stateSetHallway() {
		//Set hallway to currentHallway at Idle/SetNormalHallway, 
		currentFloor = mAtFloor.getCurrentFloor();
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		//#transition 'T11.3.1'
		if(currentFloor != MessageDictionary.NONE &&
			(mDoorClosedArrayFront.getBothClosed()==false || 
			mDoorClosedArrayBack.getBothClosed()==false)){
			currentState = State.STATE_SET_TARGET;
		}
		//#transition 'T11.4.3'
		if ((mDoorClosedArrayFront.getBothClosed()==false || 
			mDoorClosedArrayBack.getBothClosed()==false) &&
			mAtFloor.getCurrentFloor() == MessageDictionary.NONE)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateEmergency() {
		//DO
		// state actions
		mDesiredFloor.setFloor(1);
		mDesiredFloor.setHallway(Hallway.NONE);
		mDesiredFloor.setDirection(Direction.STOP);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		target = 1;
	}	
}
