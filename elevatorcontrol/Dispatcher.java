package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
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
 * mHallCall[f,b,d] 
 * mCarCall[f,b] 
 * mDriveSpeed
 * mCarLevelPosition
 * mCarWeight // not used
 * @author Yujia Wang
 * 
 */

public class Dispatcher extends Controller {
	//trace current state
	private int target; //desired floor, initialized to 1
	private int currentFloor;// do not update when all AtFloor are false
	private Direction currentDirection;//current direction, update when drive.d is not STOP, initialize to UP
	private Hallway currentHallway;//initialized to 1
	
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
    private Utility.CarCallArray mCarCall;
    private Utility.HallCallArray mHallCall;
    
    private Utility.DoorClosedArray mDoorClosedArrayFront;
    private Utility.DoorClosedArray mDoorClosedArrayBack;
    
    private ReadableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;
	
	private ReadableCanMailbox networkCarLevelPosition;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    
    private static int dwellTime = 2000; //in ms
    //add Time translator
    		
	//enumerate states
    private static enum State {
    	STATE_STOP_UP,
    	STATE_STOP_DOWN,
    	STATE_UP_STOP,
    	STATE_UP_UP,
    	STATE_UP_DOWN,
    	STATE_DOWN_STOP,
    	STATE_DOWN_DOWN,
    	STATE_DOWN_UP,
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
		this.currentState = State.STATE_STOP_UP;
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
        mCarCall = new Utility.CarCallArray(canInterface);
        mHallCall = new Utility.HallCallArray(canInterface);
        
        networkDriveSpeed = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.registerTimeTriggered(networkDriveSpeed);
        
		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
		
        mDoorClosedArrayBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
        mDoorClosedArrayFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
        
        timer.start(period);
		
	}

	@Override
	public void timerExpired(Object callbackData) {
		State oldState = currentState;
        switch (currentState) {
            case STATE_STOP_UP: 	StateStopUp();	break;
            case STATE_STOP_DOWN: StateStopDown();	break;
            case STATE_UP_STOP: stateUpStop();					break;
            case STATE_UP_UP: stateUpUp();						break;
            case STATE_UP_DOWN: stateUpDown();					break;
            case STATE_DOWN_STOP: stateDownStop();				break;
            case STATE_DOWN_DOWN: stateDownDown();				break;
            case STATE_DOWN_UP: stateDownUp();					break;
            case STATE_EMERGENCY: stateEmergency();				break;
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
	
	private int computeNearestFloor(int floor1, int floor2, Direction d){
		int retval = MessageDictionary.NONE;
		if(floor1 == MessageDictionary.NONE && floor2 != MessageDictionary.NONE){
			retval = floor2;
		}
		else if(floor1 != MessageDictionary.NONE && floor2 == MessageDictionary.NONE){
			retval = floor1;
		}
		else if(floor1 != MessageDictionary.NONE && floor2 != MessageDictionary.NONE){
			if (d == Direction.DOWN){
				retval = Math.max(floor1, floor2);
			}else if(d == Direction.UP){
				retval = Math.min(floor1, floor2);
			}
		}
		return retval;
	}

	private void stateDownUp() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.UP);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 2);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		// #Transition 'T11.6.3'
		if(currentDirection==Direction.DOWN &&
			(((nearestCarCallFloor != -1&&(secondNearestCarCallFloor != -1 || nearestHallCallFloor != -1))
			&& ((currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			|| (currentFloor >= nearestCarCallFloor && nearestCarCallFloor > nearestHallCallFloor)))
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1))){
				currentState = State.STATE_DOWN_DOWN;
			}
		// #Transition 'T11.8.6'
		if(	mDriveSpeed.getDirection()==Direction.STOP 
				&& mDriveSpeed.getSpeed()==0 
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				&& (mDoorClosedArrayFront.getBothClosed() == false||
				mDoorClosedArrayBack.getBothClosed() == false)){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'S11.1.8'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;
	}

	private void stateDownDown() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.DOWN);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 2);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		// #Transition 'T11.8.5'
		if(	mDriveSpeed.getDirection()==Direction.STOP 
			&& mDriveSpeed.getSpeed()==0 
			&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
			&& (mDoorClosedArrayFront.getBothClosed() == false||
			mDoorClosedArrayBack.getBothClosed() == false)){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'S11.1.7'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;
	}

	private void stateDownStop() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 2);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1);
		
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		// #Transition 'T11.6.2'
		if(currentDirection==Direction.DOWN &&
			(((nearestCarCallFloor != -1&&(secondNearestCarCallFloor != -1 || nearestHallCallFloor != -1))
			&& ((currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			|| (currentFloor >= nearestCarCallFloor && nearestCarCallFloor > nearestHallCallFloor)))
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1))){
				currentState = State.STATE_DOWN_DOWN;
			}
		// #Transition 'T11.7.2'
		if(currentDirection==Direction.DOWN &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.UP) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN)==-1)
			
			|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN)==-1))){
				currentState = State.STATE_DOWN_UP;
		}
		// #Transition 'T11.8.4'
		if(	mDriveSpeed.getDirection()==Direction.STOP 
				&& mDriveSpeed.getSpeed()==0 
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				&& (mDoorClosedArrayFront.getBothClosed() == false||
				mDoorClosedArrayBack.getBothClosed() == false)){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'S11.1.6'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;
	}

	private void stateUpDown() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.DOWN);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 2);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		// #Transition 'T11.3.3'
		if(currentDirection==Direction.UP 
			
			&&(  (nearestCarCallFloor != -1&&(secondNearestCarCallFloor != -1 || nearestHallCallFloor != -1))
			&& ( (currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			|| (currentFloor <= nearestCarCallFloor && nearestCarCallFloor < nearestHallCallFloor) )    )
			
			||(currentFloor <= nearestHallCallUpFloor)){
				currentState = State.STATE_UP_UP;
			}
		// #Transition 'T11.8.3'
		if(	mDriveSpeed.getDirection()==Direction.STOP 
				&& mDriveSpeed.getSpeed()==0 
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				&& (mDoorClosedArrayFront.getBothClosed() == false||
				mDoorClosedArrayBack.getBothClosed() == false)){
			currentState = State.STATE_STOP_UP;
		}
		//#Transition 'S11.1.5'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;		
	}

	private void stateUpUp() {
		// TODO Auto-generated method stub
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.UP);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 2);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		
		// #Transition 'T11.8.2'
		if(	mDriveSpeed.getDirection()==Direction.STOP 
			&& mDriveSpeed.getSpeed()==0 
			&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
			&& (mDoorClosedArrayFront.getBothClosed() == false||
			mDoorClosedArrayBack.getBothClosed() == false)){
			currentState = State.STATE_STOP_UP;
		}
		//#Transition 'S11.1.4'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;
	}

	private void stateUpStop() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 2);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1);
		
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		// #Transition 'T11.3.2'
		if(currentDirection==Direction.UP &&
			(((nearestCarCallFloor != -1&&(secondNearestCarCallFloor != -1 || nearestHallCallFloor != -1))
			&& ((currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			|| (currentFloor <= nearestCarCallFloor && nearestCarCallFloor < nearestHallCallFloor)))
			||(currentFloor <= nearestHallCallUpFloor))){
				currentState = State.STATE_UP_UP;
			}
		// #Transition 'T11.4.2'
		if(currentDirection==Direction.UP &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor !=1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.DOWN) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP)==-1)
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP)==-1))){
				currentState = State.STATE_UP_DOWN;
		}
		// #Transition 'T11.8.4'
		if(	mDriveSpeed.getDirection()==Direction.STOP 
				&& mDriveSpeed.getSpeed()==0 
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				&& (mDoorClosedArrayFront.getBothClosed() == false||
				mDoorClosedArrayBack.getBothClosed() == false)){
			currentState = State.STATE_STOP_UP;
		}
		//#Transition 'S11.1.3'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;
	}

	private void StateStopDown() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		currentHallway = mAtFloor.getCurrentHallway();
		currentDirection = Direction.DOWN;
		//mDesiredFloor.setDirection(Direction.STOP);
		//keep desiredDirection
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 2);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1);
		
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		//#Transition S11.10
		if (nearestCarCallFloor ==-1 && nearestHallCallFloor == -1){
			currentState = State.STATE_STOP_UP;
		}		
		//#Transition 'S11.5'
		if (currentDirection == Direction.DOWN && nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1
			&& mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1)==-1 
			&& mHallCall.isAllUnpressed() && mDriveSpeed.getSpeed() != 0){
			currentState = State.STATE_DOWN_STOP;
		}
		//#Transition 'S11.6.1'
		if(currentDirection==Direction.DOWN &&
			(((nearestCarCallFloor != -1&&(secondNearestCarCallFloor != -1 || nearestHallCallFloor != -1))
			&& ((currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			|| (currentFloor >= nearestCarCallFloor && nearestCarCallFloor > nearestHallCallFloor)))
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1))){
				currentState = State.STATE_DOWN_DOWN;
			}
		//#Transition 'S11.7.1'
		if(currentDirection==Direction.DOWN &&
				((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
				&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
				
				|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
				&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP) != -1
				&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.UP) == -1
				&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN)==-1)
				
				|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
				&& mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN)==-1))){
					currentState = State.STATE_DOWN_UP;
			}
		//#Transition 'S11.1.2'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
			currentState = State.STATE_EMERGENCY;
	}

	private void StateStopUp() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		mDesiredFloor.setHallway(currentHallway);
		currentHallway = mAtFloor.getCurrentHallway();
		currentDirection = Direction.UP;
		//mDesiredFloor.setDirection(Direction.STOP);
		//keep desiredDirection
		mDesiredDwellFront.set(dwellTime);
		mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 2);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1);
		
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP);
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.DOWN);
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallDownFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		//#Transition S11.9
		if (nearestCarCallFloor ==-1 && nearestHallCallFloor == -1
			&& (mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1) !=-1 
			|| mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN)!=-1
			|| mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.UP)!=-1)){
			currentState = State.STATE_STOP_DOWN;
		}		
		//#Transition 'S11.2'
		if (currentDirection == Direction.UP && nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1
			&& mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1)==-1 
			&& mHallCall.isAllUnpressed()){
			currentState = State.STATE_UP_STOP;
		}
		//#Transition 'S11.3.1'
		if(currentDirection==Direction.UP &&
			(((nearestCarCallFloor != -1&&(secondNearestCarCallFloor != -1 || nearestHallCallFloor != -1))
			&& ((currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			|| (currentFloor <= nearestCarCallFloor && nearestCarCallFloor < nearestHallCallFloor)))
			||(currentFloor <= nearestHallCallUpFloor))){
				currentState = State.STATE_UP_UP;
			}
		//#Transition 'S11.4.1'
		if(currentDirection==Direction.UP &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.DOWN) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP)==-1)
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP)==-1))){
				currentState = State.STATE_UP_DOWN;
		}
		//#Transition 'S11.1.1'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				(mDoorClosedArrayFront.getBothClosed() == false ||
				mDoorClosedArrayFront.getBothClosed() == false))
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
