/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.elevatormodules.passengers.Passenger;
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
	private int commitableFloor;
	private Direction currentDirection;//current direction, update when drive.d is not STOP, initialize to UP
	private Hallway currentHallway;//initialized to 1
	private SimTime countDown;//countDown when doors are all closed, 
	//when it reaches 0, dispatcher will ignore current floor hall call or car call
	
    //network interface
	//output
    private WriteableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    /*private WriteableCanMailbox networkDesiredDwellFront;
    private DesiredDwellCanPayloadTranslator mDesiredDwellFront;
    private WriteableCanMailbox networkDesiredDwellBack;
    private DesiredDwellCanPayloadTranslator mDesiredDwellBack;
*/    
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
    
    private ReadableCanMailbox networkCarWeight;
	private CarWeightCanPayloadTranslator mCarWeight;
    
	// m/s
	private static final double LEVEL_SPEED = DriveObject.LevelingSpeed;
	private static final double MLEVEL_SPEED = LEVEL_SPEED * 100;
	private static final double SLOW_SPEED = DriveObject.SlowSpeed; // in m/s
	private static final double FAST_SPEED = DriveObject.FastSpeed; // in m/s
	private static final double ACCELERATION = DriveObject.Acceleration; // in
		// m/s^2
	private static final double DECELERATION = DriveObject.Deceleration; // in
		// m/s^2
	private static final double ONETOMILLI = 1000.0;
    private static final SimTime waitingTime  =  new SimTime(5000, SimTimeUnit.MILLISECOND);
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
    	STATE_EMERGENCY, STATE_STOP_STOP
    }
    
	private static enum Commit {
		REACHED, NOTREACHED
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
		this.target = MessageDictionary.NONE;
		this.currentFloor = 1;
		this.commitableFloor = 1;
		this.currentHallway = Hallway.NONE;
		this.countDown = waitingTime;

        //initialize network interface
		//output
        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        /*networkDesiredDwellFront = 
        		CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID 
        				+ ReplicationComputer.computeReplicationId(Hallway.FRONT));
        networkDesiredDwellBack = 
        		CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID 
        				+ ReplicationComputer.computeReplicationId(Hallway.BACK));
        mDesiredDwellFront = new DesiredDwellCanPayloadTranslator(networkDesiredDwellFront, Hallway.FRONT);
        mDesiredDwellBack = new DesiredDwellCanPayloadTranslator(networkDesiredDwellBack, Hallway.BACK);
        canInterface.sendTimeTriggered(networkDesiredDwellFront, period);
        canInterface.sendTimeTriggered(networkDesiredDwellBack, period);*/
        
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
        
        networkCarWeight = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
		canInterface.registerTimeTriggered(networkCarWeight);
        
        timer.start(period);
		
	}

	@Override
	public void timerExpired(Object callbackData) {
		State oldState = currentState;
        switch (currentState) {
        	case STATE_STOP_STOP:	StateStopStop(); break;
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
	
	private Commit commitPoint(int floor, int CarLevelPosition, double speed,
			Direction d) {
		double floorPosition = (floor - 1) * 5 * ONETOMILLI;
		double brakeDistance;
		/*if(speed <= FAST_SPEED/5)
			brakeDistance = FAST_SPEED * FAST_SPEED /25/ (2 * DECELERATION) * ONETOMILLI;
		else if(speed> FAST_SPEED/5 && speed <= FAST_SPEED*2/5)
			brakeDistance = FAST_SPEED * FAST_SPEED *4/25/ (2 * DECELERATION) * ONETOMILLI;
		else if(speed> FAST_SPEED*2/5 && speed <= FAST_SPEED*3/5)
			brakeDistance = FAST_SPEED * FAST_SPEED *9/25/ (2 * DECELERATION) * ONETOMILLI;
		else if(speed> FAST_SPEED*3/5 && speed <= FAST_SPEED*4/5)
			brakeDistance = FAST_SPEED * FAST_SPEED *16/25/ (2 * DECELERATION) * ONETOMILLI;
		else if(speed> FAST_SPEED*4/5 && speed <= FAST_SPEED)
			brakeDistance = FAST_SPEED * FAST_SPEED / (2 * DECELERATION) * ONETOMILLI;
		else
			brakeDistance = SLOW_SPEED * SLOW_SPEED / (2* DECELERATION) * ONETOMILLI;
		*/
		brakeDistance = speed*speed / (2* DECELERATION) * ONETOMILLI;
		double allowance = 200;
		switch (d) {
		case STOP:
			return Commit.NOTREACHED;
		case UP: {
			int estimatedPosition = (int) (floorPosition - brakeDistance - allowance);
			if (estimatedPosition > CarLevelPosition) {
				return Commit.NOTREACHED;
			} else {
				return Commit.REACHED;
			}
		}
		case DOWN: {
			int estimatedPosition = (int) (floorPosition + brakeDistance + allowance);
			if (estimatedPosition < CarLevelPosition) {
				return Commit.NOTREACHED;
			} else {
				return Commit.REACHED;
			}
		}
		default:
			return Commit.NOTREACHED;
		}
	}
	
	private int getNearestCommitableFloor(int currentFloor, int CarLevelPosition, int s, Direction d){
		int floor = -1;
		double speed = s/100.0;
		if (speed <= SLOW_SPEED) return (int)Math.round(mCarLevelPosition.getPosition()/1000.0/5.0) + 1;
		switch(d){
		case UP: {
			for(floor = currentFloor; floor <= Elevator.numFloors; floor ++){
				if (commitPoint(floor, CarLevelPosition, speed, d) == Commit.NOTREACHED){
					return floor;
				}
			}
			break;
		}
		case DOWN: {
			for(floor = currentFloor; floor >= 1; floor --){
				if (commitPoint(floor, CarLevelPosition, speed, d) == Commit.NOTREACHED){
					return floor;
				}
			}
			break;
		}
		case STOP:
		default:
		}
		return MessageDictionary.NONE;
	}
	
	private boolean isIgnoringCurrentFloorCall(){
		return countDown.isLessThanOrEqual(SimTime.ZERO) && mCarWeight.getValue() >= Elevator.MaxCarCapacity- Passenger.DEFAULT_WEIGHT;
	}
	
	private boolean isAllDoorClosed(){
		return (mDoorClosedArrayFront.getBothClosed() && mDoorClosedArrayBack.getBothClosed());
	}
	
	private void stateDownUp() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		countDown = waitingTime;
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.UP);		
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		currentFloor = (int) (Math.ceil((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		/*System.out.println("At state DownUp.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/// #Transition 'T11.6.3'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed()>MLEVEL_SPEED &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
			currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			||(nearestCarCallFloor != -1&&nearestHallCallFloor != -1
			 && currentFloor >= nearestCarCallFloor && nearestCarCallFloor > nearestHallCallFloor)
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1)))
				currentState = State.STATE_DOWN_DOWN;
		// #Transition 'T11.8.6'
		if(	mDriveSpeed.getSpeed() <= MLEVEL_SPEED
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE){
			currentState = State.STATE_STOP_UP;
		}
		//#Transition 'S11.1.8'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				 !isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateDownDown() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.DOWN);
		countDown = waitingTime;
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		currentFloor = (int) (Math.ceil((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		/*System.out.println("At state DownDown.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/// #Transition 'T11.8.5'
		if(	mDriveSpeed.getSpeed() <= MLEVEL_SPEED 
			&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
			){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'S11.1.7'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateDownStop() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		countDown = waitingTime;
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		currentFloor = (int) (Math.ceil((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		/*System.out.println("At state DownStop.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/// #Transition 'T11.6.2'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
			currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			||(nearestCarCallFloor != -1&&nearestHallCallFloor != -1
			 && currentFloor >= nearestCarCallFloor && nearestCarCallFloor > nearestHallCallFloor)
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1))){
			currentState = State.STATE_DOWN_DOWN;
			}
		// #Transition 'T11.7.2'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)==-1)
			
			|| (nearestCarCallFloor !=1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)!=-1))
			
			|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall())==-1))){
				currentState = State.STATE_DOWN_UP;
		}
		// #Transition 'T11.8.4'
		if(	mDriveSpeed.getSpeed() <= MLEVEL_SPEED
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				){
			currentState = State.STATE_STOP_STOP;
		}
		//#Transition 'S11.1.6'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateUpDown() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		currentFloor = (int) (Math.floor((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.DOWN);
		countDown = waitingTime;
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		/*System.out.println("At state UpDown.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/// #Transition 'T11.3.3'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
				((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1 
				&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
				|| (nearestCarCallFloor != -1&& nearestHallCallFloor != -1
				&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < nearestHallCallFloor)
				||(currentFloor <= nearestHallCallUpFloor))){
				currentState = State.STATE_UP_UP;
			}
		// #Transition 'T11.8.3'
		if(	mDriveSpeed.getSpeed() <= MLEVEL_SPEED
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'S11.1.5'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;		
	}

	private void stateUpUp() {
		// TODO Auto-generated method stub
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.UP);
		countDown = waitingTime;
		///mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		currentFloor = (int) (Math.floor((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		/*System.out.println("At state UpUp.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/// #Transition 'T11.8.2'
		if(	mDriveSpeed.getSpeed() <= MLEVEL_SPEED
			&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
			){
			currentState = State.STATE_STOP_UP;
		}
		//#Transition 'S11.1.4'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateUpStop() {
		// TODO Auto-generated method stub
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		countDown = waitingTime;
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		currentFloor = (int) (Math.floor((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		/*System.out.println("At state UpStop.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/// #Transition 'T11.3.2'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
				((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1 
				&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
				|| (nearestCarCallFloor != -1&& nearestHallCallFloor != -1
				&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < nearestHallCallFloor)
				||(currentFloor <= nearestHallCallUpFloor))){
				currentState = State.STATE_UP_UP;
			}
		// #Transition 'T11.4.2'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)==-1)
			
			|| (nearestCarCallFloor !=1 && secondNearestCarCallFloor == -1 
				&& (mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)!=-1
					|| mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false)!=-1))
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall())==-1))){
				currentState = State.STATE_UP_DOWN;
		}
		// #Transition 'T11.8.4'
		if(	mDriveSpeed.getSpeed() <= MLEVEL_SPEED
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				){
			currentState = State.STATE_STOP_STOP;
		}
		//#Transition 'S11.1.3'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}
	
	private void StateStopStop() {
		// TODO Auto-generated method stub
		if(mDoorClosedArrayFront.getBothClosed() == true &&
				mDoorClosedArrayFront.getBothClosed() == true){
			countDown = SimTime.subtract(countDown, period);
		}else
			countDown = waitingTime;
		mDesiredFloor.setFloor(MessageDictionary.NONE);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		currentDirection = Direction.STOP;
		mDesiredFloor.setDirection(Direction.STOP);
		//keep desiredDirection
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		commitableFloor = currentFloor;
		int nearestCarCallUpFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, isIgnoringCurrentFloorCall());
		int nearestCarCallDownFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, isIgnoringCurrentFloorCall());
		
		int nearestHallCallUpUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallUpDownFloor = mHallCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallUpFloor = computeNearestFloor(nearestHallCallUpUpFloor, nearestHallCallUpDownFloor, Direction.UP);
		
		int nearestHallCallDownUpFloor = mHallCall.getNearestPressedFloor(commitableFloor-1, Direction.DOWN, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = computeNearestFloor(nearestHallCallDownUpFloor, nearestHallCallDownDownFloor, Direction.DOWN);
		
		/*System.out.println("At state StopStop.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*///#Transition S11.9
		if ((nearestCarCallDownFloor != -1 || nearestHallCallDownFloor != -1)
			&& (nearestCarCallUpFloor ==-1 && nearestHallCallUpFloor == -1)
				&& isAllDoorClosed()){
			currentState = State.STATE_STOP_DOWN;
		}
		
		//#Transition S11.10
		if ((nearestCarCallUpFloor !=-1 || nearestHallCallUpFloor != -1)
			&& isAllDoorClosed()){
			currentState = State.STATE_STOP_UP;
		}
		
		//#Transition 'S11.1.9'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
			!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void StateStopDown() {
		// TODO Auto-generated method stub
		if(mDoorClosedArrayFront.getBothClosed() == true &&
				mDoorClosedArrayFront.getBothClosed() == true){
			countDown = SimTime.subtract(countDown, period);
		}else
			countDown = waitingTime;
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		currentDirection = Direction.DOWN;
		mDesiredFloor.setDirection(Direction.DOWN);
		//mDesiredFloor.setDirection(Direction.STOP);
		//keep desiredDirection
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		/*System.out.println("At state StopDown.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		*/
		//#Transition
		if (target == -1 && isAllDoorClosed()){
			currentState = State.STATE_STOP_STOP;
		}
		//#Transition 'S11.5'
		if (currentDirection == Direction.DOWN && mDriveSpeed.getSpeed() > MLEVEL_SPEED
			&& nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1
			&& mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, isIgnoringCurrentFloorCall())==-1 
			&& mHallCall.isAllUnpressed() && mDriveSpeed.getSpeed() != 0){
			currentState = State.STATE_DOWN_STOP;
		}
		//#Transition 'S11.6.1'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
			currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			||(nearestCarCallFloor != -1&&nearestHallCallFloor != -1
			 && currentFloor >= nearestCarCallFloor && nearestCarCallFloor > nearestHallCallFloor)
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1)))
		{
				currentState = State.STATE_DOWN_DOWN;
			}
		//#Transition 'S11.7.1'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)==-1)
			
			|| (nearestCarCallFloor !=1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)!=-1))
			
			|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall())==-1))){
				currentState = State.STATE_DOWN_UP;
		}
		//#Transition 'S11.1.2'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
			!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void StateStopUp() {
		// TODO Auto-generated method stub
		if(mDoorClosedArrayFront.getBothClosed() == true &&
				mDoorClosedArrayFront.getBothClosed() == true){
			countDown = SimTime.subtract(countDown, period);
		}else
			countDown = waitingTime;
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		currentDirection = Direction.UP;
		mDesiredFloor.setDirection(Direction.UP);
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, isIgnoringCurrentFloorCall());
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, isIgnoringCurrentFloorCall());
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCurrentFloorCall());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		/*System.out.println("At state StopUp.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
	*/
		//#Transition
		if (target == -1 && isAllDoorClosed()){
			currentState = State.STATE_STOP_STOP;
		}
		
		//#Transition 'S11.2'
		if (currentDirection == Direction.UP && mDriveSpeed.getSpeed() > MLEVEL_SPEED
			&& nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1
			&& mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, isIgnoringCurrentFloorCall())==-1 
			&& mHallCall.isAllUnpressed()){
			currentState = State.STATE_UP_STOP;
		}
		
		//#Transition 'S11.3.1'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1 
			&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			|| (nearestCarCallFloor != -1&& nearestHallCallFloor != -1
			&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < nearestHallCallFloor)
			||(currentFloor <= nearestHallCallUpFloor)))
			{
				currentState = State.STATE_UP_UP;
			}
		//#Transition 'S11.4.1'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > MLEVEL_SPEED &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)==-1)
			
			|| (nearestCarCallFloor !=1 && secondNearestCarCallFloor == -1 
				&& (mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)!=-1
					|| mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false)!=-1))
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, Direction.UP, isIgnoringCurrentFloorCall())==-1))){
				currentState = State.STATE_UP_DOWN;
		}
		//#Transition 'S11.1.1'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateEmergency() {
		//DO
		// state actions
		mDesiredFloor.setFloor(1);
		mDesiredFloor.setHallway(Hallway.NONE);
		mDesiredFloor.setDirection(Direction.STOP);
		/*System.out.println("At state Emergency.\n");
		System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");*/
		//mDesiredDwellFront.set(dwellTime);
		//mDesiredDwellBack.set(dwellTime);
		target = 1;
	}	
}
