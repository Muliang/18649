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
	private SimTime countDown1;//countDown1 when doors are all closed, 
	//when it reaches 0, dispatcher will ignore current floor hall call or car call
	//solving endless door cycling
	private SimTime countDown2;//when elevator leaves a floor it starts count down,
	//it won't respond to currentFloor's hall call until countDown2 reaches 0
	private SimTime countDown3;//when elevator closes all doors and ready to leave,
	//it starts count down, the car won't move until it reaches 0, to let the passenger
	//some time to press the car call button
	
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
    private static final SimTime waitingTime  =  new SimTime(2000, SimTimeUnit.MILLISECOND);
    private static final SimTime respondingTime = new SimTime(2000, SimTimeUnit.MILLISECOND);
    private static final SimTime pressingTime = new SimTime(500, SimTimeUnit.MILLISECOND);
    //add Time translator
    		
	//enumerate states
    private static enum State {
    	STATE_STOP_STOP,
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
		this.countDown1 = waitingTime;
		this.countDown2 = SimTime.ZERO;
		this.countDown3 = pressingTime;

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
        	case STATE_STOP_STOP:	StateStopStop(); 
    		//System.out.println("At state StopStop.\n");
    		break;
            case STATE_STOP_UP: 	StateStopUp(); 
    		//System.out.println("At state StopUp.\n");
    		break;
            case STATE_STOP_DOWN: 	StateStopDown(); 
    		//System.out.println("At state StopDown.\n");
    		break;
            case STATE_UP_STOP: stateUpStop(); 
    		//System.out.println("At state UpStop.\n");
    		break;
            case STATE_UP_UP: stateUpUp(); 
    		//System.out.println("At state UpUp.\n");
    		break;
            case STATE_UP_DOWN: stateUpDown(); 
    		//System.out.println("At state UpDown.\n");
    		break;
            case STATE_DOWN_STOP: stateDownStop(); 
    		//System.out.println("At state DownStop.\n");
    		break;
            case STATE_DOWN_DOWN: stateDownDown(); 
    		//System.out.println("At state DownDown.\n");
    		break;
            case STATE_DOWN_UP: stateDownUp(); 
    		//System.out.println("At state DownUp.\n");
    		break;
            case STATE_EMERGENCY: stateEmergency(); 
    		//System.out.println("At state Emergency.\n");
    		break;
            default:
				throw new RuntimeException("State " + currentState + " was not recognized.");
            }
        /*System.out.print("currentFloor ="+ currentFloor +"\n");
		System.out.print("commitableFloor ="+ commitableFloor +"\n");
		System.out.print("target ="+ target +"\n");*/
		
        
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
		brakeDistance = speed*speed / (2* DECELERATION) * ONETOMILLI;
		double allowance = 0;
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
		if (mAtFloor.getCurrentFloor() != -1 )//&& countDown2.isLessThanOrEqual(SimTime.ZERO))
			return mAtFloor.getCurrentFloor();
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
			return currentFloor;
		default:
		}
		return MessageDictionary.NONE;
	}
	
	private boolean isIgnoringCurrentFloorCall(){
		return countDown1.isLessThanOrEqual(SimTime.ZERO) ||  countDown2.isGreaterThan(SimTime.ZERO);
				//|| (mCarWeight.getValue() >= Elevator.MaxCarCapacity- Passenger.DEFAULT_WEIGHT);
	}
	
	private boolean isIgnoringCommitableFloor(){
		return (commitableFloor == currentFloor) && isIgnoringCurrentFloorCall();
	}
	
	private boolean isAllDoorClosed(){
		return (mDoorClosedArrayFront.getBothClosed() && mDoorClosedArrayBack.getBothClosed());
	}
	
	private void stateDownUp() {
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		countDown1 = waitingTime;
		if(countDown2.isGreaterThan(SimTime.ZERO))
			countDown2 = SimTime.subtract(countDown2, period);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.UP);		
		currentFloor = (int) (Math.ceil((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int secondNearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		int farthestHallCallUpFloor = mHallCall.getFarthestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCommitableFloor());
		if (nearestCarCallFloor != MessageDictionary.NONE)
			target = nearestCarCallFloor;
		else
			target = farthestHallCallUpFloor;
		mDesiredFloor.setFloor(target);
		//#Transition 'T11.8.3'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed()>0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
			currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			||(nearestCarCallFloor != -1 && currentFloor >= nearestCarCallFloor 
			&& ((nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)
			   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false) != -1)
			   ||(nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false)
			   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false) != -1)))
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1)))
				currentState = State.STATE_DOWN_DOWN;
		
		/*System.out.println("DownDown condition 1 = "+(nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
				currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor));
		System.out.println("DownDown condition 2 = "+(nearestCarCallFloor != -1 && currentFloor >= nearestCarCallFloor 
				&& (nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)
				   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false) != -1)
				   ||(nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false)
				   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false) != -1)));
		System.out.println("DownDown condition 3 = "+(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1));
*/
		// #Transition 'T11.10.6'
		if(	mDriveSpeed.getSpeed() <= 0
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE){
			currentState = State.STATE_STOP_UP;
		}
		//#Transition 'S11.11.7'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				 !isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateDownDown() {
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.DOWN);
		countDown1 = waitingTime;
		if(countDown2.isGreaterThan(SimTime.ZERO))
			countDown2 = SimTime.subtract(countDown2, period);
		currentFloor = (int) (Math.ceil((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		if(mCarWeight.getValue() >= Elevator.MaxCarCapacity- Passenger.DEFAULT_WEIGHT
				&& nearestCarCallFloor != -1)
			target = nearestCarCallFloor;
		mDesiredFloor.setFloor(target);
		//#Transition 'T11.10.5'
		if(	mDriveSpeed.getSpeed() <= 0 
			&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
			){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'T11.9.3'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)==-1)
			
			|| (nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false)==-1)
			
			|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall())==-1))){
				currentState = State.STATE_DOWN_UP;
		}
		/*System.out.println("DownUp condition 1 = "+(nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
				&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1));
		System.out.println("DownUp condition 2 = "+(nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
				&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP, false) != -1
				&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false) == -1
				&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)==-1));
		System.out.println("DownUp condition 3 = "+(nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false)==-1));
		System.out.println("DownUp condition 4 = "+ (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
				&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCurrentFloorCall())==-1));
*/		//#Transition 'T11.11.8'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateDownStop() {
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		countDown1 = waitingTime;
		if(countDown2.isGreaterThan(SimTime.ZERO))
			countDown2 = SimTime.subtract(countDown2, period);
		currentFloor = (int) (Math.ceil((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int secondNearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.DOWN);
		}
		if(mCarWeight.getValue() >= Elevator.MaxCarCapacity- Passenger.DEFAULT_WEIGHT
				&& nearestCarCallFloor != -1)
			target = nearestCarCallFloor;
		mDesiredFloor.setFloor(target);
		//#Transition 'T11.8.2'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
			currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			||(nearestCarCallFloor != -1 && currentFloor >= nearestCarCallFloor 
			&& ((nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)
			   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false) != -1)
			   ||(nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false)
			   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false) != -1)))
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1))){
			currentState = State.STATE_DOWN_DOWN;
			}
		//#Transition 'T11.9.2'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)==-1)
			
			|| (nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false)==-1)
			
			|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCommitableFloor())==-1))){
				currentState = State.STATE_DOWN_UP;
		}
		// #Transition 'T11.10.4'
		if(	mDriveSpeed.getSpeed() <= 0
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				){
			currentState = State.STATE_STOP_STOP;
		}
		//#Transition 'T11.11.9'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateUpDown() {
		currentFloor = (int) (Math.floor((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.DOWN);
		countDown1 = waitingTime;
		if(countDown2.isGreaterThan(SimTime.ZERO))
			countDown2 = SimTime.subtract(countDown2, period);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor());
		int secondNearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		int farthestHallCallDownFloor = mHallCall.getFarthestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCommitableFloor());
		if (nearestCarCallFloor != MessageDictionary.NONE)
			target = nearestCarCallFloor;
		else
			target = farthestHallCallDownFloor;
		mDesiredFloor.setFloor(target);
		//#Transition 'T11.5.3'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1 
			&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			
			|| (nearestCarCallFloor != -1 && currentFloor <= nearestCarCallFloor
			&&((nearestCarCallFloor < mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false)
				&& mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false) != -1)
				||(nearestCarCallFloor < mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)
				&& mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)!= -1)))
			||(currentFloor <= nearestHallCallUpFloor))){
				currentState = State.STATE_UP_UP;
			}
		//#Transition 'T11.10.3'
		if(	mDriveSpeed.getSpeed() <= 0
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				){
			currentState = State.STATE_STOP_DOWN;
		}
		//#Transition 'T11.11.3'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;		
	}

	private void stateUpUp() {
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.UP);
		countDown1 = waitingTime;
		if(countDown2.isGreaterThan(SimTime.ZERO))
			countDown2 = SimTime.subtract(countDown2, period);
		currentFloor = (int) (Math.floor((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		if(mCarWeight.getValue() >= Elevator.MaxCarCapacity- Passenger.DEFAULT_WEIGHT
				&& nearestCarCallFloor != -1)
			target = nearestCarCallFloor;
		mDesiredFloor.setFloor(target);
		// #Transition 'T11.10.2'
		if(	mDriveSpeed.getSpeed() <= 0
			&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
			){
			currentState = State.STATE_STOP_UP;
		}
		// #Transition 'T11.6.3'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)==-1)	
			
			|| (nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor-1, Direction.DOWN, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false)==-1)
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor())==-1))){
				currentState = State.STATE_UP_DOWN;
		}
		//#Transition 'T11.11.4'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateUpStop() {
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setHallway(currentHallway);
		mDesiredFloor.setDirection(Direction.STOP);
		countDown1 = waitingTime;
		if(countDown2.isGreaterThan(SimTime.ZERO))
			countDown2 = SimTime.subtract(countDown2, period);
		currentFloor = (int) (Math.floor((mCarLevelPosition.getPosition()/1000.0/5.0))+1);
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int secondNearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			target = computeNearestFloor(nearestCarCallFloor, nearestHallCallFloor, Direction.UP);
		}
		if(mCarWeight.getValue() >= Elevator.MaxCarCapacity- Passenger.DEFAULT_WEIGHT
				&& nearestCarCallFloor != -1)
			target = nearestCarCallFloor;
		mDesiredFloor.setFloor(target);
		// #Transition 'T11.5.2'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1 
			&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			
			|| (nearestCarCallFloor != -1 && currentFloor <= nearestCarCallFloor
			&&((nearestCarCallFloor < mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false)
				&& mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false) != -1)
				||(nearestCarCallFloor < mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)
				&& mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)!= -1)))
				
			||(currentFloor <= nearestHallCallUpFloor))){
				currentState = State.STATE_UP_UP;
			}
		// #Transition 'T11.6.2'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > 0 &&isAllDoorClosed() &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)==-1)
			
			|| (nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor-1, Direction.DOWN, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false)==-1)
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor())==-1))){
				currentState = State.STATE_UP_DOWN;
		}
		// #Transition 'T11.10.1'
		if(	mDriveSpeed.getSpeed() <= 0
				&& mAtFloor.getCurrentFloor() != MessageDictionary.NONE
				){
			currentState = State.STATE_STOP_STOP;
		}
		//#Transition 'T11.11.5'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}
	
	private void StateStopStop() {
		countDown1 = waitingTime;
		target = MessageDictionary.NONE;
		if(mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, false)==currentFloor)
			target = currentFloor;
		mDesiredFloor.setFloor(target);
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		currentDirection = Direction.STOP;
		mDesiredFloor.setDirection(Direction.STOP);
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		commitableFloor = currentFloor;
		int nearestCarCallUpFloor = mCarCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, false);
		int nearestCarCallDownFloor = mCarCall.getNearestPressedFloor(commitableFloor-1, Direction.DOWN, 1, false);
		
		int nearestHallCallUpUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false);
		int nearestHallCallUpDownFloor = mHallCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, Direction.DOWN, false);
		int nearestHallCallUpFloor = computeNearestFloor(nearestHallCallUpUpFloor, nearestHallCallUpDownFloor, Direction.UP);
		
		int nearestHallCallDownUpFloor = mHallCall.getNearestPressedFloor(commitableFloor-1, Direction.DOWN, 1, Direction.UP, false);
		int nearestHallCallDownDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, false);
		int nearestHallCallDownFloor = computeNearestFloor(nearestHallCallDownUpFloor, nearestHallCallDownDownFloor, Direction.DOWN);
		
		//#Transition 'T11.2'
		if ((nearestCarCallDownFloor != -1 || nearestHallCallDownFloor != -1)
			&& (nearestCarCallUpFloor ==-1 && nearestHallCallUpFloor == -1)
				&& isAllDoorClosed()){
			currentState = State.STATE_STOP_DOWN;
		}
		
		//#Transition 'T11.1'
		if ((nearestCarCallUpFloor !=-1 || nearestHallCallUpFloor != -1)
			&& isAllDoorClosed()){
			currentState = State.STATE_STOP_UP;
		}
		
		//#Transition 'T11.11.1'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
			!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void StateStopDown() {
		if(isAllDoorClosed()){
			countDown1 = SimTime.subtract(countDown1, period);
		}else
			countDown1 = waitingTime;
		if(isAllDoorClosed())
			countDown3 = SimTime.subtract(countDown3, period);
		else
			countDown3 = pressingTime;
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		currentDirection = Direction.DOWN;
		mDesiredFloor.setDirection(Direction.DOWN);
				
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.DOWN);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, false);
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, isIgnoringCommitableFloor());
		int secondNearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 2, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.DOWN);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallDownFloor, Direction.DOWN);
		if(target == -1){
			if(nearestHallCallUpFloor != currentFloor)
				target = nearestHallCallUpFloor;
			else
				target = secondNearestHallCallUpFloor;
		}
		if(countDown3.isGreaterThan(SimTime.ZERO)) target = MessageDictionary.NONE;
		mDesiredFloor.setFloor(target);
		if(target != currentFloor && target != -1) countDown2 = respondingTime;
		//#Transition 'T11.3.2'
		if (nearestCarCallFloor == -1 && nearestHallCallFloor == -1
			&& isAllDoorClosed()&& countDown3.isLessThanOrEqual(SimTime.ZERO)){
			currentState = State.STATE_STOP_STOP;
		}
		//#Transition 'T11.7'
		if (currentDirection == Direction.DOWN &&isAllDoorClosed() && mDriveSpeed.getSpeed() > 0&& countDown3.isLessThanOrEqual(SimTime.ZERO)
			&& nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1
			&& mCarCall.getNearestPressedFloor(currentFloor, Direction.UP, 1, false)==-1 
			&& mHallCall.isAllUnpressed() && mDriveSpeed.getSpeed() != 0){
			currentState = State.STATE_DOWN_STOP;
			countDown2 = respondingTime;
		}
		//#Transition 'T11.8.1'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > 0 && countDown3.isLessThanOrEqual(SimTime.ZERO)&&isAllDoorClosed() &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1&&
			currentFloor >= nearestCarCallFloor && nearestCarCallFloor > secondNearestCarCallFloor)
			||(nearestCarCallFloor != -1 && currentFloor >= nearestCarCallFloor 
			&& ((nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)
			   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false) != -1)
			   ||(nearestCarCallFloor > mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false)
			   &&mHallCall.getFarthestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.UP, false) != -1)))
			||(currentFloor >= nearestHallCallDownFloor && nearestHallCallDownFloor != -1)))
			{
				currentState = State.STATE_DOWN_DOWN;
				countDown2 = respondingTime;
			}
		//#Transition 'S11.9.1'
		if(currentDirection==Direction.DOWN && mDriveSpeed.getSpeed() > 0 && countDown3.isLessThanOrEqual(SimTime.ZERO)&&isAllDoorClosed() &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallDownFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.UP, 1, Direction.UP, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, Direction.DOWN, false)==-1)
			
			|| (nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor+1, Direction.UP, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor-1, Direction.DOWN, 1, Direction.UP, false)==-1)
			
			|| (nearestHallCallUpFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.DOWN, isIgnoringCommitableFloor())==-1))){
				currentState = State.STATE_DOWN_UP;
				countDown2 = respondingTime;
		}
		//#Transition 'S11.11.6'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
			!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void StateStopUp() {
		if(isAllDoorClosed()){
			countDown1 = SimTime.subtract(countDown1, period);
		}else
			countDown1 = waitingTime;
		if(isAllDoorClosed())
			countDown3 = SimTime.subtract(countDown3, period);
		else
			countDown3 = pressingTime;			
		currentHallway = mAtFloor.getCurrentHallway();
		mDesiredFloor.setHallway(currentHallway);
		currentDirection = Direction.UP;
		mDesiredFloor.setDirection(Direction.UP);
		//System.out.println("countDown2 <= 0:" + countDown2.isLessThanOrEqual(SimTime.ZERO));
		if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
			currentFloor = mAtFloor.getCurrentFloor(); //do not update currentFloor at hoistway
		/*System.out.println("countDown2 <= 0 :"+countDown2.isLessThanOrEqual(SimTime.ZERO));
		System.out.println("mAtFloor = " +"mAtFloor.getCurrentFloor()");*/
		commitableFloor = getNearestCommitableFloor(currentFloor, mCarLevelPosition.getValue(), mDriveSpeed.getSpeed(), Direction.UP);
		int nearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, false);
		int secondNearestCarCallFloor = mCarCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, false);
		//nearest floor in the negative direction to the currentDirection
		int nearestCarCallFloorN = mCarCall.getNearestPressedFloor(currentFloor-1, Direction.DOWN, 1, false);
		int nearestHallCallUpFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor());
		int nearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.DOWN, isIgnoringCommitableFloor());
		int secondNearestHallCallDownFloor = mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 2, Direction.DOWN, isIgnoringCommitableFloor());
		int nearestHallCallFloor = computeNearestFloor(nearestHallCallUpFloor, nearestHallCallDownFloor, Direction.UP);
		target = computeNearestFloor(nearestCarCallFloor, nearestHallCallUpFloor, Direction.UP);
		if(target == -1){
			if(nearestHallCallDownFloor != currentFloor)
				target = nearestHallCallDownFloor;
			else
				target = secondNearestHallCallDownFloor;
		}
		if(target != currentFloor && target != -1) countDown2 = respondingTime;
		if(countDown3.isGreaterThan(SimTime.ZERO)) target = MessageDictionary.NONE;
		//if(nearestCarCallFloor == currentFloor) countDown2 = SimTime.ZERO;
		mDesiredFloor.setFloor(target);
		//#Transition 'T11.3.1'
		if (target == -1 && isAllDoorClosed()&& countDown3.isLessThanOrEqual(SimTime.ZERO)){
			currentState = State.STATE_STOP_STOP;
		}
		
		//#Transition 'T11.4'
		if (currentDirection == Direction.UP && isAllDoorClosed() &&mDriveSpeed.getSpeed() > 0 && countDown3.isLessThanOrEqual(SimTime.ZERO)
			&& nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1
			&& mCarCall.getNearestPressedFloor(currentFloor, Direction.DOWN, 1, false)==-1 
			&& mHallCall.isAllUnpressed()){
			currentState = State.STATE_UP_STOP;
			countDown2 = respondingTime;
		}
		
		//#Transition 'T11.5.1'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > 0 && countDown3.isLessThanOrEqual(SimTime.ZERO)&& isAllDoorClosed() &&
			((nearestCarCallFloor != -1&& secondNearestCarCallFloor != -1 
			&& currentFloor <= nearestCarCallFloor && nearestCarCallFloor < secondNearestCarCallFloor)
			
			|| (nearestCarCallFloor != -1 && currentFloor <= nearestCarCallFloor
			&&((nearestCarCallFloor < mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false)
				&& mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.DOWN, false) != -1)
				||(nearestCarCallFloor < mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)
				&& mHallCall.getFarthestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)!= -1)))
			||(currentFloor <= nearestHallCallUpFloor)))
			{
				currentState = State.STATE_UP_UP;
				countDown2 = respondingTime;
			}
		//#Transition 'T11.6.1'
		if(currentDirection==Direction.UP && mDriveSpeed.getSpeed() > 0 && countDown3.isLessThanOrEqual(SimTime.ZERO)&& isAllDoorClosed() &&
			((nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& nearestCarCallFloorN != -1 && nearestHallCallUpFloor == -1)
			
			|| (nearestCarCallFloor != -1 && secondNearestCarCallFloor == -1 
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor, Direction.DOWN, 1, Direction.DOWN, false) != -1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false) == -1
			&& mHallCall.getNearestPressedFloor(currentFloor+1, Direction.UP, 1, Direction.UP, false)==-1)
			
			|| (nearestCarCallFloor !=-1 && secondNearestCarCallFloor == -1 
			&& (mHallCall.getNearestPressedFloor(commitableFloor-1, Direction.DOWN, 1, Direction.DOWN, false)!=-1
				|| mHallCall.getNearestPressedFloor(commitableFloor, Direction.DOWN, 1, Direction.UP, false)!=-1)
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, false)==-1
			&& mHallCall.getNearestPressedFloor(nearestCarCallFloor+1, Direction.UP, 1, Direction.DOWN, false)==-1)
			
			|| (nearestHallCallDownFloor != -1 && nearestCarCallFloor == -1
			&& mHallCall.getNearestPressedFloor(commitableFloor, Direction.UP, 1, Direction.UP, isIgnoringCommitableFloor())==-1))){
				currentState = State.STATE_UP_DOWN;
				countDown2 = respondingTime;
		}
		//#Transition 'T11.11.2'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE && 
				!isAllDoorClosed())
			currentState = State.STATE_EMERGENCY;
	}

	private void stateEmergency() {
		mDesiredFloor.setFloor(1);
		mDesiredFloor.setHallway(Hallway.NONE);
		mDesiredFloor.setDirection(Direction.STOP);
		target = 1;
	}	
}
