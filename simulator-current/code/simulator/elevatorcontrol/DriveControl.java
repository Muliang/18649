package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.HoistwayLimitSensorCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.elevatormodules.SafetySensorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;


/**
 * This DriveControl which controls the elevator Drive (the main motor moving 
 * Car Up and Down).
 *
 * Source of:
 * Drive
 * mDrive //omit
 * mDriveSpeed
 * 
 * Sink of:
 * DriveSpeed
 * mAtFloor[f,b]
 * mLevel[d]
 * mCarLevelPosition[b,r]//not used
 * mDoorClosed[b,r]
 * mDoorMotor[b,r]// can't find translator
 * mEmergencyBrake
 * mDesiredFloor
 * mHoistwayLimit[d]
 * mCarWeight
 * 
 * @author Yujia Wang
 */

public class DriveControl extends Controller {
	//trace current state
	Direction DesiredDirection;
	private int currentFloor;
	private int desiredFloor;
	
	//local physical state
    private ReadableDriveSpeedPayload localDriveSpeed;
    private WriteableDrivePayload localDrive;

    //network interface
    // receive hall call from the other button
    private WriteableCanMailbox networkDriveSpeedOut;
    // translator for the hall light message -- this is a generic translator
    private DriveSpeedCanPayloadTranslator mDriveSpeed; //added mDriveSpeedTranslator
    
    //private ReadableCanMailbox networkAtFloor;
    //private AtFloorCanPayloadTranslator mAtFloor;
    private Utility.AtFloorArray mAtFloor;
    
    private ReadableCanMailbox networkLevelUp;
    private ReadableCanMailbox networkLevelDown;
    private LevelingCanPayloadTranslator mLevelUp;
    private LevelingCanPayloadTranslator mLevelDown;
    
    private ReadableCanMailbox networkCarLevelPosition;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    
    private Utility.DoorClosedArray mDoorClosedArrayFront;
    private Utility.DoorClosedArray mDoorClosedArrayBack;
    
    //private ReadableCanMailbox networkDoorMotor;
    //private DoorCommandCanPayloadTranslator mDoorMotor;
    
    private ReadableCanMailbox networkEmergencyBrake;
    private SafetySensorCanPayloadTranslator mEmergencyBrake;
    
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    private ReadableCanMailbox networkHoistwayLimit;
    private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimit;
    
    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;

    
    //additional internal state variables
    //private SimTime counter = SimTime.ZERO;

	private static final double LEVEL_SPEED					= .10;	// in m/s
	private static final double SLOW_SPEED					= .25;	// in m/s
	
    //internal constant declarations
    //if the flash is one period, then each on/off portion should be 500ms
    //private final static SimTime flashHalfPeriod = new SimTime(500, SimTime.SimTimeUnit.MILLISECOND);

	
	//enumerate states
    private static enum State {
        STATE_STOP,
        STATE_SLOW_UP,
        STATE_SLOW_DOWN,
        STATE_LEVEL_UP,
        STATE_LEVEL_DOWN,
        STATE_EMERGENCY
    }
    
    private static enum Commit{
    	REACHED,
    	NOTREACHED
    }
    
  //store the period for the controller
    private SimTime period;
    private State currentState;
	
	public DriveControl(SimTime period, boolean verbose) {
		super("DriveControl", verbose);
		
		//initialize state
		this.period = period;
		this.currentState = State.STATE_STOP;

		//initialize physical state
        //create a payload object for this floor,hallway,direction using the
        //static factory method in HallCallPayload.
        localDriveSpeed = DriveSpeedPayload.getReadablePayload();
        //register the payload with the physical interface (as in input) -- it will be updated
        //periodically when the hall call button state is modified.
        physicalInterface.registerTimeTriggered(localDriveSpeed);
        
        //create a payload object for this floor,hallway,direction
        //this is an output, so it is created with the Writeable static factory method
        localDrive = DrivePayload.getWriteablePayload();
        //register the payload to be sent periodically -- whatever value is stored
        //in the localHallLight object will be sent out periodically with the period
        //specified by the period parameter.
        physicalInterface.sendTimeTriggered(localDrive, period);

        //initialize network interface        
        //create a can mailbox - this object has the binary representation of the message data
        //the CAN message ids are declared in the MessageDictionary class.  The ReplicationComputer
        //class provides utility methods for computing offsets for replicated controllers
        networkDriveSpeedOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        /*
         * Create a translator with a reference to the CanMailbox.  Use the 
         * translator to read and write values to the mailbox
         * 
         * Note the use of the BooleanCanPayloadTranslator.  This translator, along with
         * IntegerCanPayloadTranslator, are provided for your use.  They are not
         * very bandwidth efficient, but they will be adequate for the first part
         * of the course.  When we get to network scheduling, you may wish to write
         * your own translators, although you can do so at any time.
         */
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeedOut);
        //register the mailbox to have its value broadcast on the network periodically
        //with a period specified by the period parameter.
        canInterface.sendTimeTriggered(networkDriveSpeedOut, period);

        /*
         * Registration for the DoorClosed message is similar to the mHallLight message
         * 
         * To register for network messages from the smart sensors or other objects
         * defined in elevator modules, use the translators already defined in
         * elevatormodules package.  These translators are specific to one type
         * of message.
         */
        
        mAtFloor = new Utility.AtFloorArray(canInterface);
        
        networkLevelUp = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(Direction.UP));
        networkLevelDown = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID +
        		ReplicationComputer.computeReplicationId(Direction.DOWN));
        mLevelUp = new LevelingCanPayloadTranslator(networkLevelUp, Direction.UP);
        mLevelDown = new LevelingCanPayloadTranslator(networkLevelDown, Direction.DOWN);
        canInterface.registerTimeTriggered(networkLevelUp);
        canInterface.registerTimeTriggered(networkLevelDown);
        
        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
        
        mDoorClosedArrayBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
        mDoorClosedArrayFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
        
        networkEmergencyBrake = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
        mEmergencyBrake = new SafetySensorCanPayloadTranslator(networkEmergencyBrake);
        canInterface.registerTimeTriggered(networkEmergencyBrake);
        
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        
        networkHoistwayLimit = CanMailbox.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID); 
        mHoistwayLimit = new HoistwayLimitSensorCanPayloadTranslator(networkHoistwayLimit, Direction.UP);
        canInterface.registerTimeTriggered(networkHoistwayLimit);
        
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);
        		
        /* issuing the timer start method with no callback data means a NULL value 
         * will be passed to the callback later.  Use the callback data to distinguish
         * callbacks from multiple calls to timer.start() (e.g. if you have multiple
         * timers.
         */
        timer.start(period);
		
	}
	
	private Commit commitPoint(int floor){
		if(currentFloor == floor)
			return Commit.REACHED;
		else
			return Commit.NOTREACHED;
	} 

	@Override
	public void timerExpired(Object callbackData) {
        State oldState = currentState;
        switch (currentState) {
            case STATE_STOP: 	stateStop();		break;
            case STATE_SLOW_UP: stateSlowUp();		break;
            case STATE_SLOW_DOWN: stateSlowDown();	break;
            case STATE_LEVEL_UP: stateLevelUp();	break;
            case STATE_LEVEL_DOWN: stateLevelDown();break;
            case STATE_EMERGENCY: stateEmergency();	break;
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
	
		private void stateStop(){
			//DO
			// state actions
			localDrive.set(Speed.STOP, Direction.STOP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.STOP;
			desiredFloor = mDesiredFloor.getFloor();
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'T6.1'
			if (commitPoint(desiredFloor) == Commit.NOTREACHED && desiredFloor > currentFloor
					&& mDoorClosedArrayFront.getBothClosed() == true && mDoorClosedArrayBack.getBothClosed() == true
					&& mCarWeight.getWeight() < Elevator.MaxCarCapacity)
				currentState = State.STATE_SLOW_UP;
			//#transition 'T6.2'
			else if (commitPoint(desiredFloor) == Commit.NOTREACHED && desiredFloor < currentFloor
					&& mDoorClosedArrayFront.getBothClosed() == true && mDoorClosedArrayBack.getBothClosed() == true
					&& mCarWeight.getWeight() < Elevator.MaxCarCapacity)
				currentState = State.STATE_SLOW_DOWN;
			//#transition 'T6.6'
			
			else if (mLevelUp.getValue() == false //|| mCarLevelPosition.getPosition()<0) 
					&& localDriveSpeed.speed() == 0 
					&& localDriveSpeed.direction() == Direction.STOP)
				currentState = State.STATE_LEVEL_UP;
			//#transition 'T6.8'
			else if (mLevelDown.getValue() == false //|| mCarLevelPosition.getPosition()>0) 
					&& localDriveSpeed.speed() == 0 
					&& localDriveSpeed.direction() == Direction.STOP)
				currentState = State.STATE_LEVEL_DOWN;
			//#transition 'T6.9.1'
			if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateSlowUp(){
			//DO
			// state actions
			localDrive.set(Speed.SLOW, Direction.UP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.UP;
			desiredFloor = mDesiredFloor.getFloor();
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'T6.3'
			if (commitPoint(desiredFloor) == Commit.REACHED 
					&& localDriveSpeed.speed() <= SLOW_SPEED 
					&& currentFloor == mDesiredFloor.getFloor())
				currentState = State.STATE_LEVEL_UP;
			//#transition 'T6.9.2'
			if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateSlowDown(){
			//DO
			// state actions
			localDrive.set(Speed.SLOW, Direction.DOWN);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.DOWN;
			desiredFloor = mDesiredFloor.getFloor();
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'T6.4'
			if (commitPoint(mDesiredFloor.getFloor()) == Commit.REACHED 
					&& localDriveSpeed.speed() <= SLOW_SPEED 
					&& currentFloor == mDesiredFloor.getFloor())
				currentState = State.STATE_LEVEL_DOWN;
			//#transition 'T6.9.3'
			if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateLevelUp(){
			//DO
			// state actions
			localDrive.set(Speed.LEVEL, Direction.UP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.UP;
			desiredFloor = mDesiredFloor.getFloor();
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'T6.5'
			if (mLevelUp.getValue()==true// || mCarLevelPosition.getPosition()>=0)
				&& localDriveSpeed.speed() <= LEVEL_SPEED)
				currentState = State.STATE_STOP;
			//#transition 'T6.9.4'
			if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateLevelDown(){
			//DO
			// state actions
			localDrive.set(Speed.LEVEL, Direction.DOWN);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.STOP;
			desiredFloor = mDesiredFloor.getFloor();
			currentFloor = mAtFloor.getCurrentFloor();
			
			
			//#transition 'T6.7'
			if (mLevelDown.getValue()==true //|| mCarLevelPosition.getPosition()<=0) 
				&& localDriveSpeed.speed() <= LEVEL_SPEED)
				currentState = State.STATE_STOP;
			//#transition 'T6.9.5'
			if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		      	
		private void stateEmergency(){
			//DO
			// state actions
		    localDrive.set(Speed.STOP, Direction.STOP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.STOP;
			desiredFloor = mDesiredFloor.getFloor();
			currentFloor = mAtFloor.getCurrentFloor();
		}

}
