package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.HoistwayLimitSensorCanPayloadTranslator;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
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
 * mCarLevelPosition[b,r]
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
	int currentFloor;
	
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
    
    private ReadableCanMailbox networkLevel;
    private LevelingCanPayloadTranslator mLevelUp;
    private LevelingCanPayloadTranslator mLevelDown;
    
    private ReadableCanMailbox networkCarLevelPosition;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
    
    //private ReadableCanMailbox networkDoorClosed;
    //private DoorClosedCanPayloadTranslator mDoorClosed;
    private Utility.DoorClosedArray mDoorClosedArrayFront;
    private Utility.DoorClosedArray mDoorClosedArrayBack;
    
    //private ReadableCanMailbox networkDoorMotor;
    //private DoorCommandCanPayloadTranslator mDoorMotor;
    
    private ReadableCanMailbox networkEmergencyBrake;
    private BooleanCanPayloadTranslator mEmergencyBrake;
    
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
        
        //to be fixed
        //networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(1, Hallway.FRONT));
        //mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, 1, Hallway.FRONT);
        //register to receive periodic updates to the mailbox via the CAN network
        //the period of updates will be determined by the sender of the message
        //canInterface.registerTimeTriggered(networkAtFloor);
        mAtFloor			= new Utility.AtFloorArray(canInterface);
        
        //to be fixed
        networkLevel = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID);
        mLevelUp = new LevelingCanPayloadTranslator(networkLevel, Direction.UP);
        mLevelDown = new LevelingCanPayloadTranslator(networkLevel, Direction.DOWN);
        canInterface.registerTimeTriggered(networkLevel);
        
        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
        
        //to be fixed
        //networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID);
        //mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, Hallway.FRONT, Side.LEFT);
        //canInterface.registerTimeTriggered(networkDoorClosed);
        mDoorClosedArrayBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);
        mDoorClosedArrayFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
        
        networkEmergencyBrake = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
        mEmergencyBrake = new BooleanCanPayloadTranslator(networkEmergencyBrake);
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
		return Commit.REACHED;
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
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'S6.1'
			if (commitPoint(mDesiredFloor.getFloor()) == Commit.NOTREACHED && DesiredDirection==Direction.UP 
					&& mDoorClosedArrayFront.getBothClosed() == true && mDoorClosedArrayBack.getBothClosed() == true
					&& mCarWeight.getWeight() < Elevator.MaxCarCapacity)
				currentState = State.STATE_SLOW_UP;
			//#transition 'S6.2'
			else if (commitPoint(mDesiredFloor.getFloor()) == Commit.NOTREACHED && DesiredDirection==Direction.DOWN
					&& mDoorClosedArrayFront.getBothClosed() == true && mDoorClosedArrayBack.getBothClosed() == true
					&& mCarWeight.getWeight() < Elevator.MaxCarCapacity)
				currentState = State.STATE_SLOW_DOWN;
			//#transition 'S6.6'
			
			//to be fixed mLevel[d] not specified
			else if (mLevelUp.getValue() == false && localDriveSpeed.speed() == 0 
					&& localDriveSpeed.direction() == Direction.STOP)
				currentState = State.STATE_LEVEL_UP;
			//#transition 'S6.8'
			else if (mLevelDown.getValue() == false && localDriveSpeed.speed() == 0 
					&& localDriveSpeed.direction() == Direction.STOP)
				currentState = State.STATE_LEVEL_DOWN;
			//#transition 'S6.9.1'
			else if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateSlowUp(){
			//DO
			// state actions
			localDrive.set(Speed.SLOW, Direction.UP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.UP;
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'S6.3'
			if (commitPoint(mDesiredFloor.getFloor()) == Commit.REACHED && DesiredDirection==Direction.UP 
					&& localDriveSpeed.speed() <= SLOW_SPEED && currentFloor == mDesiredFloor.getFloor())
				currentState = State.STATE_LEVEL_UP;
			//#transition 'S6.9.2'
			else if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateSlowDown(){
			//DO
			// state actions
			localDrive.set(Speed.SLOW, Direction.DOWN);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.DOWN;
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'S6.4'
			if (commitPoint(mDesiredFloor.getFloor()) == Commit.REACHED && DesiredDirection==Direction.DOWN 
					&& localDriveSpeed.speed() <= SLOW_SPEED && currentFloor == mDesiredFloor.getFloor())
				currentState = State.STATE_LEVEL_DOWN;
			//#transition 'S6.9.3'
			else if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateLevelUp(){
			//DO
			// state actions
			localDrive.set(Speed.LEVEL, Direction.UP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.UP;
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'S6.5'
			if (mLevelUp.getValue()==true && localDriveSpeed.speed() <= LEVEL_SPEED)
				currentState = State.STATE_STOP;
			//#transition 'S6.9.4'
			else if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		
		private void stateLevelDown(){
			//DO
			// state actions
			localDrive.set(Speed.LEVEL, Direction.DOWN);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.STOP;
			currentFloor = mAtFloor.getCurrentFloor();
			
			//add get currentFloor method
			
			//#transition 'S6.7'
			if (mLevelDown.getValue()==true 
				&& localDriveSpeed.speed() <= LEVEL_SPEED)
				currentState = State.STATE_STOP;
			//#transition 'S6.9.5'
			else if (mEmergencyBrake.getValue() == true)
				currentState = State.STATE_EMERGENCY;
		}
		      	
		private void stateEmergency(){
			//DO
			// state actions
		    localDrive.set(Speed.STOP, Direction.STOP);
			mDriveSpeed.setSpeed(localDriveSpeed.speed());
			mDriveSpeed.setDirection(localDriveSpeed.direction());
			DesiredDirection = Direction.STOP;
			currentFloor = mAtFloor.getCurrentFloor();
		}

}
