/*
 * 18649 Fall 2013
 * group 9
 * Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * Author: Wenhui Hu (wenhuih)
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.framework.Controller;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;

public class CarPositionControl extends Controller{
	
	// Constants
	private static enum State
	{
		STATE_INDICATE_CURRENT_FLOOR,
		STATE_INDICATE_PREVIOUS_FLOOR,
		STATE_INDICATE_APPROXIMATE_FLOOR;
	}

	private static final State STATE_INIT = State.STATE_INDICATE_CURRENT_FLOOR;
	
	private static final double SLOW_SPEED		= DriveObject.SlowSpeed;		// in m/s
	
	// Globals
	private SimTime		period;
	private State		state;
	
	// Internals
	private int currentFloor;
	private int previousFloor;
	private int approximateFloor;
	
	// Inputs
	private Utility.AtFloorArray					mAtFloor;
	private	CarLevelPositionCanPayloadTranslator	mCarLevelPosition;
	private DriveSpeedCanPayloadTranslator			mDriveSpeed;
	
    private ReadableCanMailbox networkCarLevelPosition;
    private ReadableCanMailbox networkDriveSpeed;
	
	// Outputs
	private Utility.CarPositionIndicator			mCarPositionIndicator;
	private WriteableCarPositionIndicatorPayload	CarPositionIndicator;

	public CarPositionControl(SimTime period, boolean verbose) {
		super("CarPositionControl", verbose);

		this.period = period;
		this.state = STATE_INIT;
		
		currentFloor = 1;
		previousFloor = 1;
		approximateFloor = 1;
		
		// Inputs
		mAtFloor				= new Utility.AtFloorArray(canInterface);

        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
        
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDriveSpeed);

		// Outputs
		CarPositionIndicator	= Utility.CarPositionIndicator.Writeable(physicalInterface, period);
		mCarPositionIndicator 	= new Utility.CarPositionIndicator(canInterface, physicalInterface, period);
		
		// Start Timer
		timer.start(period);
		
	}

	@Override
	public void timerExpired(Object callbackData) {
		State oldState = state;

		switch(state) {
		
			case STATE_INDICATE_CURRENT_FLOOR:     
				State_Indicate_Current_Floor();    
				break;
				
			case STATE_INDICATE_PREVIOUS_FLOOR:
				State_Indicate_Previous_Floor();
				break;
				
			case STATE_INDICATE_APPROXIMATE_FLOOR:
				State_Indicate_Approximate_Floor();
				break;
				
			default: 
				throw new RuntimeException("State " + state + " was not recognized.");
		}
		
		if (state == oldState)
			log("No transition: ", state);
		else
			log("Transition:", oldState, "->", state);
		
		setState(STATE_KEY, state.toString());
		
		timer.start(period);
		
	}

	private void State_Indicate_Previous_Floor() {
		CarPositionIndicator.set(previousFloor);
		mCarPositionIndicator.set(previousFloor);
		
		previousFloor = mCarPositionIndicator.getValue();
		currentFloor = mAtFloor.getCurrentFloor();
		approximateFloor = (int)Math.round(mCarLevelPosition.getPosition()/1000.0/5.0) + 1;
		
		// #transition 'T10.2.1'
		if(currentFloor != MessageDictionary.NONE && mDriveSpeed.getSpeed() <= SLOW_SPEED) {
			state = State.STATE_INDICATE_CURRENT_FLOOR;
		}
		
		// #transition 'T10.3.1'
		if(mDriveSpeed.getSpeed() > SLOW_SPEED) {
			state = State.STATE_INDICATE_APPROXIMATE_FLOOR;
		}
	}

	private void State_Indicate_Current_Floor() {
		CarPositionIndicator.set(currentFloor);
		mCarPositionIndicator.set(currentFloor);
		
		
		previousFloor = mCarPositionIndicator.getValue();
		currentFloor = mAtFloor.getCurrentFloor();
		approximateFloor = (int)Math.round(mCarLevelPosition.getPosition()/1000.0/5.0) + 1;
		
		
		// #transition 'T10.1.1'
		if(currentFloor == MessageDictionary.NONE && mDriveSpeed.getSpeed() <= SLOW_SPEED) {
			state = State.STATE_INDICATE_PREVIOUS_FLOOR;
		}
		// #transition 'T10.3.2'
		if(mDriveSpeed.getSpeed() > SLOW_SPEED) {
			state = State.STATE_INDICATE_APPROXIMATE_FLOOR;
		}
	}
	
	private void State_Indicate_Approximate_Floor() {
		CarPositionIndicator.set(approximateFloor);
		mCarPositionIndicator.set(approximateFloor);
		
		previousFloor = mCarPositionIndicator.getValue();
		currentFloor = mAtFloor.getCurrentFloor();
		approximateFloor = (int)Math.round(mCarLevelPosition.getPosition()/1000.0/5.0) + 1;
		
		// #transition 'T10.2.2'
		if(mDriveSpeed.getSpeed() <= SLOW_SPEED && currentFloor != MessageDictionary.NONE) {
			state = State.STATE_INDICATE_CURRENT_FLOOR;
		}
		
		// #transition 'T10.1.2'
		if(mDriveSpeed.getSpeed() <= SLOW_SPEED && currentFloor == MessageDictionary.NONE) {
			state = State.STATE_INDICATE_PREVIOUS_FLOOR;
		}
	
	}

}
