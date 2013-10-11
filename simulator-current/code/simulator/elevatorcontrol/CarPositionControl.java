package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;

public class CarPositionControl extends Controller{
	
	// Constants
	private static enum State
	{
		STATE_INDICATE_CURRENT_FLOOR,
		STATE_INDICATE_PREVIOUS_FLOOR,
	}

	private static final State STATE_INIT = State.STATE_INDICATE_CURRENT_FLOOR;
	
	// Globals
	private SimTime		period;
	private State		state;
	
	// Internals
	private int currentFloor;
	private int previousFloor;
	
	// Inputs
	private Utility.AtFloorArray		mAtFloor;
	
	// Outputs
	private Utility.CarPositionIndicator			mCarPositionIndicator;
	private WriteableCarPositionIndicatorPayload	CarPositionIndicator;

	public CarPositionControl(SimTime period, boolean verbose) {
		super("CarPositionControl", verbose);

		this.period = period;
		this.state = STATE_INIT;
		
		currentFloor = 1;
		previousFloor = 1;
		
		// Inputs
		mAtFloor				= new Utility.AtFloorArray(canInterface);
		
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
		
		currentFloor = mAtFloor.getCurrentFloor();
		
		// #transition 'T10.2'
		if(currentFloor != MessageDictionary.NONE) {
			state = State.STATE_INDICATE_CURRENT_FLOOR;
		}
		
	}

	private void State_Indicate_Current_Floor() {
		CarPositionIndicator.set(currentFloor);
		mCarPositionIndicator.set(currentFloor);
		
		previousFloor = currentFloor;
		currentFloor = mAtFloor.getCurrentFloor();
		
		// #transition 'T10.1'
		if(currentFloor == MessageDictionary.NONE) {
			state = State.STATE_INDICATE_PREVIOUS_FLOOR;
		}
	}

}
