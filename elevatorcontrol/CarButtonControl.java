/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * Author: Wenhui Hu (wenhuih)
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;


/**
 * 
 * @author wenhui
 * 
 * This is controller for Car Button. 
 *
 */

public class CarButtonControl extends Controller {
	
	// global
	private int floor;
	private Hallway hallway;
	private SimTime period;
	private State state;
	
	private static final State STATE_INIT = State.STATE_OFF;
	
	private static enum State {
		STATE_ON,
		STATE_OFF,
	}
	
	// input
	private Utility.AtFloorArray mAtFloor;
	private Utility.DoorClosedArray mDoorClosed;
	private Utility.DesiredFloor mDesiredFloor;
	
	private ReadableCarCallPayload CarCall;
	
	
	
	// output
	private Utility.CarCall	mCarCall;
	private Utility.CarLight mCarLight;
	
	private WriteableCarLightPayload CarLight;


	
	public CarButtonControl(int floor, Hallway hallway, SimTime period, boolean verbose)
	{
		super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway), verbose);
		
		// Globals
        this.floor		= floor;
        this.hallway	= hallway;
		this.period		= period;
		this.state		= STATE_INIT;
		
		// Inputs
		mAtFloor		= new Utility.AtFloorArray(canInterface);
		mDesiredFloor	= new Utility.DesiredFloor(canInterface);
		mDoorClosed		= new Utility.DoorClosedArray(hallway, canInterface);
		
		
		
		
		// Outputs
		mCarCall		= new Utility.CarCall(canInterface, period, floor, hallway);
		mCarLight		= new Utility.CarLight(canInterface, period, floor, hallway); 
		
		CarLight		= Utility.CarLight.Writeable(physicalInterface, period, floor, hallway);
		CarCall			= mCarCall.Readable(physicalInterface);


		
		// CarLight 		= CarLightPayload.getWriteablePayload(floor, hallway);
		// CarCall			= CarCallPayload.getReadablePayload(floor, hallway);
		
		
		// Start Timer
		timer.start(period);
		
        log("Created CarButtonControl with period = ", period);
    }

	@Override
	public void timerExpired(Object callbackData) {
		State oldState = state;

		switch(oldState) {
		
			case STATE_ON:	
				StateOn();
				break;
		
			case STATE_OFF:	
				StateOff();
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

	private void StateOn() {
		CarLight.set(true);
		mCarLight.set(true);
		mCarCall.set(true);
		
		// #transition 'T9.1'
		if (mDesiredFloor.getFloor() == floor && mAtFloor.isAtFloor(floor, hallway) == true && !mDoorClosed.getBothClosed()) {
			state = State.STATE_OFF;
		}
		
	}

	private void StateOff() {
		CarLight.set(false);
		mCarLight.set(false);
		mCarCall.set(false);
		
		// #transition 'T9.2'
		if(CarCall.isPressed()) {
			state = State.STATE_ON;
		}
		
	}

}
