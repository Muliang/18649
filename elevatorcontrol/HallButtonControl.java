/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;

/*
 * author:priyam
 */

public class HallButtonControl extends Controller{

	// global
	private Hallway hallway;
	private Direction direction;
	private int floor;
	private State state;
	private SimTime period;

	private static final State STATE_INIT = State.STATE_OFF;

	// Enumerate state
	private enum State {
			STATE_OFF,
			STATE_ON,
	}
	

	// input 
	private Utility.AtFloorArray mAtFloor;
	private Utility.DoorClosedArray mDoorClosed;
	private Utility.DesiredFloor mDesiredFloor;
	
	private ReadableHallCallPayload HallCall;
	
	
	
	// output
	private Utility.HallCall	mHallCall;
	private Utility.HallLight mHallLight;
	
	private WriteableHallLightPayload HallLight;

	
	/*
	 * The arguments listed in the .cf configuration files should match
	 * given here.
	 * 
	 */
	public HallButtonControl(int floor, Hallway hallway, Direction direction, SimTime period, boolean verbose){

		super("HallButtonControl"+ReplicationComputer.makeReplicationString(floor, hallway,direction),verbose);
		
		this.period = period;
		this.hallway = hallway;
		this.direction = direction;
		this.floor = floor;
		this.state = STATE_INIT;
		
		// input
		mAtFloor		 = new Utility.AtFloorArray(canInterface);
		mDesiredFloor	 = new Utility.DesiredFloor(canInterface);
		mDoorClosed		 = new Utility.DoorClosedArray(hallway, canInterface);
		
		
		// output		
		mHallCall		= new Utility.HallCall(canInterface, period, floor, hallway, direction);
		mHallLight		= new Utility.HallLight(canInterface, period, floor, hallway, direction);
		
		HallLight		= Utility.HallLight.Writeable(physicalInterface, period, floor, hallway, direction);
		HallCall		= mHallCall.Readable(physicalInterface);
		
		
		timer.start(period);
		/*
		 * The log() method is inherited from the Controller class. 
		 * It takes an array of objects which will be converted to strings
		 * and concatenated only if the log message is actually written.
		 */
		
		
		log("Created HallButtonControl with period = ", period);
		
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
		
		HallLight.set(true);
		mHallLight.set(true);
		mHallCall.set(true);

		// #transition 'T8.2'
		if (mDesiredFloor.getFloor() == floor && mAtFloor.isAtFloor(floor, hallway) == true && !mDoorClosed.getBothClosed()) {
							state = State.STATE_OFF;
		}
		
	}

	private void StateOff() {
		HallLight.set(false);
		mHallLight.set(false);
		mHallCall.set(false);
		
		// #transition 'T8.1'
		if(HallCall.pressed()) {
			state = State.STATE_ON;
		}
		
	}


}
