package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
/*
 * 18-649 Fall-2013
 * Team#9
 * author:priyam
 */

public class HallButtonControl extends Controller{

	//INPUTS are Readable Objects, while OUTPUTS are Writable Objects
	
	//local physical state
	private ReadableHallCallPayload HallCall; //input
	private WriteableHallLightPayload HallLight; //output
	
	//Network Interfaces
	//receive hall call from the other button
	private Utility.HallCall mHallCall; 
	//translator for the hall light message 
	private Utility.HallLight mHallLight;
	
	
	//private DoorClosedCanPayloadTranslator mDoorClosedFrontLeft;
	
	//Enumerate state
	private enum State {
			STATE_OFF,
			STATE_ON,
	}
	
	//State Variables
	private final Hallway hallway;
	private final Direction direction;
	private final int CurrentFloor;
	private State state;
	
	//The following variables are input interface to the Hall Button Controller 
	private Utility.AtFloorArray mAtFloor;
	private Utility.DoorClosedArray mDoorClosed;
	private Utility.DesiredFloor mDesiredFloor;
	
	//Store the period for the Controller
	private SimTime period;


	//State Variable initialized to the initial state off
	private State state_init = State.STATE_OFF;
	
	/*
	 * The arguments listed in the .cf configuration files should match
	 * given here.
	 * 
	 */
	public HallButtonControl(int floor, Hallway hallway, Direction direction, SimTime period, boolean verbose){
		//call to the Controller superclass constructor
		super("HallButtonControl"+ReplicationComputer.makeReplicationString(floor, hallway,direction),verbose);
		
		//store the constructor arguments in internal state
		this.period = period;
		this.hallway = hallway;
		this.direction = direction;
		this.CurrentFloor = floor;
		this.state = state_init;
		
		//Initialize INPUT Variables and Physical State
		//Create a payload object
		//HallCall		 = HallCallPayload.getReadablePayload(floor, hallway, direction);
		HallCall			= mHallCall.Readable(physicalInterface);
		mAtFloor		 = new Utility.AtFloorArray(canInterface);
		mDesiredFloor	 = new Utility.DesiredFloor(canInterface);
		mDoorClosed		 = new Utility.DoorClosedArray(hallway, canInterface);
		
		//We have to register th payload with physical interface
		//physicalInterface.registerTimeTriggered(HallCall);
		
		//Write to OUTPUT variables
		
		mHallCall		= new Utility.HallCall(canInterface, period, floor, hallway, direction);
		mHallLight		= new Utility.HallLight(canInterface, period, floor, hallway, direction);
		HallLight		= Utility.HallLight.Writeable(physicalInterface, period, floor, hallway, direction);
		//mHallLight		= new BooleanCanPayloadTranslator(mHallCall);
		
		
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
		State OldState = state;
		switch(OldState) {
		case STATE_ON:	
			StateOn();
			break;
		case STATE_OFF:	
			StateOff();
			break;
		default:
			throw new RuntimeException("State " + state + " was not recognized.");
		}
		
			if (state == OldState)
				log("No transition: ", state);
			else
				log("Transition:", OldState, "->", state);
			
			setState(STATE_KEY, state.toString());
			
			timer.start(period);
		}
	
	
	private void StateOn() {
		
		HallLight.set(true);
		mHallLight.set(true);
		mHallCall.set(true);
		// transition T8.2
		if (mDesiredFloor.getFloor() == CurrentFloor && mAtFloor.isAtFloor(CurrentFloor, hallway) == true && !mDoorClosed.getBothClosed()) {
							state = State.STATE_OFF;
		}
		
	}

	private void StateOff() {
		HallLight.set(false);
		mHallLight.set(false);
		mHallCall.set(false);
		
		// transition T8.1
		if(HallCall.pressed() && CurrentFloor != mDesiredFloor.getFloor()) {
			state = State.STATE_ON;
		}
		
	}


}
