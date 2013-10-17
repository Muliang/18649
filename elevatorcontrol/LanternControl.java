/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * Author: Yichao Xue
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;

public class LanternControl extends Controller {
	// define the states
	public static enum State {
		STATE_IDLE, STATE_ON, STATE_OFF
	}

	private static final State INIT_STATE = State.STATE_IDLE;

	private int currentFloor;
	private Direction direction;
	private State state;
	private State newState;
	// define the period of sending out message
	private SimTime period;

	private Utility.AtFloorArray mAtFloor;
	private Utility.DoorClosedArray mDoorClosedFront, mDoorClosedBack;
	private Utility.DesiredFloor mDesiredFloor; // readable

	private WriteableCarLanternPayload localCarLantern;

	public LanternControl(Direction direction, SimTime period, boolean verbose) {
		super("LanternControl"
				+ ReplicationComputer.makeReplicationString(direction), verbose);
		// TODO Auto-generated constructor stub
		this.direction = direction;
		this.period = period;
		this.state = INIT_STATE;

		log("Created LanternControl with period=" + period);
		// readable message
		mAtFloor = new Utility.AtFloorArray(canInterface);
		mDoorClosedFront = new Utility.DoorClosedArray(Hallway.FRONT,
				canInterface);
		mDoorClosedBack = new Utility.DoorClosedArray(Hallway.BACK,
				canInterface);
		mDesiredFloor = new Utility.DesiredFloor(canInterface);

		localCarLantern = CarLanternPayload.getWriteablePayload(this.direction);
		physicalInterface.sendTimeTriggered(localCarLantern, this.period);

		timer.start(period);

	}

	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub
		newState = state;

		switch (state) {
		case STATE_IDLE:
			// do:
			localCarLantern.set(false);
			if (mAtFloor.getCurrentFloor() != MessageDictionary.NONE)
				currentFloor = mAtFloor.getCurrentFloor();
			// #transition 'T7.1'
			if (//(currentFloor == mDesiredFloor.getFloor())&&
					 ((!mDoorClosedFront.getBothClosed()) || (!mDoorClosedBack
							.getBothClosed()))
					&& (direction == mDesiredFloor.getDirection()))
				newState = State.STATE_ON;
			// #transition 'T7.4'
			else if (//(currentFloor == mDesiredFloor.getFloor())&&
					 ((!mDoorClosedFront.getBothClosed()) || (!mDoorClosedBack
							.getBothClosed()))
					&& (direction != mDesiredFloor.getDirection()))
				newState = State.STATE_OFF;
			break;

		case STATE_ON:
			//do:
			localCarLantern.set(true);
			// #transition 'T7.2'
			if((mDoorClosedFront.getBothClosed()) && (mDoorClosedBack
					.getBothClosed()))
				newState = State.STATE_IDLE;
			break;

		case STATE_OFF:
			//do:
			localCarLantern.set(false);
			// #transition 'T7.3'
			if((mDoorClosedFront.getBothClosed()) && (mDoorClosedBack
					.getBothClosed()))
				newState = State.STATE_IDLE;
			break;

		default:
			throw new RuntimeException("State " + state
					+ " was not recognized.");
		}

		if (state == newState) {
			log("remains in state: ", state);
		} else {
			log("Transition:", state, "->", newState);
		}

		// update the state variable
		state = newState;

		// report the current state
		setState(STATE_KEY, newState.toString());

		// schedule the next iteration of the controller
		// you must do this at the end of the timer callback in order to restart
		// the timer
		timer.start(period);
	}

}
