/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * Author: Yichao Xue
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorMotor;
import simulator.elevatormodules.DoorOpenedCanPayloadTranslator;
import simulator.elevatormodules.DoorReversalCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/*
 * Input Interface:
 * mAtFloor
 * mDriveSpeed
 * mDesireFloor
 * mDesireDwell
 * mDoorClosed
 * mDoorOpened
 * mDoorReversal
 * mCarCall
 * mHallCall
 * mCarWeight
 * 
 * Output Interface:
 * DoorMotor
 * mDoorMotor
 */

public class DoorControl extends Controller {

	// define the states
	public static enum State {
		STATE_CLOSED, STATE_OPENING, STATE_OPEN, STATE_CLOSING, STATE_REOPENING, STATE_REOPEN, STATE_NUDGING,
	}

	// constant value
	// set Dwell value
	private static final SimTime DWELL = new SimTime(2000,
			SimTimeUnit.MILLISECOND);

	// initial state
	private static final State INIT_STATE = State.STATE_CLOSED;

	// set global variables
	private Hallway hallway;
	private Side side;
	private State state;
	private State newState;
	// define the period of sending out message
	private SimTime period;
	// current car weight
	private SimTime countDown;

	// physical interface
	// physical output
	private WriteableDoorMotorPayload localDoorMotor;
	// output network message
	private WriteableCanMailbox networkDoorMotor;
	// no DoorMotor translator

	// input network message
	private Utility.AtFloorArray mAtFloor;

	private ReadableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;

	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	private ReadableCanMailbox networkDoorClosed;
	private DoorClosedCanPayloadTranslator mDoorClosed;
	private ReadableCanMailbox networkDoorOpened;
	private DoorOpenedCanPayloadTranslator mDoorOpened;

	private ReadableCanMailbox networkDoorReversalLeft;
	private DoorReversalCanPayloadTranslator mDoorReversalLeft;
	
	private ReadableCanMailbox networkDoorReversalRight;
	private DoorReversalCanPayloadTranslator mDoorReversalRight;

	// private ReadableCanMailbox networkCarCall;
	private Utility.CarCallArray mCarCall;

	// private ReadableCanMailbox networkHallCall;
	private Utility.HallCallArray mHallCall;

	private ReadableCanMailbox networkCarWeight;
	private CarWeightCanPayloadTranslator mCarWeight;

	// constructor
	public DoorControl(Hallway hallway, Side side, SimTime period, boolean verbose) {
		super("DoorControl"
				+ ReplicationComputer.makeReplicationString(hallway, side),
				verbose);

		this.hallway = hallway;
		this.side = side;
		this.period = period;
		this.state = INIT_STATE;

		// log
		log("Created DoorControl with period=" + period);

		localDoorMotor = DoorMotorPayload.getWriteablePayload(this.hallway,
				this.side);
		physicalInterface.sendTimeTriggered(localDoorMotor, period);

		networkDoorMotor = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(
								this.hallway, this.side));
		
		// initialize mAtFloor
		mAtFloor = new Utility.AtFloorArray(canInterface);
        
		// initialize mDriveSpeed
		networkDriveSpeed = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.registerTimeTriggered(networkDriveSpeed);
        
		// initialize mDesiredFloor
		networkDesiredFloor = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(
				networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
        
		// initialize mDoorClosed
		networkDoorClosed = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(
								this.hallway, this.side));
		mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed,
				this.hallway, this.side);
		canInterface.registerTimeTriggered(networkDoorClosed);
        
		// initialize mDoorOpen
		networkDoorOpened = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(
								this.hallway, this.side));
		mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened,
				this.hallway, this.side);
		canInterface.registerTimeTriggered(networkDoorOpened);
        
		// initialize mDoorReversal
		networkDoorReversalLeft = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(
								this.hallway, Side.LEFT));
		mDoorReversalLeft = new DoorReversalCanPayloadTranslator(
				networkDoorReversalLeft, this.hallway, Side.LEFT);
		canInterface.registerTimeTriggered(networkDoorReversalLeft);
		
		networkDoorReversalRight = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(
								this.hallway, Side.RIGHT));
		mDoorReversalRight = new DoorReversalCanPayloadTranslator(
				networkDoorReversalRight, this.hallway, Side.RIGHT);
		canInterface.registerTimeTriggered(networkDoorReversalRight);

		// initialize mCarCall
		mCarCall = new Utility.CarCallArray(canInterface);

		// initialize mHallCall
		mHallCall = new Utility.HallCallArray(canInterface);
		// initialize mCarweight
		networkCarWeight = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
		canInterface.registerTimeTriggered(networkCarWeight);

		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub
		newState = state;

		switch (state) {
		case STATE_CLOSED:
			StateClosed();
			break;

		case STATE_OPENING:
			StateOpening();
			break;

		case STATE_OPEN:
			StateOpen();
			break;

		case STATE_CLOSING:
			StateClosing();
			break;

		case STATE_REOPENING:
			StateReopening();
			break;

		case STATE_REOPEN:
			StateReopen();
			break;

		case STATE_NUDGING:
			StateNudging();
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

	// method for each state
	private void StateClosed() {
		// do:
		localDoorMotor.set(DoorCommand.STOP);
		// #transition 'T5.1'
		if ((mAtFloor.getCurrentFloor() == mDesiredFloor.getFloor())
				&& (mDesiredFloor.getHallway() == hallway || mDesiredFloor.getHallway() == Hallway.BOTH)
				&& (mDriveSpeed.getDirection() == Direction.STOP || mDriveSpeed.getSpeed() == 0))
			newState = State.STATE_OPENING;
	}

	private void StateOpening() {
		// do:
		localDoorMotor.set(DoorCommand.OPEN);
		// set countDown
		countDown = DWELL;
		// #transition 'T5.2'
		if (mDoorOpened.getValue() == true)
			newState = State.STATE_OPEN;
	}

	private void StateOpen() {
		// do:
		localDoorMotor.set(DoorCommand.STOP);
		// countDown decremented
		countDown = SimTime.subtract(countDown, period);
		// #transition 'T5.3'
		if (countDown.isLessThanOrEqual(SimTime.ZERO)){
			newState = State.STATE_CLOSING;
        }
	}

	//
	private void StateClosing() {
		// do:
		localDoorMotor.set(DoorCommand.CLOSE);
		// #transition 'T5.4'
		if (mDoorClosed.getValue() == true){
			newState = State.STATE_CLOSED;
        }
		// #transition 'T5.5'
		if ((mCarWeight.getWeight() >= Elevator.MaxCarCapacity)
				|| (mCarCall.isPressed(mAtFloor.getCurrentFloor(), hallway))
				|| (mHallCall.isAnyPressed(mAtFloor.getCurrentFloor(), hallway)))
			newState = State.STATE_OPENING;

		// #transition 'T5.6'
		if (mDoorReversalLeft.getValue() == true || 
				mDoorReversalRight.getValue() == true)
			newState = State.STATE_REOPENING;
	}

	private void StateReopening() {
		// do:
		localDoorMotor.set(DoorCommand.OPEN);
		countDown = DWELL;
		// #transition 'T5.7'
		if (mDoorOpened.getValue() == true)
			newState = State.STATE_REOPEN;
	}

	private void StateReopen() {
		// do:
		localDoorMotor.set(DoorCommand.STOP);
		// countdown decremented
		countDown = SimTime.subtract(countDown, period);
		// #transition 'T5.8'
		if (countDown.isLessThanOrEqual(SimTime.ZERO))
			
		newState = State.STATE_NUDGING;
	}

	private void StateNudging() {
		// do:
		localDoorMotor.set(DoorCommand.NUDGE);
		// #transition 'T5.9'
		if ((mCarWeight.getWeight() >= Elevator.MaxCarCapacity)
				|| (mCarCall.isPressed(mAtFloor.getCurrentFloor(), hallway))//need change mCarCall class
				|| (mHallCall.isAnyPressed(mAtFloor.getCurrentFloor(), hallway))//need change mCarCall class
				||	mDoorReversalLeft.getValue() == true || 
				mDoorReversalRight.getValue() == true)
			newState = State.STATE_REOPENING;
		// #transition 'T5.10'
		if (mDoorClosed.getValue() == true)
			newState = State.STATE_CLOSED;
	}
}
