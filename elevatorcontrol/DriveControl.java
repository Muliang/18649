/*
 * 18649 Fall 2013
 * group 9
 * Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * Author: Yujia Wang
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
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
 * mDriveSpeed
 * 
 * Sink of: 
 * DriveSpeed 
 * mAtFloor[f,b] 
 * mLevel[d] m
 * CarLevelPosition[b,r]
 * mDoorClosed[b,r] 
 * mEmergencyBrake
 * mDesiredFloor 
 * mHoistwayLimit[d] 
 * mCarWeight
 * 
 * @author Yujia Wang
 */

public class DriveControl extends Controller {
	// trace current state
	Direction DesiredDirection;
	private int currentFloor;
	private int desiredFloor;

	// local physical state
	private ReadableDriveSpeedPayload localDriveSpeed;
	private WriteableDrivePayload localDrive;

	// network interface
	// receive hall call from the other button
	private WriteableCanMailbox networkDriveSpeedOut;
	// translator for the hall light message -- this is a generic translator
	private DriveSpeedCanPayloadTranslator mDriveSpeed; // added
														// mDriveSpeedTranslator

	private Utility.AtFloorArray mAtFloor;

	private ReadableCanMailbox networkLevelUp;
	private ReadableCanMailbox networkLevelDown;
	private LevelingCanPayloadTranslator mLevelUp;
	private LevelingCanPayloadTranslator mLevelDown;

	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;

	private Utility.DoorClosedArray mDoorClosedArrayFront;
	private Utility.DoorClosedArray mDoorClosedArrayBack;

	private ReadableCanMailbox networkEmergencyBrake;
	private SafetySensorCanPayloadTranslator mEmergencyBrake;

	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	private ReadableCanMailbox networkHoistwayLimit;
	private HoistwayLimitSensorCanPayloadTranslator mHoistwayLimit;

	private ReadableCanMailbox networkCarWeight;
	private CarWeightCanPayloadTranslator mCarWeight;

	private static final double LEVEL_SPEED = DriveObject.LevelingSpeed; // in
																			// m/s
	private static final double SLOW_SPEED = DriveObject.SlowSpeed; // in m/s
	private static final double FAST_SPEED = DriveObject.FastSpeed; // in m/s
	private static final double ACCELERATION = DriveObject.Acceleration; // in
																			// m/s^2
	private static final double DECELERATION = DriveObject.Deceleration; // in
																			// m/s^2
	private static final double ONETOMILLI = 1000.0;

	// enumerate states
	private static enum State {
		STATE_STOP, STATE_SLOW_UP, STATE_FAST_UP, STATE_SLOW_DOWN, STATE_FAST_DOWN, STATE_LEVEL_UP, STATE_LEVEL_DOWN, STATE_EMERGENCY
	}

	private static enum Commit {
		REACHED, NOTREACHED
	}

	// store the period for the controller
	private SimTime period;
	private State currentState;
	private double allowance;

	public DriveControl(SimTime period, boolean verbose) {
		super("DriveControl", verbose);

		// initialize state
		this.period = period;
		this.currentState = State.STATE_STOP;
		this.allowance = 2 * (100 + FAST_SPEED * 6
				* period.getFracMilliseconds());
		this.desiredFloor = MessageDictionary.NONE;
		this.currentFloor = 1;
		this.DesiredDirection = Direction.UP;

		// initialize physical state
		localDriveSpeed = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(localDriveSpeed);
		localDrive = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localDrive, period);

		// initialize network interface
		networkDriveSpeedOut = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);

		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeedOut);
		canInterface.sendTimeTriggered(networkDriveSpeedOut, period);

		mAtFloor = new Utility.AtFloorArray(canInterface);

		networkLevelUp = CanMailbox
				.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Direction.UP));
		networkLevelDown = CanMailbox
				.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID
						+ ReplicationComputer
								.computeReplicationId(Direction.DOWN));
		mLevelUp = new LevelingCanPayloadTranslator(networkLevelUp,
				Direction.UP);
		mLevelDown = new LevelingCanPayloadTranslator(networkLevelDown,
				Direction.DOWN);
		canInterface.registerTimeTriggered(networkLevelUp);
		canInterface.registerTimeTriggered(networkLevelDown);

		networkCarLevelPosition = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(
				networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);

		mDoorClosedArrayBack = new Utility.DoorClosedArray(Hallway.BACK,
				canInterface);
		mDoorClosedArrayFront = new Utility.DoorClosedArray(Hallway.FRONT,
				canInterface);

		networkEmergencyBrake = CanMailbox
				.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mEmergencyBrake = new SafetySensorCanPayloadTranslator(
				networkEmergencyBrake);
		canInterface.registerTimeTriggered(networkEmergencyBrake);

		networkDesiredFloor = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(
				networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);

		networkHoistwayLimit = CanMailbox
				.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID);
		mHoistwayLimit = new HoistwayLimitSensorCanPayloadTranslator(
				networkHoistwayLimit, Direction.UP);
		canInterface.registerTimeTriggered(networkHoistwayLimit);

		networkCarWeight = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
		canInterface.registerTimeTriggered(networkCarWeight);

		timer.start(period);

	}

	// CarLevelPosition in millimeter, speed in m/s
	private Commit commitPoint(int floor, int CarLevelPosition, double speed,
			Direction d) {
		double floorPosition = (floor - 1) * 5 * ONETOMILLI;
		double brakeDistance;
		brakeDistance = speed * speed / (2 * DECELERATION) * ONETOMILLI;
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

	@Override
	public void timerExpired(Object callbackData) {
		State oldState = currentState;
		switch (currentState) {
		case STATE_STOP:
			stateStop();
			break;
		case STATE_SLOW_UP:
			stateSlowUp();
			break;
		case STATE_SLOW_DOWN:
			stateSlowDown();
			break;
		case STATE_FAST_UP:
			stateFastUp();
			break;
		case STATE_FAST_DOWN:
			stateFastDown();
			break;
		case STATE_LEVEL_UP:
			stateLevelUp();
			break;
		case STATE_LEVEL_DOWN:
			stateLevelDown();
			break;
		case STATE_EMERGENCY:
			stateEmergency();
			break;
		default:
			throw new RuntimeException("State " + currentState
					+ " was not recognized.");
		}

		if (currentState == oldState)
			log("No transitions:", currentState);
		else
			log("Transitions:", oldState, "->", currentState);

		setState(STATE_KEY, currentState.toString());
		timer.start(period);
	}

	private void stateFastUp() {
		// state actions
		localDrive.set(Speed.FAST, Direction.UP);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.UP;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();
		// #transition 'T6.9'
		if (SLOW_SPEED < localDriveSpeed.speed()
				&& localDriveSpeed.speed() <= FAST_SPEED
				&& commitPoint(desiredFloor, mCarLevelPosition.getPosition(),
						localDriveSpeed.speed(), localDriveSpeed.direction()) == Commit.REACHED)
			currentState = State.STATE_SLOW_UP;
		// #transition 'T6.13.6'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateFastDown() {
		// state actions
		localDrive.set(Speed.FAST, Direction.DOWN);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.DOWN;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();
		// #transition 'T6.11'
		if (SLOW_SPEED < localDriveSpeed.speed()
				&& localDriveSpeed.speed() <= FAST_SPEED
				&& commitPoint(desiredFloor, mCarLevelPosition.getPosition(),
						localDriveSpeed.speed(), localDriveSpeed.direction()) == Commit.REACHED)
			currentState = State.STATE_SLOW_DOWN;
		// #transition 'T6.13.7'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateStop() {
		// state actions
		localDrive.set(Speed.STOP, Direction.STOP);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.STOP;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();

		// #transition 'T6.1'
		if (commitPoint(desiredFloor, mCarLevelPosition.getPosition(),
				localDriveSpeed.speed(), localDriveSpeed.direction()) == Commit.NOTREACHED
				&& desiredFloor != MessageDictionary.NONE
				&& desiredFloor > currentFloor
				&& mDoorClosedArrayFront.getBothClosed() == true
				&& mDoorClosedArrayBack.getBothClosed() == true
				&& mCarWeight.getWeight() < Elevator.MaxCarCapacity)
			currentState = State.STATE_SLOW_UP;
		// #transition 'T6.2'
		else if (commitPoint(desiredFloor, mCarLevelPosition.getPosition(),
				localDriveSpeed.speed(), localDriveSpeed.direction()) == Commit.NOTREACHED
				&& desiredFloor != MessageDictionary.NONE
				&& desiredFloor < currentFloor
				&& mDoorClosedArrayFront.getBothClosed() == true
				&& mDoorClosedArrayBack.getBothClosed() == true
				&& mCarWeight.getWeight() < Elevator.MaxCarCapacity)
			currentState = State.STATE_SLOW_DOWN;
		// #transition 'T6.6'

		else if (mLevelUp.getValue() == false // ||mCarLevelPosition.getPosition()<0)
				&& localDriveSpeed.speed() == 0
				&& localDriveSpeed.direction() == Direction.STOP)
			currentState = State.STATE_LEVEL_UP;
		// #transition 'T6.8'
		else if (mLevelDown.getValue() == false // mCarLevelPosition.getPosition()>0)
				&& localDriveSpeed.speed() == 0
				&& localDriveSpeed.direction() == Direction.STOP)
			currentState = State.STATE_LEVEL_DOWN;
		// #transition 'T6.13.1'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateSlowUp() {
		// state actions
		localDrive.set(Speed.SLOW, Direction.UP);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.UP;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();

		// #transition 'T6.3'
		if (localDriveSpeed.speed() <= SLOW_SPEED
				&& currentFloor == mDesiredFloor.getFloor())
			currentState = State.STATE_LEVEL_UP;
		// #transition 'T6.10'
		if (Double.compare(localDriveSpeed.speed(), (SLOW_SPEED)) >= 0
				&& desiredFloor != MessageDictionary.NONE
				&& commitPoint(desiredFloor, mCarLevelPosition.getPosition(),
						localDriveSpeed.speed(), localDriveSpeed.direction()) == Commit.NOTREACHED)
			currentState = State.STATE_FAST_UP;
		// #transition 'T6.13.2'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateSlowDown() {
		// state actions
		localDrive.set(Speed.SLOW, Direction.DOWN);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.DOWN;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();

		// #transition 'T6.4'
		if (localDriveSpeed.speed() <= SLOW_SPEED
				&& (currentFloor == mDesiredFloor.getFloor() || mDesiredFloor
						.getFloor() == MessageDictionary.NONE))
			currentState = State.STATE_LEVEL_DOWN;
		// #transition 'T6.12'
		if (Double.compare(localDriveSpeed.speed(), (SLOW_SPEED)) >= 0
				&& desiredFloor != MessageDictionary.NONE
				&& commitPoint(desiredFloor, mCarLevelPosition.getPosition(),
						localDriveSpeed.speed(), localDriveSpeed.direction()) == Commit.NOTREACHED)
			currentState = State.STATE_FAST_DOWN;
		// #transition 'T6.13.3'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateLevelUp() {
		// state actions
		localDrive.set(Speed.LEVEL, Direction.UP);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.UP;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();

		// #transition 'T6.5'
		if (mLevelUp.getValue() == true// || mCarLevelPosition.getPosition()>=0)
				&& localDriveSpeed.speed() <= LEVEL_SPEED)
			currentState = State.STATE_STOP;
		// #transition 'T6.13.4'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateLevelDown() {
		// state actions
		localDrive.set(Speed.LEVEL, Direction.DOWN);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.STOP;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();

		// #transition 'T6.7'
		if (mLevelDown.getValue() == true // ||
											// mCarLevelPosition.getPosition()<=0)
				&& localDriveSpeed.speed() <= LEVEL_SPEED)
			currentState = State.STATE_STOP;
		// #transition 'T6.13.5'
		if (mEmergencyBrake.getValue() == true)
			currentState = State.STATE_EMERGENCY;
	}

	private void stateEmergency() {
		// state actions
		localDrive.set(Speed.STOP, Direction.STOP);
		mDriveSpeed.setSpeed(localDriveSpeed.speed());
		mDriveSpeed.setDirection(localDriveSpeed.direction());
		DesiredDirection = Direction.STOP;
		desiredFloor = mDesiredFloor.getFloor();
		currentFloor = mAtFloor.getCurrentFloor();
	}

}
