/*
 * 18649 - Fall 2013
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.framework.Direction;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;

/**
 *	Runtime Monitor for Project 11.
 */
public class Proj11RuntimeMonitor extends RuntimeMonitor
{

	// Variables
	protected int currentFloor = MessageDictionary.NONE;

	// State machines to be updated
	protected RT6StateMachine	rt6		= new RT6StateMachine();
	protected RT7StateMachine	rt7		= new RT7StateMachine();
	protected RT81StateMachine 	rt81 	= new RT81StateMachine();
	protected RT82StateMachine 	rt82 	= new RT82StateMachine();
	protected RT83StateMachine 	rt83 	= new RT83StateMachine();
	protected RT9StateMachine 	rt9 	= new RT9StateMachine();
	protected RT10StateMachine 	rt10_f 	= new RT10StateMachine(Hallway.FRONT);
	protected RT10StateMachine 	rt10_b 	= new RT10StateMachine(Hallway.BACK);

	@Override
	public void timerExpired(Object callbackData)
	{
		// gets the compiler to be quiet lol
    }

	@Override
	/**
	 *	whenever DriveSpeed value is changed
	 *	RT6 and RT9 receives this msg
	 */
    public void receive(ReadableDriveSpeedPayload msg)
    {
		checkStateMachine(rt6);    	
		checkStateMachine(rt9);
    }

    @Override
    /**
	 *	update current floor whenever any AtFloor is true
	 */
    public void receive(ReadableAtFloorPayload msg)
    {
        updateCurrentFloor(msg);
    }

    @Override
    /**
	* update lantern 
	* RT8.1, RT8.2, RT8.3 receives this msg
	 */
	public void receive(ReadableCarLanternPayload msg)
	{
		checkStateMachine(rt81);
		checkStateMachine(rt82);
		checkStateMachine(rt83);
        	
    }

	@Override
	/**
	 *	Whenever a DoorMotor is updated, do this.
	 *	Updates the state machines that primarily use the DoorMotors
	 *	@param msg latest DoorMotor message
	 */
    public void receive(ReadableDoorMotorPayload msg) {
		checkStateMachine(rt10_f);
		checkStateMachine(rt10_b);
    }

    @Override
    protected String[] summarize()
    {
    	String[] summary = new String[7];
		summary[0] = "RT6: stops at floors for which there are no pending calls " + rt6.getViolations();
		summary[1] = "RT7: open doors at hallways for which there are no pending calls " + rt7.getViolations();
    	summary[2] = "RT8.1: no lantern lit when any door is open at a hallway and there are any pending calls at any other floor(s) " + rt81.getViolations();
    	summary[3] = "RT8.2: direction indicated by lantern changes while the doors are open " + rt82.getViolations();
    	summary[4] = "RT8.3: car services other direction when one of the car lanterns is lit " + rt83.getViolations();
    	summary[5] = "RT9: drive speed not commaded to maximum degree practicable " + rt9.getViolations();
		summary[6] = "RT10: no door reversal occurs before the doors are commanded to nudge " + (rt10_f.getViolations() + rt10_b.getViolations());
    	return summary;
    }


    protected void checkStateMachine(StateMachine machine)
    {
    	machine.update();
    	String message = machine.getWarning();
    	if(message != null)
        	warning(message);
    }

	private static enum RT6State
	{
		MOVING,
		VALID_STOP,
		INVALID_STOP,
	}

	/**
	 *	State machine for RT-6
	 */
	private class RT6StateMachine extends StateMachine
	{
		// initial state VALID_STOP
		private RT6State state = RT6State.VALID_STOP;

		public static final double LEVEL_SPEED = 0.10;

		@Override
		protected String warningMessage()
		{
			return "RT6: stops at floors for which there are no pending calls " + currentFloor + ".";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case MOVING:	moving();	break;
				case VALID_STOP:	validStop();	break;
				case INVALID_STOP:	invalidStop();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void moving()
		{
			releaseWarning();
			
			if(driveActualSpeed.speed() <= LEVEL_SPEED && hasCall(currentFloor)) {
				state = RT6State.VALID_STOP;
			}
			
			if(driveActualSpeed.speed() <= LEVEL_SPEED && !hasCall(currentFloor)) {
				state = RT6State.INVALID_STOP;
			}

		}

		private void validStop()
		{
			releaseWarning();

			if(driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT6State.MOVING;
			}

		}

		private void invalidStop()
		{
			setWarning();

			if(driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT6State.MOVING;
			}
		}

	} 


	private static enum RT7State
	{
		CLOSED,
		VALID_OPEN,
		INVALID_OPEN,
	}

	/**
	 *	State machine for RT-7
	 */
	private class RT7StateMachine extends StateMachine
	{
		// initial state
		private RT7State state = RT7State.CLOSED;

		@Override
		protected String warningMessage()
		{
			return "RT7: open doors at hallways for which there are no pending calls " + currentFloor + ".";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case CLOSED:	closed();	break;
				case VALID_OPEN:		open();		break;
				case INVALID_OPEN:	badOpen();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void closed()
		{
			releaseWarning();

			
			if(anyDoorsOpen() && hasCall(currentFloor)) {
				state = RT7State.VALID_OPEN;
			}
			
			if(anyDoorsOpen() && !hasCall(currentFloor)) {
				state = RT7State.INVALID_OPEN;
			}

		}

		private void open()
		{
			releaseWarning();

			
			if(!anyDoorsOpen()) {
				state = RT7State.CLOSED;
			}

		}

		private void badOpen()
		{
			setWarning();

			
			if(!anyDoorsOpen()) {
				state = RT7State.CLOSED;
			}
		}

	}


	private static enum RT81State
	{
		LANTERN_IDLE,
		LANTERN_ON,
		LANTERN_OFF
	}

	/**
	 *	State machine for R-T8.1
	 */
	private class RT81StateMachine extends StateMachine
	{

		public static final int LANTERN_CONTROL_TIME = 200;
		public static final int LANTERN_DOUBLE_GLOBAL_TICK = 2*LANTERN_CONTROL_TIME;
		
		// initial state: no call, lantern is idle
		private RT81State state = RT81State.LANTERN_IDLE;
		private IntervalTimer timer = new IntervalTimer();

		@Override
		protected String warningMessage()
		{
			return "RT8.1: no lantern lit when any door is open at a hallway and there are any pending calls at any other floor(s) " + currentFloor + ".";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case LANTERN_IDLE:	correctLanterns();	break;
				case LANTERN_ON:	callMadeDelay();	break;
				case LANTERN_OFF:	wrongLanterns();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void correctLanterns()
		{
			releaseWarning();

			//Timer should not be running in this state
			timer.stop();

			if(anyDoorsOpen() && anyOtherCall(currentFloor) &&
					(lanternLit(Direction.UP) || lanternLit(Direction.DOWN)) ) {
				state = RT81State.LANTERN_ON;
			} else if(anyDoorsOpen() && anyOtherCall(currentFloor) && ((!lanternLit(Direction.UP)) && (!lanternLit(Direction.DOWN)))) {
				state = RT81State.LANTERN_OFF;
			}

		}

		private void callMadeDelay()
		{
			releaseWarning();

			//Timer should be running in this state
			timer.start(new SimTime(LANTERN_DOUBLE_GLOBAL_TICK, SimTimeUnit.MILLISECOND));
				
			if(!timer.isExpired() && (!anyDoorsOpen()))
				state = RT81State.LANTERN_IDLE;

		}

		private void wrongLanterns()
		{
			setWarning();

			//Timer should not be running in this state
			timer.stop();

			if(!anyDoorsOpen())
				state = RT81State.LANTERN_IDLE;
		}

	} 

	private static enum RT82State
	{		
		NO_DIRECTION,
		VALID_DIRECTION_UP,
		VALID_DIRECTION_DOWN,
		INVALID_DIRECTION
	}

	/**
	 *	State machine for R-T8.2
	 */
	private class RT82StateMachine extends StateMachine
	{
		
		private RT82State state = RT82State.NO_DIRECTION;

		@Override
		protected String warningMessage()
		{
			return "RT8.2: direction indicated by lantern changes while the doors are open " + currentFloor + ".";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case NO_DIRECTION:	directionNone();	break;
				case VALID_DIRECTION_UP:	validDirectionUp();	break;
				case VALID_DIRECTION_DOWN:	validDirectionDown(); break;
				case INVALID_DIRECTION:	invalidDirection();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void directionNone()
		{
			releaseWarning();

			if(anyDoorsOpen() && (lanternLit(Direction.UP)))
				state = RT82State.VALID_DIRECTION_UP;
			
			else if(anyDoorsOpen() && (lanternLit(Direction.DOWN)))
				state = RT82State.VALID_DIRECTION_DOWN;

		}
		
		private void validDirectionUp() {
			releaseWarning();
			
			if(!anyDoorsOpen()) {
				state = RT82State.NO_DIRECTION;
			}
			else if(anyDoorsOpen() && !lanternLit(Direction.UP)) {
				state = RT82State.INVALID_DIRECTION;
			}
		}
		
		
		
		private void validDirectionDown() {
			releaseWarning();
			
			if(!anyDoorsOpen()) {
				state = RT82State.NO_DIRECTION;
			}
			else if(anyDoorsOpen() && !lanternLit(Direction.DOWN)) {
				state = RT82State.INVALID_DIRECTION;
			}
		}


		private void invalidDirection()
		{
			//This state produces a warning
			setWarning();

			if(!anyDoorsOpen())
				state = RT82State.NO_DIRECTION;
		}

	}


	private static enum RT83State
	{
		DIRECTION_NONE,
		DIRECTION_DOWN,
		DIRECTION_UP,
		DIRECTION_WRONG
	}

	/**
	 *	State machine for R-T8.3
	 */
	private class RT83StateMachine extends StateMachine
	{
		
		private RT83State state = RT83State.DIRECTION_NONE;

		@Override
		protected String warningMessage()
		{
			return "RT8.3: car services other direction when one of the car lanterns is lit " + currentFloor + ".";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case DIRECTION_NONE:	directionNone();	break;
				case DIRECTION_UP:		directionUp(); 		break;
				case DIRECTION_DOWN:	directionDown();	break;
				case DIRECTION_WRONG:	directionWrong();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void directionNone()
		{
			releaseWarning();

			if(!allDoorsClosed() && lanternLit(Direction.DOWN))
				state = RT83State.DIRECTION_DOWN;

			else if(!allDoorsClosed() && lanternLit(Direction.UP))
				state = RT83State.DIRECTION_UP;
		}

		private void directionUp()
		{
			releaseWarning();

			boolean higherCalls = checkDirectionForCalls(currentFloor,Direction.UP);

			if(!allDoorsClosed() && !lanternLit(Direction.UP))
				state = RT83State.DIRECTION_NONE;
			
			else if(!allDoorsClosed() && lanternLit(Direction.UP) && mDesiredFloor.getFloor() < currentFloor && higherCalls)
				state = RT83State.DIRECTION_WRONG;
		}

		private void directionDown()
		{
			releaseWarning();

			boolean lowerCalls = checkDirectionForCalls(currentFloor,Direction.DOWN);
			
			
			if(!allDoorsClosed() && !lanternLit(Direction.DOWN))
				state = RT83State.DIRECTION_NONE;
			
			else if(!allDoorsClosed() && lanternLit(Direction.DOWN) && mDesiredFloor.getFloor() > currentFloor && lowerCalls)
				state = RT83State.DIRECTION_WRONG;
		}

		private void directionWrong()
		{
			// warning
			setWarning();
			
			if(allDoorsClosed())
				state = RT83State.DIRECTION_NONE;
		}

	}


	private static enum RT9State
	{
		CURRENT_FLOOR,
		NEXT_FLOOR_EFFICIENT,
		NEXT_FLOOR_INEFFICIENT
	}

	/**
	 *	State machine for R-T9
	 */
	private class RT9StateMachine extends StateMachine
	{
		public static final int DRIVE_CONTROL_TIME = 10;
		public static final double SLOW_SPEED = 0.25;
		
		private RT9State state = RT9State.CURRENT_FLOOR;
		private IntervalTimer timer = new IntervalTimer();
		private boolean fastSpeed = false;

		@Override
		public void update() {
			switch(state) {
				case CURRENT_FLOOR:	currentFloor();	break;
				case NEXT_FLOOR_EFFICIENT:	nextFloor();	break;
				case NEXT_FLOOR_INEFFICIENT:	inefficientNextFloor();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
			
		}
		
		private void currentFloor() {
			releaseWarning();
			timer.start(new SimTime(3*DRIVE_CONTROL_TIME, SimTimeUnit.MILLISECOND));
			
			if(driveActualSpeed.speed() > SLOW_SPEED) {
				fastSpeed = true;
			}
			
			if(currentFloor != MessageDictionary.NONE && fastSpeed == true && driveActualSpeed.speed() == 0) {
				state = RT9State.NEXT_FLOOR_EFFICIENT;
			}
			
			else if(timer.isExpired() && currentFloor != MessageDictionary.NONE && fastSpeed == false && driveActualSpeed.speed() == 0) {
				state = RT9State.NEXT_FLOOR_INEFFICIENT;
			}
		}
		
		private void nextFloor() {
			releaseWarning();
			
			if(driveActualSpeed.speed() != 0) {
				fastSpeed = false;
				state = RT9State.CURRENT_FLOOR;
			}
		}
		
		private void inefficientNextFloor() {
			// warning
			setWarning();
	
			if(driveActualSpeed.speed() != 0) {
				fastSpeed = false;
				state = RT9State.CURRENT_FLOOR;
			}
			
		}

		@Override
		protected String warningMessage() {
			return "RT9: drive speed not commaded to maximum degree practicable";
		}

	} 

	private static enum RT10State
	{
		MOVING,
		STOPPED,
		REVERSED,
		BAD_NUDGE,
	}

	/**
	 *	State machine for RT-10
	 */
	private class RT10StateMachine extends StateMachine
	{
		
		private RT10State state = RT10State.STOPPED;
		private Hallway hall;

		public static final double LEVEL_SPEED = 0.10;

		public RT10StateMachine(Hallway hall) {
			this.hall = hall;
		}
		@Override
		protected String warningMessage()
		{
			return "RT10: no door reversal occurs before the doors are commanded to nudge";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case MOVING:	moving();	break;
				case STOPPED:	stopped();	break;
				case REVERSED:	reversed(); break;
				case BAD_NUDGE:	badNudge();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void moving()
		{
			releaseWarning();

			if(driveActualSpeed.speed() <= LEVEL_SPEED) {
				state = RT10State.STOPPED;
			}

		}

		private void stopped()
		{
			releaseWarning();

			if(driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT10State.MOVING;
			}
			else if(driveActualSpeed.speed() <= LEVEL_SPEED && anyDoorReversing(hall)) {
				state = RT10State.REVERSED;
			}
			else if(driveActualSpeed.speed() <= LEVEL_SPEED && anyDoorNudging(hall)) {
				state = RT10State.BAD_NUDGE;
			}

		}

		private void reversed()
		{
			releaseWarning();

			if(allDoorsClosed() && driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT10State.MOVING;
			}

		}

		private void badNudge()
		{
			setWarning();

			if(allDoorsClosed() && driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT10State.MOVING;
			}
		}

	} 


	/**
	 *	Abstract Class representing a state machine
	 *	Contains abstract method for being updated to be called when this state
	 *			should update
	 *	Contains methods to simplify setting a warning for the runtime monitor to detect
	 */
	private abstract class StateMachine
	{

		protected boolean inWarningState = false;
		protected boolean hasWarningToReport = false;
		protected int numViolations = 0;


		/**
		 *	Method to be called when the runtime monitor wants this statechart
		 *		to update. Should contain state machine to be implemented.
		 */
		public abstract void update();

		/**
		 *	Method to set the warning message when a warning occurs.
		 *	@return warning message to send
		 */
		protected abstract String warningMessage();

		/**
		 *	Method to set a flag so the runtime monitor will know that this state
		 *		machine is in a warning state
		 */
		protected void setWarning()
		{
			if(!inWarningState) {
				hasWarningToReport = true;
				inWarningState = true;
			}
		}

		/**
		 *	Method to set a flag so the runtime monitor will know that this state
		 *		machine is not in a warning state.
		 */
		protected void releaseWarning()
		{
			// If the main monitor hasn't noticed the warning, 
			if(!hasWarningToReport)
				inWarningState = false;
		}

		/**
		 *	Gets warning message from state machine.
		 *	@return warning message
		 */
		public String getWarning()
		{
			//If there is a warning, return error message and note
			if(hasWarningToReport) {
				numViolations++;
				hasWarningToReport = false;
				return warningMessage();
			}
			else {
				return null;
			}
		}

		/**
		 *	Gets the number of violations reported from this state machine
		 */
		public int getViolations()
		{
			return numViolations;
		}
	}

	/**
     * Times process and determines whether a specified interval is exceeded
     */
    private class IntervalTimer {

        private boolean isRunning;
        private SimTime startTime;
        private SimTime interval;

        /**
         *	Generic constructor
         */
        public IntervalTimer()
        {
        	this.interval = null;
        	this.startTime = null;
        	this.isRunning = false;
        }

        /**
         *	Starts timer. If the timer has already been started, this function does nothing.
         *	@param interval SimTime for this interval timer to last before becoming expired.
         */
        public void start(SimTime interval)
        {
            if (!isRunning) {
            	this.interval = interval;
                startTime = Harness.getTime();
                isRunning = true;
            }
        }

        /**
         *	Stops and resets timer. If the timer was not running this function does nothing.
         */
        public void stop()
        {
        	if (isRunning) {
        		startTime = null;
        		isRunning = false;
        	}
        }

		/**
		 *	determines wheter the length of time that the timer has been running
		 *		is greater than the specified time interval
		 *	@return true if the timer has been running for longer than the specified time interval
		 */
		public boolean isExpired()
		{
			if(interval == null) {
				return true;
			}
			else {
				return interval.isLessThan(SimTime.subtract(Harness.getTime(), startTime));
			}
		}

    }

    /**
     *	updates the current floor with latest AtFloor Payload
     *	Stolen frim SampleDispatcherMonitor.java lines 97-110
     *	@param lastAtFloor Payload for last received AtFloor message
     */
	private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
        if (lastAtFloor.getFloor() == currentFloor) {
            //the atFloor message is for the currentfloor, so check both sides to see if they a
            if (!atFloors[lastAtFloor.getFloor()-1][Hallway.BACK.ordinal()].value() && !atFloors[lastAtFloor.getFloor()-1][Hallway.FRONT.ordinal()].value()) {
                //both sides are false, so set to NONE
                currentFloor = MessageDictionary.NONE;
            }
            //otherwise at least one side is true, so leave the current floor as is
        } else {
            if (lastAtFloor.value()) {
                currentFloor = lastAtFloor.getFloor();
            }
        }
    }

	/**
	 *	Helper Functions
	 */
	private boolean anyOtherCall(int excludedFloor)
	{
		for(int floor = 1; floor < Elevator.numFloors; floor++)
		{
			if(floor != excludedFloor && hasCall(floor))
				return true;
		}

		return false;
	}

	private boolean checkDirectionForCalls(int floor, Direction dir)
	{
		if( floor != MessageDictionary.NONE && dir == Direction.UP ) {
			for (int i = Elevator.numFloors; i > floor; i--) {
				if(hasCall(i))
					return true;
			}
		}
		else if ( floor != MessageDictionary.NONE && dir == Direction.DOWN ) {
			for (int i = 1; i < floor; i++) {
				if(hasCall(i))
					return true;
			}
		}

		return false;
	}	

	private boolean hasCall(int floor)
	{
		floor -= 1;
		return 	   carLights[floor][Hallway.FRONT.ordinal()].lighted()
				|| carLights[floor][Hallway.BACK.ordinal() ].lighted()
				|| hallLights[floor][Hallway.FRONT.ordinal()][Direction.UP.ordinal()  ].lighted()
				|| hallLights[floor][Hallway.FRONT.ordinal()][Direction.DOWN.ordinal()].lighted()
				|| hallLights[floor][Hallway.BACK.ordinal() ][Direction.UP.ordinal()  ].lighted()
				|| hallLights[floor][Hallway.BACK.ordinal() ][Direction.DOWN.ordinal()].lighted();
	}

	private boolean lanternLit(Direction dir) {
		return carLanterns[dir.ordinal()].lighted();
	}

//	private boolean allDoorsOpen(Hallway hall)
//	{
//		return doorOpeneds[hall.ordinal()][Side.LEFT.ordinal() ].isOpen()
//			&& doorOpeneds[hall.ordinal()][Side.RIGHT.ordinal()].isOpen();
//	}

	private boolean anyDoorsOpen()
	{
		return anyDoorsOpen(Hallway.FRONT) || anyDoorsOpen(Hallway.BACK);
	}

	private boolean anyDoorsOpen(Hallway hall)
	{
		return doorOpeneds[hall.ordinal()][Side.LEFT.ordinal() ].isOpen()
			|| doorOpeneds[hall.ordinal()][Side.RIGHT.ordinal()].isOpen();
	}
	
	private boolean anyDoorReversing(Hallway hall)
	{
		return doorReversals[hall.ordinal()][Side.LEFT.ordinal() ].isReversing()
			|| doorReversals[hall.ordinal()][Side.RIGHT.ordinal()].isReversing();
	}

	private boolean anyDoorNudging(Hallway hall)
	{
		return doorMotors[hall.ordinal()][Side.LEFT.ordinal() ].command() == DoorCommand.NUDGE
			|| doorMotors[hall.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE;
	}
	
	private boolean allDoorsClosed() {
		return allDoorsClosed(Hallway.FRONT)
			&& allDoorsClosed(Hallway.BACK);
	}
	
	private boolean allDoorsClosed(Hallway hall) {
		return doorCloseds[hall.ordinal()][Side.LEFT.ordinal() ].isClosed()
			&& doorCloseds[hall.ordinal()][Side.RIGHT.ordinal()].isClosed();
	}
}