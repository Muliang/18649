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
import simulator.framework.Speed;
//import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;
//import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
//import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
//import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
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

//	@Override
//	/**
//	 *	Whenever a DoorMotor is updated, do this.
//	 *	Updates the state machines that primarily use the DoorMotors
//	 *	@param msg latest DoorMotor message
//	 */
//    public void receive(ReadableDoorMotorPayload msg) {
//		checkStateMachine(rt10_f);
//		checkStateMachine(rt10_b);
//    }

    @Override
    /**
     *	Gives array of strings that summarize the warnings generated 
     *	in the runtime monitor while it was running
     *	@return Array of summarized warnings
     */
    protected String[] summarize()
    {
    	String[] summary = new String[7];
		summary[0] = "RT6: Times a stop occured without a pending call: " + rt6.getViolations();
		summary[1] = "RT7: Times a door open occurred without a pending call: " + rt7.getViolations();
    	summary[2] = "RT8.1: Times a call occured without a car lantern: " + rt81.getViolations();
    	summary[3] = "RT8.2: Times car lanterns changed direction: " + rt82.getViolations();
    	summary[4] = "RT8.3: Times car traveled in different direction than indicated: " + rt83.getViolations();
    	summary[5] = "RT9:   Times Speed not commaded to maximum degree practicable: " + rt9.getViolations();
		summary[6] = "RT10: Times a door reversal occured without a nudge: " + (rt10_f.getViolations() + rt10_b.getViolations());
    	return summary;
    }

    /**
     *	Runs the specified state machine and checks for warnings generated
     *	@param machine state machine to be run
     */
    protected void checkStateMachine(StateMachine machine)
    {
    	machine.update();
    	String message = machine.getWarning();
    	if(message != null)
        	warning(message);
    }

	//Java doesn't like enums in private classes.
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
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT6: The car stopped at a floor for which there are no pending calls.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT6: The car stopped with no pending calls at " + currentFloor + ".";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case MOVING:	moving();	break;
				case VALID_STOP:	stopCall();	break;
				case INVALID_STOP:	badStop();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void moving()
		{
			//This state does not set a warning
			releaseWarning();

			
			if(driveActualSpeed.speed() <= LEVEL_SPEED && hasCall(currentFloor)) {
				state = RT6State.VALID_STOP;
			}
			
			if(driveActualSpeed.speed() <= LEVEL_SPEED && !hasCall(currentFloor)) {
				state = RT6State.INVALID_STOP;
			}

		}

		private void stopCall()
		{
			//This state does not set a warning
			releaseWarning();

			if(driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT6State.MOVING;
			}

		}

		private void badStop()
		{
			//This state sets a warning
			setWarning();

			//transition RT6-3
			if(driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT6State.MOVING;
			}
		}

	} 

	//Java doesn't like enums in private classes.
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
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT7: The car opened doors at a floor for which there are no pending calls.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT7: The car opened doors with no pending calls at " + currentFloor + ".";
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
			//This state does not set a warning
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
			//This state does not set a warning
			releaseWarning();

			
			if(!anyDoorsOpen()) {
				state = RT7State.CLOSED;
			}

		}

		private void badOpen()
		{
			//This state sets a warning
			setWarning();

			
			if(!anyDoorsOpen()) {
				state = RT7State.CLOSED;
			}
		}

	}

    //Java doesn't like enums in private classes.
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
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT8.1: The Lanterns failed to indicate direction at currentFloor.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT8.1: The Lanterns failed to indicate direction at " + currentFloor + ".";
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
			//This state does not set a warning
			releaseWarning();

			//Timer should not be running in this state
			timer.stop();

			// car stopped at certain floor
			if(anyDoorsOpen() && anyOtherCall(currentFloor) &&
					(lanternLit(Direction.UP) || lanternLit(Direction.DOWN)) ) {
				state = RT81State.LANTERN_ON;
			} else if(anyDoorsOpen() && anyOtherCall(currentFloor) && ((!lanternLit(Direction.UP)) && (!lanternLit(Direction.DOWN)))) {
				state = RT81State.LANTERN_OFF;
			}

		}

		private void callMadeDelay()
		{
			//This state does not set a warning
			releaseWarning();

			//Timer should be running in this state
			timer.start(new SimTime(LANTERN_DOUBLE_GLOBAL_TICK, SimTimeUnit.MILLISECOND));
				
			if(!timer.isExpired() && (!anyDoorsOpen()))
				state = RT81State.LANTERN_IDLE;

		}

		private void wrongLanterns()
		{
			//This state sets a warning
			setWarning();

			//Timer should not be running in this state
			timer.stop();

			//transition RT81-4
			if(!anyDoorsOpen())
				state = RT81State.LANTERN_IDLE;
		}

	} 

	//Java doesn't like enums in private classes.
	private static enum RT82State
	{
		DIRECTION_NONE,
		DIRECTION_UP,
		DIRECTION_DOWN,
		DIRECTION_CHANGE
	}

	/**
	 *	State machine for R-T8.2
	 */
	private class RT82StateMachine extends StateMachine
	{
		
		private RT82State state = RT82State.DIRECTION_NONE;

		@Override
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT8.2: The Lanterns changed indicated direction on floor currentFloor.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT8.2: The Lanterns changed indicated direction on floor " + currentFloor + ".";
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
				case DIRECTION_CHANGE:	directionChange();	break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void directionNone()
		{
			//This state does not produce a warning
			releaseWarning();

			//transition RT82-1
			if(anyDoorsOpen() && lanternLit(Direction.UP))
				state = RT82State.DIRECTION_UP;
			//transition RT82-2
			else if(anyDoorsOpen() && lanternLit(Direction.DOWN))
				state = RT82State.DIRECTION_DOWN;
		}

		private void directionUp()
		{
			//This state does not produce a warning
			releaseWarning();

			//transition RT82-3
			if(!anyDoorsOpen())
				state = RT82State.DIRECTION_NONE;
			//transition RT82-6
			else if(!lanternLit(Direction.UP))
				state = RT82State.DIRECTION_CHANGE;
		}

		private void directionDown()
		{
			//This state does not produce a warning
			releaseWarning();

			//transition RT82-4
			if(!anyDoorsOpen())
				state = RT82State.DIRECTION_NONE;
			//transition RT82-7
			else if(!lanternLit(Direction.DOWN))
				state = RT82State.DIRECTION_CHANGE;
		}

		private void directionChange()
		{
			//This state produces a warning
			setWarning();

			//transition RT82-5
			if(!anyDoorsOpen())
				state = RT82State.DIRECTION_NONE;
		}

	}


	//Java doesn't like enums in private classes.
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
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT8.3: The car traveled in different direction than indicated at floor currentFloor.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT8.3: The car traveled in different direction than indicated at floor " + currentFloor + ".";
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
			//This state does not produce a warning
			releaseWarning();

			//transition RT83-1
			if(!allDoorsClosed() && lanternLit(Direction.DOWN))
				state = RT83State.DIRECTION_DOWN;
			//transition RT83-2
			else if(!allDoorsClosed() && lanternLit(Direction.UP))
				state = RT83State.DIRECTION_UP;
		}

		private void directionUp()
		{
			//This state does not produce a warning
			releaseWarning();

			boolean higherCalls = checkDirectionForCalls(currentFloor,Direction.UP);

			//transition RT83-4
			if(!allDoorsClosed() && !lanternLit(Direction.UP))
				state = RT83State.DIRECTION_NONE;
			//transition RT83-6
			else if(currentFloor != MessageDictionary.NONE && mDesiredFloor.getFloor() < currentFloor && higherCalls)
				state = RT83State.DIRECTION_WRONG;
		}

		private void directionDown()
		{
			//This state does not produce a warning
			releaseWarning();

			boolean lowerCalls = checkDirectionForCalls(currentFloor,Direction.DOWN);
			
			//transition RT83-3
			if(!allDoorsClosed() && !lanternLit(Direction.DOWN))
				state = RT83State.DIRECTION_NONE;
			//transition RT83-5
			else if(currentFloor != MessageDictionary.NONE && mDesiredFloor.getFloor() > currentFloor && lowerCalls)
				state = RT83State.DIRECTION_WRONG;
		}

		private void directionWrong()
		{
			//This state produces a warning
			setWarning();

			//transition RT83-7
			if(!allDoorsClosed())
				state = RT83State.DIRECTION_NONE;
		}

	}


	//Java doesn't like enums in private classes.
	private static enum RT9State
	{
		NORMAL_OPERATION,
		TRANS_SLOW_TO_FAST,
		INEFFICIENCY
	}

	/**
	 *	State machine for R-T9
	 */
	private class RT9StateMachine extends StateMachine
	{

		public static final int DRIVE_CONTROL_TIME = 10;
		public static final double SLOW_SPEED = 0.25;
		
		private RT9State state = RT9State.NORMAL_OPERATION;
		private IntervalTimer timer = new IntervalTimer();

		@Override
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT9: Speed not commanded to maximum degree practicable.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT9: Speed not commanded to maximum degree practicable.";
		}

		@Override
		/**
		 *	Updates the state machine
		 */
		public void update()
		{
			switch(state) {
				case NORMAL_OPERATION:		normalOperation();		break;
				case TRANS_SLOW_TO_FAST:	transSlowToFast();		break;
				case INEFFICIENCY:			inefficiency();			break;
				default:
					throw new RuntimeException("State " + state + " was not recognized.");
			}
		}

		private void normalOperation()
		{
			//There is no warning in this state
			releaseWarning();

			//The timer should not be going
			timer.stop();

			//RT9-1
			if(driveActualSpeed.speed() == SLOW_SPEED) {
				state = RT9State.TRANS_SLOW_TO_FAST;
			}

		}

		private void transSlowToFast()
		{
			//There is no warning in this state
			releaseWarning();

			//The timer should be set when entering this state
			timer.start(new SimTime(3*DRIVE_CONTROL_TIME, SimTimeUnit.MILLISECOND));
				
			//RT9-2
			if(!timer.isExpired() && driveCommandedSpeed.speed() != Speed.SLOW )
				state = RT9State.NORMAL_OPERATION;
			//RT9-3
			else if(timer.isExpired())
				state = RT9State.INEFFICIENCY;

		}

		private void inefficiency()
		{
			//There is a warning in this state
			setWarning();

			//The timer should not be going
			timer.stop();

			//transition RT9-4
			if(driveActualSpeed.speed() < SLOW_SPEED)
				state = RT9State.NORMAL_OPERATION;
		}

	} 

	//Java doesn't like enums in private classes.
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
		/**
		 *	Warning message generated whenever this state machine produces a warning
		 *		RT10: A nudge was commanded without a door reversal happening first.
		 *	@return Warning Message
		 */
		protected String warningMessage()
		{
			return "RT10: A nudge was commanded without a door reversal happening first.";
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
			//This state does not set a warning
			releaseWarning();

			//RT10-1
			if(driveActualSpeed.speed() <= LEVEL_SPEED) {
				state = RT10State.STOPPED;
			}

		}

		private void stopped()
		{
			//This state does not set a warning
			releaseWarning();

			//RT10-2
			if(driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT10State.MOVING;
			}
			//RT10-3
			else if(driveActualSpeed.speed() <= LEVEL_SPEED && anyDoorReversing(hall)) {
				state = RT10State.REVERSED;
			}
			//RT10-5
			else if(driveActualSpeed.speed() <= LEVEL_SPEED && anyDoorNudging(hall)) {
				state = RT10State.BAD_NUDGE;
			}

		}

		private void reversed()
		{
			//This state does not set a warning
			releaseWarning();

			//RT10-4
			if(allDoorsClosed() && driveActualSpeed.speed() > LEVEL_SPEED) {
				state = RT10State.MOVING;
			}

		}

		private void badNudge()
		{
			//This state sets a warning
			setWarning();

			//transition RT6-6
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