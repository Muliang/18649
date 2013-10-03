package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class HallCallCanPayloadTranslator extends BooleanCanTranslator{

	public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
		super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
		// TODO Auto-generated constructor stub
	}

	public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
		super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
		// TODO Auto-generated constructor stub
	}
}
