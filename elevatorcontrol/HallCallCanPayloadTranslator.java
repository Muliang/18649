/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * Author: Wenhui Hu (wenhuih)
 */

package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class HallCallCanPayloadTranslator extends ByteBooleanCanPayloadTranslator{

	public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
//		super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
		// TODO Auto-generated constructor stub
		super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));
	}

	public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
		// TODO Auto-generated constructor stub
		super(payload, MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction), "HallCall" + ReplicationComputer.makeReplicationString(floor, hallway, direction));

	}
}
