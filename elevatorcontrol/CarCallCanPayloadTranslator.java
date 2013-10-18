/*
 * 18649 Fall 2013
 * group 9
 * Priya Mahajan (priyam), Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * 
 */

package simulator.elevatorcontrol;

//import simulator.elevatormodules.BooleanCanTranslator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
//import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class CarCallCanPayloadTranslator extends ByteBooleanCanPayloadTranslator {

	
    /**
     * CAN translator for messages from CarCall sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
	public CarCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
		super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
	}

    /**
     * CAN translator for messages from CarCall sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
	public CarCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
		super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
	}

}
