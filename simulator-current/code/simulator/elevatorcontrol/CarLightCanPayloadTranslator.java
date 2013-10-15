package simulator.elevatorcontrol;

import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;


public class CarLightCanPayloadTranslator extends ByteBooleanCanPayloadTranslator {

	
    /**
     * CAN translator for messages from CarLight sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
	public CarLightCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
		super(payload, MessageDictionary.CAR_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarLight" + ReplicationComputer.computeReplicationId(floor, hallway));
	}

    /**
     * CAN translator for messages from CarLight sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
	public CarLightCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
		super(payload, MessageDictionary.CAR_LIGHT_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarLight" + ReplicationComputer.computeReplicationId(floor, hallway));
	}
	

}
