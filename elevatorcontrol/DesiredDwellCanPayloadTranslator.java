package simulator.elevatorcontrol;

import simulator.framework.Hallway;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public  class DesiredDwellCanPayloadTranslator extends IntegerCanPayloadTranslator{

	public DesiredDwellCanPayloadTranslator(ReadableCanMailbox payload, Hallway b) {
		super(payload);
	}
	
	public DesiredDwellCanPayloadTranslator(WriteableCanMailbox payload, Hallway b) {
		super(payload);
	}
	
}
