/*
 * 18649 Fall 2013
 * group 9
 * Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 */

package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class ByteBooleanCanPayloadTranslator extends simulator.payloads.translators.CanPayloadTranslator{
	
	private String name;
	
	public ByteBooleanCanPayloadTranslator(WriteableCanMailbox payload, int expectedId, String name) {
		super(payload, 1, expectedId);
		this.name = name;
	}
	
	public ByteBooleanCanPayloadTranslator(ReadableCanMailbox payload, int expectedId, String name) {
		super(payload, 1, expectedId);
		this.name = name;
	}

	@Override
	public String payloadToString() {
        return name + Boolean.toString(getValue());
	}
	
    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }

    
    public void setValue(boolean value) {       
        BitSet b = new BitSet(8);
        b.set(7, value);
        setMessagePayload(b, getByteSize());
    }
    
    public boolean getValue() {
        return getMessagePayload().get(7);
    }
    
    
}
