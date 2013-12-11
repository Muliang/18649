/*
 * 18649 - Fall 2013
 * Group 9 - Wenhui Hu(wenhuih), Yichao Xue(yichoax), Yujia Wang(yujiaw)
 */

package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.elevatorcontrol.MessageDictionary;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * 
 * @author Yujia Wang
 */
public class DoorMotorCommandCanPayloadTranslator extends CanPayloadTranslator {

    /**
     * CAN translator for messages for door motor commands
     * @param payload CAN payload object whose message is interpreted by this translator
     */
    public DoorMotorCommandCanPayloadTranslator(WriteableCanMailbox payload, Hallway h, Side s) {
        super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(h, s));
    }

    /**
     * CAN translator for messages for door motor commands
     * @param payload CAN payload object whose message is interpreted by this translator
     */
    public DoorMotorCommandCanPayloadTranslator(ReadableCanMailbox payload, Hallway h, Side s) {
    	super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + 
        		ReplicationComputer.computeReplicationId(h, s));
    }
    
    
    public void set(DoorCommand cmd) {
        setCommand(cmd);
    }

    public void setValue(DoorCommand cmd) {
        set(cmd);
    }

    /**
     * Set the command for mDoorMotor in bits 0-1 of the can payload
     * @param dir
     */
    public void setCommand(DoorCommand cmd) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, cmd.ordinal(), 0, 2);
        setMessagePayload(b, getByteSize());
    }

    /**
     * 
     * @return the command value from the can payload
     */
    public DoorCommand getCommand() {
        int val = getUnsignedIntFromBitset(getMessagePayload(), 0, 2);
        for (DoorCommand c : DoorCommand.values())
            if (c.ordinal() == val)
                return c;
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    /**
     * Implement a printing method for the translator.
     * @return
     */
    @Override
    public String payloadToString() {
        return "DoorMotor = " + getCommand();
    }
}