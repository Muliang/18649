/*
 * 18649 - Fall 2013
 * Group 9 - 	Priya Mahajan 	(priyam)
 * 				Wenhui Hu		(wenhuih)
 * 				Yichao Xue		(yichaox)
 * 				Yujia Wang		(yujiaw) 
 */

package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.elevatorcontrol.MessageDictionary;
import simulator.framework.Direction;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * @author Yujia Wang
 */
public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {

    /**
     * CAN translator for messages from drive speed sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     */
    public DriveSpeedCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 3, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }

    /**
     * CAN translator for messages from drive speed sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     */
    public DriveSpeedCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 3, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }
    
    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param speed
     * @param dir
     */
    public void set(double speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    /**
     * Similar to the other set method, but the Speed/Dir field order reversed.
     * @param dir
     * @param speed
     */
    public void set(Direction dir, double speed) {
        setSpeed(speed);
        setDirection(dir);
    }


    /**
     * Set the speed for mDriveSpeed into the loweest 32 bits of the payload
     * @param speed
     */
    public void setSpeed(double speed) {
    	int iSpeed = (short)(speed * 100 + 0.5);
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, iSpeed, 0, 16);
        setMessagePayload(b, getByteSize());
    }

    /**
     *
     * @return the speed value from the can message payload
     */
    public int getSpeed() {
        return getUnsignedIntFromBitset(getMessagePayload(), 0, 16);
    }

    /**
     * Set the direction for mDesiredSpeed in bits 32-47 of the can payload
     * @param dir
     */
    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, dir.ordinal(), 16, 2);
        setMessagePayload(b, getByteSize());
    }

    /**
     * 
     * @return the direction value from the can payload
     */
    public Direction getDirection() {
        int val = getUnsignedIntFromBitset(getMessagePayload(), 16, 2);
        for (Direction d : Direction.values())
            if (d.ordinal() == val)
                return d;
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }
    
    /**
     * 
     * @return if the car is stopped
     */
    public boolean isStop() {
    	return (getSpeed() == 0 || getDirection() == Direction.STOP);
    }

    /**
     * Implement a printing method for the translator.
     * @return
     */
    @Override
    public String payloadToString() {
        return "DesiredSpeed = " + getSpeed() + ", DesiredDirection = " + getDirection();
    }
}