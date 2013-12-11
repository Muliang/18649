/*
 * 18649 Fall 2013
 * group 9
 * Wenhui Hu (wenhuih), Yichao Xue(yichaox), Yujia Wang(yujiaw)
 * 
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;

/**
 * This class defines constants for CAN IDs that are used throughout the simulator.
 *
 * The default values will work for early projects.  Later on, you will modify these
 * values when you create a network schedule.
 *
 * @author justinr2
 */
public class MessageDictionary {

    //controller periods
    public final static int NONE = -1;
    public final static SimTime HALL_BUTTON_CONTROL_PERIOD = new SimTime(100, SimTimeUnit.MILLISECOND);
    public final static SimTime CAR_BUTTON_CONTROL_PERIOD = new SimTime(100, SimTimeUnit.MILLISECOND);
    public final static SimTime LANTERN_CONTROL_PERIOD = new SimTime(200, SimTimeUnit.MILLISECOND);
    public final static SimTime CAR_POSITION_CONTROL_PERIOD = new SimTime(50, SimTimeUnit.MILLISECOND);
    public final static SimTime DISPATCHER_PERIOD = new SimTime(50, SimTimeUnit.MILLISECOND);
    public final static SimTime DOOR_CONTROL_PERIOD = new SimTime(10, SimTimeUnit.MILLISECOND);
    public final static SimTime DRIVE_CONTROL_PERIOD = new SimTime(10, SimTimeUnit.MILLISECOND);
    
  //controller message IDs
    public final static int DRIVE_SPEED_CAN_ID =                0x0ABCB500;
    public final static int DRIVE_COMMAND_CAN_ID =              0x0F0AB500;
    public final static int DESIRED_DWELL_BASE_CAN_ID =         0x0CB0B600;
    public final static int DESIRED_FLOOR_CAN_ID =              0x0F0CB600;
    public final static int CAR_POSITION_CAN_ID =               0x0F0DB700;
    public final static int DOOR_MOTOR_COMMAND_BASE_CAN_ID =    0x0B20B800;
    public final static int HALL_CALL_BASE_CAN_ID =             0x0D78B900;
    public final static int HALL_LIGHT_BASE_CAN_ID =            0x0F10B900;
    public final static int CAR_CALL_BASE_CAN_ID =              0x0DDCBA00;
    public final static int CAR_LIGHT_BASE_CAN_ID =             0x0F12BA00;
    public final static int CAR_LANTERN_BASE_CAN_ID =           0x0F13BB00;
    
    //module message IDs
    public final static int AT_FLOOR_BASE_CAN_ID =              0x0BE82800;
    public final static int CAR_LEVEL_POSITION_CAN_ID =         0x0C4C3C00;
    public final static int CAR_WEIGHT_CAN_ID =                 0x0EA47800;
    public final static int CAR_WEIGHT_ALARM_CAN_ID =           0x0F088C00;
    public final static int DOOR_OPEN_SENSOR_BASE_CAN_ID =      0x0F6CA000;
    public final static int DOOR_CLOSED_SENSOR_BASE_CAN_ID =    0x0D145000;
    public final static int DOOR_REVERSAL_SENSOR_BASE_CAN_ID =  0x0A586400;
    public final static int HOISTWAY_LIMIT_BASE_CAN_ID =        0x0FD0B400;
    public final static int EMERGENCY_BRAKE_CAN_ID =            0x0B841400;
    public final static int LEVELING_BASE_CAN_ID =              0x09F41400;
}

