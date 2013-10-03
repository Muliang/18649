/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;

import java.util.HashMap;

import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.payloads.CANNetwork;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;
import simulator.payloads.PhysicalNetwork.PhysicalConnection;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot. It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they
 * constitute a communication channel between controllers.
 * 
 * @author justinr2
 */
public class Utility {

	public static class DesiredFloor extends DesiredFloorCanPayloadTranslator {

		// writeable
		public DesiredFloor(CANNetwork.CanConnection conn, SimTime period) {
			super(getWriteMailbox(conn, period));
		}

		private static WriteableCanMailbox getWriteMailbox(
				CANNetwork.CanConnection conn, SimTime period) {
			WriteableCanMailbox canMailbox = CanMailbox
					.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
			conn.sendTimeTriggered(canMailbox, period);
			return canMailbox;
		}

		// readable
		public DesiredFloor(CANNetwork.CanConnection conn) {
			super(getReadMailbox(conn));
		}

		private static CanMailbox.ReadableCanMailbox getReadMailbox(
				CANNetwork.CanConnection conn) {
			CanMailbox.ReadableCanMailbox canMailbox = CanMailbox
					.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
			conn.registerTimeTriggered(canMailbox);
			return canMailbox;
		}

	}

	public static class DoorClosedArray {

		private HashMap<Integer, DoorClosedCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
		public final Hallway hallway;

		public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
			this.hallway = hallway;
			for (Side s : Side.values()) {
				int index = ReplicationComputer
						.computeReplicationId(hallway, s);
				ReadableCanMailbox m = CanMailbox
						.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID
								+ index);
				DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(
						m, hallway, s);
				conn.registerTimeTriggered(m);
				translatorArray.put(index, t);
			}
		}

		public boolean getBothClosed() {
			return translatorArray.get(
					ReplicationComputer
							.computeReplicationId(hallway, Side.LEFT))
					.getValue()
					&& translatorArray.get(
							ReplicationComputer.computeReplicationId(hallway,
									Side.RIGHT)).getValue();
		}

	}

	public static class AtFloorArray {

		public HashMap<Integer, AtFloorCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, AtFloorCanPayloadTranslator>();
		public final int numFloors = Elevator.numFloors;

		public AtFloorArray(CANNetwork.CanConnection conn) {
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							h);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID
									+ index);
					AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(
							m, floor, h);
					conn.registerTimeTriggered(m);
					networkAtFloorsTranslators.put(index, t);
				}
			}
		}

		public boolean isAtFloor(int floor, Hallway hallway) {
			return networkAtFloorsTranslators.get(
					ReplicationComputer.computeReplicationId(floor, hallway))
					.getValue();
		}

		public int getCurrentFloor() {
			int retval = MessageDictionary.NONE;
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							h);
					AtFloorCanPayloadTranslator t = networkAtFloorsTranslators
							.get(index);
					if (t.getValue()) {
						if (retval == MessageDictionary.NONE) {
							// this is the first true atFloor
							retval = floor;
						} else if (retval != floor) {
							// found a second floor that is different from the
							// first one
							throw new RuntimeException(
									"AtFloor is true for more than one floor at "
											+ Harness.getTime());
						}
					}
				}
			}
			return retval;
		}
	}

	public static class HallCallArray {
		private HashMap<Integer, HallCallCanPayloadTranslator> networkHallCallArray = new HashMap<Integer, HallCallCanPayloadTranslator>();


		public HallCallArray(CANNetwork.CanConnection conn) {

			for (int i = 0; i < Elevator.numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					for (Direction d : Direction.replicationValues) {
						int index = ReplicationComputer.computeReplicationId(
								floor, h, d);
						ReadableCanMailbox m = CanMailbox
								.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID
										+ index);
						HallCallCanPayloadTranslator t = new HallCallCanPayloadTranslator(
								m, floor, h, d);
						conn.registerTimeTriggered(m);
						networkHallCallArray.put(index, t);
					}
				}
			}
		}
		
		// any hall call button is pressed at a floor
		public boolean isAnyPressed(int floor, Hallway hallway){
			int upIndex = ReplicationComputer.computeReplicationId(floor, hallway, Direction.UP);
			int downIndex = ReplicationComputer.computeReplicationId(floor, hallway, Direction.DOWN);
			
			return networkHallCallArray.get(upIndex).getValue() || networkHallCallArray.get(downIndex).getValue();
		}
	}

	public static class CarCallArray {
		private HashMap<Integer, CarCallCanPayloadTranslator> translatorArray = new HashMap<Integer, CarCallCanPayloadTranslator>();


		public CarCallArray(CANNetwork.CanConnection conn) {
			for (int i = 0; i < Elevator.numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							h);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID
									+ index);
					CarCallCanPayloadTranslator t = new CarCallCanPayloadTranslator(
							m, floor, h);
					conn.registerTimeTriggered(m);
					translatorArray.put(index, t);
				}
			}
		}


		public boolean isPressed(int floor, Hallway hallway) {
			int index = ReplicationComputer
					.computeReplicationId(floor, hallway);
			return translatorArray.get(index).getValue();
		}

	}

	public static class CarCall extends CarCallCanPayloadTranslator {
		private int floor;
		private Hallway hallway;
		private SimTime period;

		public CarCall(CANNetwork.CanConnection conn, SimTime period,
				int floor, Hallway hallway) {
			super(getMailbox(conn, period, floor, hallway), floor, hallway);

			this.floor = floor;
			this.hallway = hallway;
			this.period = period;
		}

		private static WriteableCanMailbox getMailbox(
				CANNetwork.CanConnection conn, SimTime period, int floor,
				Hallway hallway) {
			int id = ReplicationComputer.computeReplicationId(floor, hallway);
			WriteableCanMailbox canMailbox = CanMailbox
					.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID
							+ id);
			conn.sendTimeTriggered(canMailbox, period);
			return canMailbox;
		}

		public ReadableCarCallPayload Readable(
				PhysicalConnection physicalInterface) {
			ReadableCarCallPayload CarCall = CarCallPayload.getReadablePayload(
					floor, hallway);
			physicalInterface.registerTimeTriggered(CarCall);
			return CarCall;
		}
	}

	public static class CarLight extends BooleanCanPayloadTranslator {

		// global
		private int floor;
		private Hallway hallway;
		private SimTime period;

		public CarLight(CANNetwork.CanConnection conn, SimTime period,
				int floor, Hallway hallway) {
			super(getMailbox(conn, period, floor, hallway));
			this.floor = floor;
			this.hallway = hallway;
			this.period = period;
		}

		private static WriteableCanMailbox getMailbox(
				CANNetwork.CanConnection conn, SimTime period, int floor,
				Hallway hallway) {
			int id = ReplicationComputer.computeReplicationId(floor, hallway);
			WriteableCanMailbox canMailbox = CanMailbox
					.getWriteableCanMailbox(MessageDictionary.CAR_LIGHT_BASE_CAN_ID
							+ id);
			conn.sendTimeTriggered(canMailbox, period);
			return canMailbox;
		}

		public static WriteableCarLightPayload Writeable(
				PhysicalConnection physicalInterface, SimTime period,
				int floor, Hallway hallway) {
			WriteableCarLightPayload CarLight = CarLightPayload
					.getWriteablePayload(floor, hallway);
			physicalInterface.sendTimeTriggered(CarLight, period);
			return CarLight;
		}
	}
}
