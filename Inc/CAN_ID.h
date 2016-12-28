/*
 * CAN_ID.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Frank Gu
 */

#ifndef CAN_ID_H_
#define CAN_ID_H_

/* CAN NodeID and associated ID organizations
 * P2P ID: nodeID + p2pOffest
 * Status Word ID: nodeID + swOffset
 * Firmware Version ID: nodeID + fwOffset
 */
#define CAN_HB_DLC 	 4							// Heartbeat frame data length
#define CAN_FW_DLC	 4							// Firmware version data length

// Offsets BEGIN
#define p2pOffset	 0x040
#define swOffset 	 0x050
#define fwOffset	 0x180
// Offsets END

// NodeIDs BEGIN
#define radio_nodeID 6							// Radio module nodeID

// NodeIDs END

// P2P IDs BEGIN
#define radio_P2P	radio_nodeID + p2pOffset
// P2P IDs END

// Status Word IDs BEGIN
#define radio_SW	radio_nodeID + swOffset
// Status Word IDs END


// Firmware Revision IDs BEGIN
#define radio_FW	radio_nodeID + fwOffset		// Radio module firmware

// Firmware Revision IDs END

// Enumerations
// Connection state
enum connState {
	DISCONNECTED,
	CONNECTING,
	CONNECTED,
	UNRELIABLE,
	CONN_ERROR
};

// Node state
enum nodeState{
	INIT,
	ACTIVE,
	SHUTDOWN,
	HARD_ERROR
};

enum nodeCommands{
	NODE_HRESET,
	NODE_RESET,
	NODE_SHUTDOWN,
	NODE_START,
	CC_ACK,
	CC_NACK
};

#endif /* CAN_ID_H_ */
