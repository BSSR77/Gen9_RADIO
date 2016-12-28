/*
 * CAN_ID.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Frank Gu
 *      Version 1.0
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
#define cc_nodeID	 1							// Command center nodeID
#define mc_nodeID	 2							// Motor controller nodeID
#define bps_nodeID	 3							// Battery protection system nodeID
#define ads_nodeID	 4							// Array diagnostic system nodeID
#define radio_nodeID 5							// Radio module nodeID
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

// Error Messages
#define SysEMSD		0x10		// System emergency shutdown (Hard)
#define UsrEMSD		0x11		// User emergency shutdown (Hard)
#define bpsTrip		0x12		// BPS Trip condition (soft)
#define mcFault		0x13		// Motor controller fault

// Car Control Variables
#define swPos		0x21		// DCI Switch positions
#define brakePos	0x22		// DCI Brake position
#define accelPos	0x23		// DCI Accelerator position
#define regenPos	0x24		// DCI Regeneration position

// Radio Received Commands
#define remoteSD	0x30		// Radio controlled car shutdown (soft)
#define setSpeed	0x31		// Set target cruise speed

// Energy metering variables
#define battQCount	0x200		// Battery net Coulomb count
#define battPwr		0x201		// Battery power
#define motorPwr	0x202		// Motor power
#define lpBusPwr	0x203		// Low power bus power

#define pptAPwr		0x20A		// PPT A Power
#define pptBPwr		0x20B		// PPT B Power
#define pptCPwr		0x20C		// PPT C Power

// Module voltage offset
#define voltOffset	0x350		// Note the index of first module voltage at 0x351

// Module temperature array offset
#define tempOffset  0x500		// Note the index of first temperature is at 0x501

#endif /* CAN_ID_H_ */
