#ifndef SETTINGS_H
#define SETTINGS_H

/*
#ifndef MIXED_PDR_REQUIREMENTS
#define MIXED_PDR_REQUIREMENTS
#endif
*/

//Whether use PDR protection
//#define NO_PROTECTION

#define RID_INITIAL_ER
//#define ENABLE_RID
#define MIN_VARIANCE_CONTROLLER

//#define P_CONTROLLER_DESIRED_PDR

//#define P_CONTROLLER_REFERENCE_I

//#define CONSERVATIVE_ER_CHANGE

//------------------PDR-------------------
#ifndef MIXED_PDR_REQUIREMENTS
const double DESIRED_DATA_PDR = 0.9;
#endif
const double DESIRED_ACK_PDR = 0.9;
//Tx Probability

#ifndef TX_PROBABILITY_1
#define TX_PROBABILITY_1 
#endif
/*
#ifndef CONFLICTING_SET_PROBABILITY
#define CONFLICTING_SET_PROBABILITY
#endif
#ifndef TX_DURATION_PROBABILITY
#define TX_DURATION_PROBABILITY
#endif
#ifndef ESTIMATED_MAX
#define ESTIMATED_MAX
#endif
*/
#ifndef ESTIMATED_3_STD
#define ESTIMATED_3_STD
#endif

const int64_t PRIORITY_RESET_TIMESLOT = 100;
const uint64_t PACKET_GENERATION_INTERVAL = 8; // milliseconds

//-------------------INTERFERENCE.H-----------------------
#ifndef NI_QUANTILE_ESTIMATION_EWMA
#define NI_QUANTILE_ESTIMATION_EWMA
#endif

const double NOISE_POWER_W = 4.01237e-13;
const double DEFAULT_INTERFERENCE_W = 2.00e-16; //
const uint32_t NORMAL_SAMPLE_INTERVAL = 150; //microsecods
const uint32_t DATA_INTERFERENCE_SAMPLE_INTERVAL = 20;
const uint32_t ACK_INTERFERENCE_SAMPLE_INTERVAL = 10;
const uint32_t CONTROL_INTERFERENCE_SAMPLE_INTERVAL = 2000;
const uint32_t MAX_INTERFERENCE_SAMPLE_SIZE = 3000;
const double ABSOLUTE_NI_THRESHOLD = 1.0e-09;
const double EWMA_COEFFICIENT = 0.8;
const double NI_SAMPLE_QUANTILE = 0.90;
const double NI_SAMPLE_FILTER = 4; //times of standard deviation


/*
#ifndef NI_QUANTILE_ESTIMATION_SLIDING_WINDOW
#define NI_QUANTILE_ESTIMATION_SLIDING_WINDOW
#endif

#ifndef NI_QUANTILE_ESTIMATION_SLIDING_EWMA
#define NI_QUANTILE_ESTIMATION_SLIDING_EWMA
#endif
*/

//--------------------------WIFI-IMAC-PHY.H-------------------

const uint32_t CHANNEL_SWITCH_DELAY = 750; // Microseconds;
const int32_t NORMAL_TX_POWER = -25; // in dBm
const double ENERGY_DETECTION_THRESHOLD = -96;// in dBm
const double CCA_THRESHOLD = -87.2; // in dBm
const double TX_GAIN = 1;
const double RX_GAIN = 1;
const uint32_t DATA_CHANNEL = 1;
const uint32_t CONTROL_CHANNEL = 2;
const uint8_t MAX_TX_POWER_LEVEL = 255;
const double DEFAULT_INITIAL_EDGE = 1.3213e-12;
const uint32_t DATA_TX_POWER_LEVEL = 0;


//---------------------MAC-LOW.H-----------------------------------
#ifndef POWER_CONTROL
#define POWER_CONTROL
#endif 

#ifndef SMALL_NETWORK
#define SMALL_NETWORK
#endif
/*
#ifndef MAX_POWER_LEVEL
#define MAX_POWER_LEVEL
#endif
*/
#ifndef ENABLE_D0
#define ENABLE_D0
#endif


// ------------------------const define section ------------------------
const uint32_t MAX_INFO_ITEM_SIZE = 18;
const uint32_t ER_INFO_ITEM_SIZE = 4;
const uint32_t MAX_INFO_ITEM_SIZE_IN_DATA_PACKET = 15;
const uint32_t CONTROL_PAYLOAD_LENGTH = MAX_INFO_ITEM_SIZE * ER_INFO_ITEM_SIZE + 28;
const uint32_t DATA_PACKET_PAYLOAD_LENGTH = MAX_INFO_ITEM_SIZE_IN_DATA_PACKET * ER_INFO_ITEM_SIZE + 28;
const uint32_t ER_INFO_ITEM_CATEGORY_ONE = 1;
const uint32_t ER_INFO_ITEM_CATEGORY_TWO = 2;
#ifdef SMALL_NETWORK
const uint16_t INVALID_SENDER = 130;
const uint16_t NETWORK_SIZE = 125;
#endif

#ifdef LARGE_NETWORK
const uint16_t INVALID_SENDER = 280;
const uint16_t NETWORK_SIZE = 270;
#endif
const int AMPLIFY_TIMES = 30;
//----------------------------------------

//--------------D0 QUANTILE---------------
const double D_0_QUANTILE = 0.90;
//----------------------------------------


const uint32_t ESTIMATION_WINDOW = 20;
const uint32_t TIME_SLOT_LENGTH = 8; // in terms of ms
const uint16_t DEFAULT_INFO_ITEM_PRIORITY = 7;
const uint16_t EXTRA_HIGHEST_PRIORITY_SENDING_TRIALS = 1;
const uint16_t MAX_CONTROL_PACKET_PRIORITY = DEFAULT_INFO_ITEM_PRIORITY; // start from 0, end with 7. 8 in total
const double SNR_WITH_100_PERCENT_PDR = 9.7;
const double DELTA_SNR_MARGIN = 2;// dB
const uint32_t MAX_D0_SAMPLE_SIZE = 200;
const uint32_t MAX_TIME_SLOT_IN_PAYLOAD = 4096;
const uint32_t MAX_SEQ_NO = 4096;
const uint32_t IMPOSSIBLE_D0_VALUE = 3000;
const double INITIAL_BI_DIRECTIOANL_ER_EDGE = 100;
const double IMPOSSIBLE_ER = 100;
const uint32_t MAX_ER_INFORM_TIMES = 2;
const uint32_t IMPOSSIBLE_VER_NO_DIFFERENCE = 100;
const uint8_t MAX_VERSION_NUMBER = 128;
const uint32_t DEFAULT_D0_VALUE = 0;
const uint32_t TIME_TO_TX_IN_SLOT = 751;// in micro seconds
const uint32_t DELAY_BEFORE_SWITCH_CHANNEL = 6700; // within this time, a data transmission should be finished
const uint32_t MAX_PROPAGATION_DELAY = 190; // nanoSeconds
const uint32_t MAX_BACKOFF_TIME = 3000000;//nano seconds
const uint32_t ONE_MILLISECOND = 1000000;// nano seconds

const uint32_t ITEM_LIFE_TIMESLOT = 8000;
const double DEFAULT_TX_PROBABILITY = 1;
const uint32_t INFO_ITEM_SUM = 3000;
const uint32_t QUEUE_SIZE = 1000;
const double DEFAULT_PACKET_GENERATION_PROBABILITY = 1;
const int64_t UNDEFINED_NEXT_TX_SLOT = -1;


//-----------------------CONTROLLER --------------------------------
const double DELTA_Y = 0.00;
const double E_0 = 0.04;

/*
#ifndef EWMA_PDR_RISING
#define EWMA_PDR_RISING
#endif
*/


// ----------------------------end definition---------------------------
//
#endif
