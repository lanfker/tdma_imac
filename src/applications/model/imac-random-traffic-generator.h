/*
  This file is created for random traffic generation for the iMac simulation.
  In order to generate traffic, the first thing we should do is to find which
  node should be the destination of the generated packet. Therefore, we should 
  first let each node know their neighbors. 
  One standard of choosing destination is SNR ratio. According to Dr. Zhang, we
  should first try the situation where the SNR ratio is about 8 Dbm.

  The neighbor discovery procedure is like this: let each node in the network send
  a boradcast message to inform its neighbors its existance, then these neighbors 
  add or update the signal map record. In this way, every node knows its neighbors.

  After the neigbhor discovery procedure, we ran this random traffic generator on each
  node. The generator randomly generate packet based on a probability.
*/

#ifndef IMAC_RANDOM_TRAFFIC_GENERATOR_H
#define IMAC_RANDOM_TRAFFIC_GENERATOR_H

#include "ns3/application.h"
#include "ns3/traced-callback.h"
#include "ns3/ptr.h"
#include "ns3/address.h"
#include "ns3/event-id.h"
#include <string>
#include "ns3/random-variable.h"
#include "ns3/socket.h"
#include "ns3/mac48-address.h"
#include "ns3/event-id.h"
#include "ns3/simulator.h"

namespace ns3{

const double DEFAULT_GENERATION_PROBABILITY = 0.6;
const uint32_t DEFAULT_PACKET_SIZE = 100;
const uint32_t LEARNING_PROCESS_PACKET_LENGTH = 8;
const double NEIGHBOR_SELECTION_SNR_THRESHOLD = 8.9370;
const uint32_t MAX_BEACON_TRIALS = 18;
const uint32_t MAX_SETTING_DELAY = 2000;
const uint32_t DEFAULT_WIFI_DEVICE_INDEX = 0;

class ImacRandomTrafficGenerator: public Application
  {
  public:
    static TypeId GetTypeId(void);
    ImacRandomTrafficGenerator ();// by defalt, the threshold value is 0.6
    ImacRandomTrafficGenerator (double probabilityThreshold);
    virtual ~ImacRandomTrafficGenerator();
    //void SetDelay ( RandomVariable delay);
    //void SetSize (RandomVariable size);
    //void SetRemote (std::string socketType, Address remote);
    /*
     * In this method, we choose a neighbor of the node according to, for example, SNR ratio
     * as the destination of the generated packet. To achieve this, we use signal map as a basis of 
     * selecting. Therefore, the address should be MAC address since we only maintain MAC address
     * in signal map.
     * */
    Mac48Address GetDesiredAddress();
    void GenerateBeaconMessage ();
    void ClassifyLinks (std::vector<TdmaLink> &vec);
  private:
    virtual void StartApplication (void);
    virtual void StopApplication (void);
    void DoGenerate ();
    uint32_t m_size;
    uint32_t m_unicastCount;
    double m_probabilityThreshold;
    RandomVariable m_delay;
    RandomVariable m_settingDelay;
    EventId m_nextEvent;
    Ptr<Socket> m_socket;
    uint32_t m_beaconCount;
    uint32_t m_outboundReqested;
    RandomVariable m_timeIntervalForBeacon;
    Time m_stopTime;
  };

}


#endif // IMAC_RANDOM_TRAFFIC_GENERATOR_H
