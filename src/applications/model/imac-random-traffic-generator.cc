#include "imac-random-traffic-generator.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/socket.h"
#include "ns3/random-variable.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/mac48-address.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-imac-phy.h"
#include "ns3/event-id.h"


NS_LOG_COMPONENT_DEFINE ("ImacRandomTrafficGenerator");
namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED(ImacRandomTrafficGenerator);

  TypeId ImacRandomTrafficGenerator::GetTypeId()
  {
    static TypeId tid = TypeId ("ns3::ImacRandomTrafficGenerator")
      .SetParent<Application>()
      .AddConstructor<ImacRandomTrafficGenerator>()
      .AddAttribute("Probability","The probability with which a node generates packet",
          DoubleValue(DEFAULT_GENERATION_PROBABILITY),
          MakeDoubleAccessor (&ImacRandomTrafficGenerator::m_probabilityThreshold),
          MakeDoubleChecker<double>())
      .AddAttribute ("PacketSize", "The packet size of the generated packets",
          UintegerValue(DEFAULT_PACKET_SIZE), MakeUintegerAccessor(&ImacRandomTrafficGenerator::m_size),
          MakeUintegerChecker<uint32_t>())
      .AddAttribute ("ApplicationStopTime", "The time at which the simulation stops", TimeValue (Seconds(0)),
          MakeTimeAccessor (&ImacRandomTrafficGenerator::m_stopTime), MakeTimeChecker ())
    ;
    return tid;
  }
  /*
   * the default constructor
   */
  ImacRandomTrafficGenerator::ImacRandomTrafficGenerator()
  {
    m_outboundReqested = 0;
    m_unicastCount = 0;
    m_beaconCount = 0;
    m_settingDelay = UniformVariable(0,MAX_SETTING_DELAY);
    NS_LOG_FUNCTION (this);
  }
  
  /*
   * the constructor which could initialize the probability threshold value
   */
  ImacRandomTrafficGenerator::ImacRandomTrafficGenerator( double probabilityThreshold)
  {
    m_probabilityThreshold = probabilityThreshold; 
    m_unicastCount = 0;
    m_outboundReqested = 0;
    m_beaconCount = 0;
    m_settingDelay = UniformVariable(0,MAX_SETTING_DELAY); 
    NS_LOG_FUNCTION (this);
  }
 
  ImacRandomTrafficGenerator::~ImacRandomTrafficGenerator()
  {
    NS_LOG_FUNCTION (this);
  }
  

  /*
   * There are three phase in this traffic generator:
   *   1: every node broadcasts a message such that its neighbors would be aware of its existance
   *   2: every node send a packe to its (with the outBoudnAttenuation and outSinr regarding to these neighbors been piggybacked) 
   *      neighbors one by one such that all its neighbors could know their outBoundAttenuation and outSinr to this node. By doing this,
   *      every node could select packet destination according to its outSinr.
   *   3: generate traffic randomly by a destination selection standard, for example, sinr = 8 dB
   *
   */
  void
  ImacRandomTrafficGenerator::DoGenerate()
  {
    // if m_stopTime is not 0, then the StopApplication method has been invoked. That means the application should stop now.
    if ( m_stopTime != Seconds (0))
    {
      return;
    }
    // Get the MAC & PHY layers of the node
    Ptr<AdhocWifiMac> mac = GetNode()->GetDevice(DEFAULT_WIFI_DEVICE_INDEX)->GetObject<WifiNetDevice>()->GetMac()->GetObject<AdhocWifiMac> ();
    Ptr<WifiImacPhy> phy = GetNode ()->GetDevice (DEFAULT_WIFI_DEVICE_INDEX)->GetObject<WifiNetDevice> ()->GetPhy ()->GetObject<WifiImacPhy> ();
    UniformVariable uni;
    //if simulation is at the intial phase, send a beacon message to let neighbors know the node's existance

    if ( Simulator::Now ()< Simulator::LearningTimeDuration)
    {
      if (m_beaconCount < MAX_BEACON_TRIALS)//every node should send out a beacon message.
      {
        Ptr<Packet> p = Create<Packet> (LEARNING_PROCESS_PACKET_LENGTH);
        mac->Enqueue(p, Mac48Address::GetBroadcast ());// no broadcast message has been sent yet, and is in initial phase, send broadcast message
        m_beaconCount ++;
        m_nextEvent = Simulator::Schedule (MilliSeconds ((uint64_t)m_settingDelay.GetValue ()), &ImacRandomTrafficGenerator::DoGenerate, this);
      }
      else if ( Simulator::Now () < Seconds (100) ) 
      {
        Mac48Address receiver = phy->NeighborSelectionBySinr(NEIGHBOR_SELECTION_SNR_THRESHOLD); //
        if ( receiver != Mac48Address::GetBroadcast ())
        {
          Mac48Address sender = mac->GetAddress ();
          TdmaLink link;
          link.senderAddr = sender.ToString ();
          link.receiverAddr = receiver.ToString ();
          link.linkId = sender.GetNodeId () * Simulator::NodesCountUpperBound + receiver.GetNodeId ();
          Simulator::AddTdmaLink (link); // register the link
          //Simulator::PrintLinks ();
          //Simulator::PrintSignalMap (sender.ToString ());
        }
        phy->RegisterSignalMap ();
        m_nextEvent = Simulator::Schedule(MilliSeconds ((uint64_t)m_settingDelay.GetValue ()), &ImacRandomTrafficGenerator::DoGenerate, this);
      }
      else if (Simulator::Now () > Seconds (40) && Simulator::Now () < Simulator::LearningTimeDuration )
      //share initial ER information
      {
        if (Simulator::m_linksClassified == false )
        {
          Simulator::m_linksClassified = true;
          std::vector<TdmaLink> _vec = Simulator::ListAllLinks ();
          ClassifyLinks (_vec);
        }
        Ptr<Packet> p = Create<Packet> (LEARNING_PROCESS_PACKET_LENGTH);
        mac->Enqueue(p, Mac48Address::GetBroadcast ());
        m_nextEvent = Simulator::Schedule(MilliSeconds ((uint64_t)m_settingDelay.GetValue ()), &ImacRandomTrafficGenerator::DoGenerate, this);
      }
    }
  }

  void ImacRandomTrafficGenerator::GenerateBeaconMessage ()
  {
    // if m_stopTime is not 0, then the StopApplication method has been invoked. That means the application should stop now.
    if ( m_stopTime != Seconds (0))
    {
      return;
    }
    Ptr<Packet> p = Create<Packet> (LEARNING_PROCESS_PACKET_LENGTH);
    Ptr<AdhocWifiMac> mac = GetNode()->GetDevice(DEFAULT_WIFI_DEVICE_INDEX)->GetObject<WifiNetDevice>()->GetMac()->GetObject<AdhocWifiMac> ();
    mac->Enqueue (p, Mac48Address::GetBroadcast ());
  }
  void ImacRandomTrafficGenerator::StartApplication(void)
  {
    NS_LOG_FUNCTION_NOARGS ();
    DoGenerate();
    //Simulator::Schedule (Simulator::LearningTimeDuration, &ImacRandomTrafficGenerator::GenerateBeaconMessage, this);
  }

  void ImacRandomTrafficGenerator::StopApplication(void)
  {
    NS_LOG_FUNCTION_NOARGS ();
    m_stopTime = Simulator::Now ();
  }
  void ImacRandomTrafficGenerator::ClassifyLinks (std::vector<TdmaLink> &vec)
  {
    for (int32_t i = 0; i < Simulator::NodesCountUpperBound; ++ i)
    {
      Simulator::m_nodeLinkDetails[i].selfInitiatedLink.senderAddr = "";
      Simulator::m_nodeLinkDetails[i].selfInitiatedLink.receiverAddr = "";
      Simulator::m_nodeLinkDetails[i].selfInitiatedLink.linkId = 0;
    }
    for (std::vector<TdmaLink>::iterator it = vec.begin (); it != vec.end (); ++ it)
    {
      Mac48Address sender = Mac48Address (it->senderAddr.c_str ());
      Mac48Address receiver = Mac48Address (it->receiverAddr.c_str ());
      //   m_nodeLinkDetails
      uint32_t senderIndex = sender.GetNodeId ();
      Simulator::m_nodeLinkDetails [senderIndex].selfInitiatedLink = *it;
      Simulator::m_nodeLinkDetails [senderIndex].relatedLinks.push_back (*it);

      uint32_t receiverIndex = receiver.GetNodeId ();
      Simulator::m_nodeLinkDetails [receiverIndex].relatedLinks.push_back (*it);
    }
  }

}
