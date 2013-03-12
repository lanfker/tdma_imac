#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/object-factory.h"
#include "wifi-imac-channel.h"
#include "wifi-imac-phy.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"
#include "wifi-imac-header.h"
#include "wifi-net-device.h"


NS_LOG_COMPONENT_DEFINE ("WifiImacChannel");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (WifiImacChannel);

TypeId
WifiImacChannel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::WifiImacChannel")
    .SetParent<WifiChannel> ()
    .AddConstructor<WifiImacChannel> ()
    .AddAttribute ("PropagationLossModel", "A pointer to the propagation loss model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&WifiImacChannel::m_loss),
                   MakePointerChecker<PropagationLossModel> ())
    .AddAttribute ("PropagationDelayModel", "A pointer to the propagation delay model attached to this channel.",
                   PointerValue (),
                   MakePointerAccessor (&WifiImacChannel::m_delay),
                   MakePointerChecker<PropagationDelayModel> ())
  ;
  return tid;
}

WifiImacChannel::WifiImacChannel ()
{
}
WifiImacChannel::~WifiImacChannel ()
{
  NS_LOG_FUNCTION_NOARGS ();
  m_phyList.clear ();
}

void
WifiImacChannel::SetPropagationLossModel (Ptr<PropagationLossModel> loss)
{
  m_loss = loss;
}
void
WifiImacChannel::SetPropagationDelayModel (Ptr<PropagationDelayModel> delay)
{
  m_delay = delay;
}

void
WifiImacChannel::Send (Ptr<WifiImacPhy> sender, Ptr<const Packet> packet, double txPowerDbm,
                       WifiMode wifiMode, WifiPreamble preamble) const
{
  /*
  if (Simulator::Now () < Simulator::LearningTimeDuration )
  {
    std::cout<<Simulator::Now ()<<" txPowerDbm: "<<txPowerDbm << std::endl;
  }
  */
  Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();



  NS_ASSERT (senderMobility != 0);
  uint32_t j = 0;
  for (PhyList::const_iterator i = m_phyList.begin (); i != m_phyList.end (); i++, j++)
    {
      if (sender != (*i))
        {
          // For now don't account for inter channel interference
          if ((*i)->GetChannelNumber () != sender->GetChannelNumber ())
            {
              continue;
            }

          Ptr<MobilityModel> receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
          Time delay = m_delay->GetDelay (senderMobility, receiverMobility);
          m_loss->SetNext (0);
          double rxPowerDbm = m_loss->CalcRxPower (txPowerDbm, senderMobility, receiverMobility);
          if ( rxPowerDbm == txPowerDbm )
          {
            continue;
          }
          NS_LOG_DEBUG ("propagation: txPower=" << txPowerDbm << "dbm, rxPower=" << rxPowerDbm << "dbm, " <<
                        "distance=" << senderMobility->GetDistanceFrom (receiverMobility) << "m, delay=" << delay);

          Ptr<Packet> copy = packet->Copy ();

          Ptr<Object> dstNetDevice = m_phyList[j]->GetDevice ();
          uint32_t dstNode;
          if (dstNetDevice == 0)
            {
              dstNode = 0xffffffff;
            }
          else
            {
              dstNode = dstNetDevice->GetObject<NetDevice> ()->GetNode ()->GetId ();
            }
          // since it is in a for loop, all the nodes using the same wifi channel will be scheduled to receive data.
          Simulator::ScheduleWithContext (dstNode,
                                          delay, &WifiImacChannel::Receive, this,
                                          j, copy, rxPowerDbm, wifiMode, preamble);
        }
    }
}

double
WifiImacChannel::GetDistanceBetweenNodes(Mac48Address src, Mac48Address dest) const
{
  if ( src == Mac48Address::GetBroadcast () || dest == Mac48Address::GetBroadcast ())
  {
    return 0;
  }
  Ptr<MobilityModel> senderMobility = NULL;
  Ptr<MobilityModel> receiverMobility = NULL;

  senderMobility = m_phyList[ src.GetNodeId () - 1]->GetMobility ()->GetObject<MobilityModel> ();
  receiverMobility = m_phyList[ dest.GetNodeId () - 1]->GetMobility ()->GetObject<MobilityModel> ();
  if( senderMobility != NULL && receiverMobility != NULL )
  {
    double distance = senderMobility->GetDistanceFrom (receiverMobility);
    return distance;
  }
  return 0;
}
Time 
WifiImacChannel::GetPropagationDelayToNode(Mac48Address src, Mac48Address dest) const
{
  //Ptr<MobilityModel> senderMobility = sender->GetMobility ()->GetObject<MobilityModel> ();
  
  //NS_ASSERT (senderMobility != 0);
  Ptr<MobilityModel> senderMobility = NULL;
  Ptr<MobilityModel> receiverMobility = NULL;
  uint32_t j = 0;
  for (PhyList::const_iterator i = m_phyList.begin(); i != m_phyList.end(); i++, j++)
  {
      //std::cout<<"mac address list: "<<m_phyList[j]->GetDevice ()->GetObject<WifiNetDevice> ()->GetAddress ()<<std::endl;
      if (m_phyList[j]->GetDevice ()->GetObject<WifiNetDevice> ()->GetAddress () == dest)
      {
        receiverMobility = (*i)->GetMobility ()->GetObject<MobilityModel> ();
      }
      else if (m_phyList[j]->GetDevice ()->GetObject<WifiNetDevice> ()->GetAddress () == src)
      {
        senderMobility = (*i)->GetMobility ()->GetObject<MobilityModel> (); 
      }
  }
  if( senderMobility != NULL && receiverMobility != NULL )
  {
    Time delay = m_delay->GetDelay (senderMobility, receiverMobility);
    //std::cout<<"src=: "<<src<<" dest=: "<<dest<<"  equal? "<<(senderMobility == receiverMobility)<<" they are not NULL, with delay: "<<delay<<std::endl;
    return delay;
  }
  //std::cout<<"************************************************did not find "<<"src=: "<<src<<" dest=: "<<dest<<std::endl;
  return Seconds (0);
}

void
WifiImacChannel::Receive (uint32_t i, Ptr<Packet> packet, double rxPowerDbm,
                          WifiMode txMode, WifiPreamble preamble) const
{
  // specify the i-th WifiImacPhy to receive data.
  m_phyList[i]->StartReceivePacket (i, packet, rxPowerDbm, txMode, preamble);// m_phyList is of type vector<Ptr<WifiImacPhy> >
}

uint32_t
WifiImacChannel::GetNDevices (void) const
{
  return m_phyList.size ();
}
Ptr<NetDevice>
WifiImacChannel::GetDevice (uint32_t i) const
{
  return m_phyList[i]->GetDevice ()->GetObject<NetDevice> ();
}

void
WifiImacChannel::Add (Ptr<WifiImacPhy> phy)
{
  m_phyList.push_back (phy);
}

double WifiImacChannel::GetRxPowerByDistance (double txPowerDbm, double distance)
{
  return m_loss->GetObject <LogDistancePropagationLossModel> ()->GetReceivePowerByDistance (txPowerDbm, distance );
}


} // namespace ns3
