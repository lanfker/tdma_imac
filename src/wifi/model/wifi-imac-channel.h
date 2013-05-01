#ifndef WIFI_IMAC_CHANNEL_H
#define WIFI_IMAC_CHANNEL_H

#include <vector>
#include <stdint.h>
#include "ns3/packet.h"
#include "ns3/vector.h"
#include "wifi-channel.h"
#include "wifi-mode.h"
#include "wifi-preamble.h"

namespace ns3 {

class NetDevice;
class PropagationLossModel;
class PropagationDelayModel;
class WifiImacPhy;

/**
 * \brief A Yans wifi channel
 * \ingroup wifi
 *
 * This wifi channel implements the propagation model described in
 * "Yet Another Network Simulator", (http://cutebugs.net/files/wns2-yans.pdf).
 *
 * This class is expected to be used in tandem with the ns3::YansWifiPhy
 * class and contains a ns3::PropagationLossModel and a ns3::PropagationDelayModel.
 * By default, no propagation models are set so, it is the caller's responsability
 * to set them before using the channel.
 */
class WifiImacChannel : public WifiChannel
{
public:
  static TypeId GetTypeId (void);

  WifiImacChannel ();
  virtual ~WifiImacChannel ();

  // inherited from Channel.
  virtual uint32_t GetNDevices (void) const;
  virtual Ptr<NetDevice> GetDevice (uint32_t i) const;

  void Add (Ptr<WifiImacPhy> phy);

  /**
   * \param loss the new propagation loss model.
   */
  void SetPropagationLossModel (Ptr<PropagationLossModel> loss);
  /**
   * \param delay the new propagation delay model.
   */
  void SetPropagationDelayModel (Ptr<PropagationDelayModel> delay);

  /**
   * \param sender the device from which the packet is originating.
   * \param packet the packet to send
   * \param txPowerDbm the tx power associated to the packet
   * \param wifiMode the tx mode associated to the packet
   * \param preamble the preamble associated to the packet
   *
   * This method should not be invoked by normal users. It is
   * currently invoked only from WifiPhy::Send. YansWifiChannel
   * delivers packets only between PHYs with the same m_channelNumber,
   * e.g. PHYs that are operating on the same channel.
   */
  void Send (Ptr<WifiImacPhy> sender, Ptr<const Packet> packet, double txPowerDbm,
             WifiMode wifiMode, WifiPreamble preamble) const;

  Time GetPropagationDelayToNode(Mac48Address src, Mac48Address dest) const;

  double GetDistanceBetweenNodes(Mac48Address src, Mac48Address dest) const;
  double GetRxPowerByDistance (double txPowerDbm, double distance);
  Vector GetNodePosition (Mac48Address addr);

  // -----------------------------------------------------PRIVATE -------------------------------------

private:
  WifiImacChannel& operator = (const WifiImacChannel &);
  WifiImacChannel (const WifiImacChannel &);

  typedef std::vector<Ptr<WifiImacPhy> > PhyList;
  void Receive (uint32_t i, Ptr<Packet> packet, double rxPowerDbm,
                WifiMode txMode, WifiPreamble preamble) const;


  PhyList m_phyList;
  Ptr<PropagationLossModel> m_loss;
  Ptr<PropagationDelayModel> m_delay;
};

} // namespace ns3


#endif /* WIFI_IMAC_CHANNEL_H */
