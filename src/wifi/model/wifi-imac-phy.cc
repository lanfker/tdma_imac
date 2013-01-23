#include "max-er-edge-interference-tag.h"
#include "receiver-address-tag.h"
#include <iostream>
#include <cmath>
#include <string>
#include <algorithm>
#include "wifi-imac-phy.h"
#include "wifi-imac-channel.h"
#include "wifi-mode.h"
#include "wifi-preamble.h"
#include "wifi-phy-state-helper.h"
#include "error-rate-model.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/random-variable.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/pointer.h"
#include "ns3/net-device.h"
#include "ns3/trace-source-accessor.h"
#include "wifi-net-device.h"
#include <math.h>
#include "ns3/address.h"
#include "wifi-mac-header.h"
#include "ns3/object-vector.h"
#include "wifi-imac-header.h"
#include "wifi-mac.h"
#include "adhoc-wifi-mac.h"
#include "ns3/propagation-loss-model.h"
#include "attenuation-tag.h"
#include "out-bound-snr-tag.h"
#include "wifi-net-device.h"
#include "interference-tag.h"
#include "tx-power-dbm-tag.h"
#include "noise-plus-interference-tag.h"
NS_LOG_COMPONENT_DEFINE ("WifiImacPhy");

namespace ns3 {


  NS_OBJECT_ENSURE_REGISTERED (SignalMap);

  //SignalMap
  SignalMap::SignalMap()
  {

  }

  SignalMap::~SignalMap()
  {

  }

  TypeId 
    SignalMap::GetTypeId (void)
    {
      static TypeId tid = TypeId ("ns3::SignalMap")
        .SetParent<Object>()
        .AddConstructor<SignalMap>()
        ;
      return tid;
    }

  NS_OBJECT_ENSURE_REGISTERED (WifiImacPhy);



  TypeId
    WifiImacPhy::GetTypeId (void)
    {
      static TypeId tid = TypeId ("ns3::WifiImacPhy")
        .SetParent<WifiPhy> ()
        .AddConstructor<WifiImacPhy> ()
        .AddAttribute ("EnergyDetectionThreshold",
            "The energy of a received signal should be higher than "
            "this threshold (dbm) to allow the PHY layer to detect the signal.",
            //DoubleValue (-96.0),
            DoubleValue (ENERGY_DETECTION_THRESHOLD),
            MakeDoubleAccessor (&WifiImacPhy::SetEdThreshold,
              &WifiImacPhy::GetEdThreshold),
            MakeDoubleChecker<double> ())
        .AddAttribute ("CcaMode1Threshold",
            "The energy of a received signal should be higher than "
            "this threshold (dbm) to allow the PHY layer to declare CCA BUSY state",
            //DoubleValue (-99.0),
            DoubleValue (CCA_THRESHOLD),
            MakeDoubleAccessor (&WifiImacPhy::SetCcaMode1Threshold,
              &WifiImacPhy::GetCcaMode1Threshold),
            MakeDoubleChecker<double> ())
        .AddAttribute ("TxGain",
            "Transmission gain (dB).",
            DoubleValue (TX_GAIN),
            MakeDoubleAccessor (&WifiImacPhy::SetTxGain,
              &WifiImacPhy::GetTxGain),
            MakeDoubleChecker<double> ())
        .AddAttribute ("RxGain",
            "Reception gain (dB).",
            DoubleValue (RX_GAIN),
            MakeDoubleAccessor (&WifiImacPhy::SetRxGain,
              &WifiImacPhy::GetRxGain),
            MakeDoubleChecker<double> ())
        .AddAttribute ("TxPowerLevels",
            "Number of transmission power levels available between "
            "TxPowerStart and TxPowerEnd included.",
            UintegerValue (2),
            MakeUintegerAccessor (&WifiImacPhy::m_nTxPower),
            MakeUintegerChecker<uint32_t> ())
        .AddAttribute ("TxPowerEnd",
            "Maximum available transmission level (dbm).",
            DoubleValue (NORMAL_TX_POWER),
            MakeDoubleAccessor (&WifiImacPhy::SetTxPowerEnd,
              &WifiImacPhy::GetTxPowerEnd),
            MakeDoubleChecker<double> ())
        .AddAttribute ("TxPowerStart",
            "Minimum available transmission level (dbm).",
            DoubleValue (NORMAL_TX_POWER),
            MakeDoubleAccessor (&WifiImacPhy::SetTxPowerStart,
              &WifiImacPhy::GetTxPowerStart),
            MakeDoubleChecker<double> ())
        .AddAttribute ("RxNoiseFigure",
            "Loss (dB) in the Signal-to-Noise-Ratio due to non-idealities in the receiver."
            " According to Wikipedia (http://en.wikipedia.org/wiki/Noise_figure), this is "
            "\"the difference in decibels (dB) between"
            " the noise output of the actual receiver to the noise output of an "
            " ideal receiver with the same overall gain and bandwidth when the receivers "
            " are connected to sources at the standard noise temperature T0 (usually 290 K)\"."
            " For",
            DoubleValue (7),
            MakeDoubleAccessor (&WifiImacPhy::SetRxNoiseFigure,
              &WifiImacPhy::GetRxNoiseFigure),
            MakeDoubleChecker<double> ())
        .AddAttribute ("State", "The state of the PHY layer",
            PointerValue (),
            MakePointerAccessor (&WifiImacPhy::m_state),
            MakePointerChecker<WifiPhyStateHelper> ())
        .AddAttribute ("ChannelSwitchDelay",
            "Delay between two short frames transmitted on different frequencies. NOTE: Unused now.",
            //TimeValue (MicroSeconds (250)), // 250 \mu s maybe too long
            TimeValue (MicroSeconds (CHANNEL_SWITCH_DELAY)), // defined in wifi-imac-phy.h
            MakeTimeAccessor (&WifiImacPhy::m_channelSwitchDelay),
            MakeTimeChecker ())
        .AddAttribute ("ChannelNumber",
            "Channel center frequency = Channel starting frequency + 5 MHz * (nch - 1)",
            UintegerValue (DATA_CHANNEL),
            MakeUintegerAccessor (&WifiImacPhy::SetChannelNumber,
              &WifiImacPhy::GetChannelNumber),
            MakeUintegerChecker<uint16_t> ())
        .AddAttribute("SignalMap", "a vector that stores the signal map of node neighbors",
            ObjectVectorValue(), 
            MakeObjectVectorAccessor(&WifiImacPhy::m_signalMap),
            MakeObjectVectorChecker<SignalMap>())
        .AddAttribute ("PathLossExponent", "The path loss exponent of the wifi channel",
            DoubleValue(PATH_LOSS_EXPONENT), // defined in simulator.h
            MakeDoubleAccessor (&WifiImacPhy::m_pathlossExponent), 
            MakeDoubleChecker<double> ())
        .AddTraceSource ("SignalMapSource", "A map record attenuation from and to neighbor nodes",
            MakeTraceSourceAccessor (&WifiImacPhy::m_signalMapCallback)) 
        .AddTraceSource ("IntTrace", "A test for trace integer", MakeTraceSourceAccessor(&WifiImacPhy::m_intTrace))

        ;
      return tid;
    }

  WifiImacPhy::WifiImacPhy ()
    :  m_channelNumber (1),
    m_endRxEvent (),
    m_random (0.0, 1.0),
    m_channelStartingFrequency (0)
  {
    NS_LOG_FUNCTION (this);
    m_state = CreateObject<WifiPhyStateHelper> ();

    // the default value 
    m_erEdgeInterferenceW = DEFAULT_INITIAL_EDGE;  // the interference power at 1.5 transmission range. Computed by Matlab. This initial value seems of no use.
    m_noiseW = NOISE_POWER_W;
    m_initialPowerLevel = MAX_TX_POWER_LEVEL; // the power level of the beacon message in the learning process of the iMAC
  }

  void WifiImacPhy::ScheduleSwitchChannel (Time delay, uint16_t number)
  {
    Simulator::Schedule (delay, &WifiImacPhy::SetChannelNumber, this, number);
  }

  WifiImacPhy::~WifiImacPhy ()
  {
    NS_LOG_FUNCTION (this);
  }

  void
    WifiImacPhy::DoDispose (void)
    {
      NS_LOG_FUNCTION (this);
      m_channel = 0;
      m_deviceRateSet.clear ();
      m_device = 0;
      m_mobility = 0;
      m_state = 0;
    }

  void
    WifiImacPhy::ConfigureStandard (enum WifiPhyStandard standard)
    {
      NS_LOG_FUNCTION (this << standard);
      switch (standard)
      {
        case WIFI_PHY_STANDARD_80211a:
          Configure80211a ();
          break;
        case WIFI_PHY_STANDARD_80211b:
          Configure80211b ();
          break;
        case WIFI_PHY_STANDARD_80211g:
          Configure80211g ();
          break;
        case WIFI_PHY_STANDARD_80211_10MHZ:
          Configure80211_10Mhz ();
          break;
        case WIFI_PHY_STANDARD_80211_5MHZ:
          Configure80211_5Mhz ();
          break;
        case WIFI_PHY_STANDARD_holland:
          ConfigureHolland ();
          break;
        case WIFI_PHY_STANDARD_80211p_CCH:
          Configure80211p_CCH ();
          break;
        case WIFI_PHY_STANDARD_80211p_SCH:
          Configure80211p_SCH ();
          break;
        default:
          NS_ASSERT (false);
          break;
      }
    }

  /* return the signal map that the current node is maintaining
   * Added by Chuan
   */
  std::vector<Ptr<SignalMap> > 
    WifiImacPhy::GetSignalMap(void) const
    {
      return m_signalMap;
    }

  void
    WifiImacPhy::SetRxNoiseFigure (double noiseFigureDb)
    {
      NS_LOG_FUNCTION (this << noiseFigureDb);
      m_interference.SetNoiseFigure (DbToRatio (noiseFigureDb));
    }
  void
    WifiImacPhy::SetTxPowerStart (double start)
    {
      NS_LOG_FUNCTION (this << start);
      m_txPowerBaseDbm = start;
    }
  void
    WifiImacPhy::SetTxPowerEnd (double end)
    {
      NS_LOG_FUNCTION (this << end);
      m_txPowerEndDbm = end;
    }
  void
    WifiImacPhy::SetNTxPower (uint32_t n)
    {
      NS_LOG_FUNCTION (this << n);
      m_nTxPower = n;
    }
  void
    WifiImacPhy::SetTxGain (double gain)
    {
      NS_LOG_FUNCTION (this << gain);
      m_txGainDb = gain;
    }
  void
    WifiImacPhy::SetRxGain (double gain)
    {
      NS_LOG_FUNCTION (this << gain);
      m_rxGainDb = gain;
    }
  void
    WifiImacPhy::SetEdThreshold (double threshold)
    {
      NS_LOG_FUNCTION (this << threshold);
      m_edThresholdW = DbmToW (threshold);
    }
  void
    WifiImacPhy::SetCcaMode1Threshold (double threshold)
    {
      NS_LOG_FUNCTION (this << threshold);
      m_ccaMode1ThresholdW = DbmToW (threshold);
    }
  void
    WifiImacPhy::SetErrorRateModel (Ptr<ErrorRateModel> rate)
    {
      m_interference.SetErrorRateModel (rate);
    }
  void
    WifiImacPhy::SetDevice (Ptr<Object> device)
    {
      m_device = device;
    }
  void
    WifiImacPhy::SetMobility (Ptr<Object> mobility)
    {
      m_mobility = mobility;
    }

  double
    WifiImacPhy::GetRxNoiseFigure (void) const
    {
      return RatioToDb (m_interference.GetNoiseFigure ());
    }
  double
    WifiImacPhy::GetTxPowerStart (void) const
    {
      return m_txPowerBaseDbm;
    }
  double
    WifiImacPhy::GetTxPowerEnd (void) const
    {
      return m_txPowerEndDbm;
    }
  double
    WifiImacPhy::GetTxGain (void) const
    {
      return m_txGainDb;
    }
  double
    WifiImacPhy::GetRxGain (void) const
    {
      return m_rxGainDb;
    }

  double
    WifiImacPhy::GetEdThreshold (void) const
    {
      return WToDbm (m_edThresholdW);
    }

  double
    WifiImacPhy::GetCcaMode1Threshold (void) const
    {
      return WToDbm (m_ccaMode1ThresholdW);
    }

  Ptr<ErrorRateModel>
    WifiImacPhy::GetErrorRateModel (void) const
    {
      return m_interference.GetErrorRateModel ();
    }
  Ptr<Object>
    WifiImacPhy::GetDevice (void) const
    {
      return m_device;
    }
  Ptr<Object>
    WifiImacPhy::GetMobility (void)
    {
      return m_mobility;
    }

  double
    WifiImacPhy::CalculateSnr (WifiMode txMode, double ber) const
    {
      return m_interference.GetErrorRateModel ()->CalculateSnr (txMode, ber);
    }

  Ptr<WifiChannel>
    WifiImacPhy::GetChannel (void) const
    {
      return m_channel;
    }
  void
    WifiImacPhy::SetChannel (Ptr<WifiImacChannel> channel)
    {
      m_channel = channel;
      m_channel->Add (this);
    }

  void
    WifiImacPhy::SetChannelNumber (uint16_t nch)
    {
      if (Simulator::Now () == Seconds (0))
      {
        // this is not channel switch, this is initialization
        NS_LOG_DEBUG ("start at channel " << nch);
        m_channelNumber = nch;
        return;
      }

      NS_ASSERT (!IsStateSwitching ());
      switch (m_state->GetState ())
      {
        case WifiImacPhy::RX:
          NS_LOG_DEBUG ("drop packet because of channel switching while reception");
          m_endRxEvent.Cancel ();
          goto switchChannel;
          break;
        case WifiImacPhy::TX:
          NS_LOG_DEBUG ("channel switching postponed until end of current transmission");
          Simulator::Schedule (GetDelayUntilIdle (), &WifiImacPhy::SetChannelNumber, this, nch);
          break;
        case WifiImacPhy::CCA_BUSY:
        case WifiImacPhy::IDLE:
          goto switchChannel;
          break;
        default:
          NS_ASSERT (false);
          break;
      }

      return;

switchChannel:

      NS_LOG_DEBUG ("switching channel " << m_channelNumber << " -> " << nch);
      m_state->SwitchToChannelSwitching (m_channelSwitchDelay);
      m_interference.EraseEvents ();
      /*
       * Needed here to be able to correctly sensed the medium for the first
       * time after the switching. The actual switching is not performed until
       * after m_channelSwitchDelay. Packets received during the switching
       * state are added to the event list and are employed later to figure
       * out the state of the medium after the switching.
       */
      m_channelNumber = nch;
    }

  uint16_t
    WifiImacPhy::GetChannelNumber () const
    {
      return m_channelNumber;
    }

  double
    WifiImacPhy::GetChannelFrequencyMhz () const
    {
      return m_channelStartingFrequency + 5 * GetChannelNumber ();
    }

  void
    WifiImacPhy::SetReceiveOkCallback (RxOkCallback callback)
    {
      m_state->SetReceiveOkCallback (callback);
    }
  void
    WifiImacPhy::SetReceiveErrorCallback (RxErrorCallback callback)
    {
      m_state->SetReceiveErrorCallback (callback);
    }

  /* param dest The mac48address at the 'from' field in the signal map which correspondent to a neighbor mac address
   * of the current node.
   * returns the outBoundAttenuation regarding this 'dest' neighbor.
   * added by Chuan
   */
  double WifiImacPhy::GetOutBoundAttenuationForDest(Mac48Address dest) const
  {
    std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    for( ; it != m_signalMap.end(); ++ it)
    {
      if ( (*it)->from == dest) 
      {
        return (*it)->outBoundAttenuation;
      }
    }
    //if no record has been found in the signal map, return 0
    return 0;
  }

  /* param dest The mac48address at the 'from' field in the signal map which correspondent to a neighbor mac address
   * of the current node.
   * returns the inBoundAttenuation regarding this 'dest' neighbor.
   * added by Chuan
   */
  double WifiImacPhy::GetAttenuationForDest(Mac48Address dest) const
  {
    std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    for( ; it != m_signalMap.end(); ++ it)
    {//if there is a record about the destination address, return itself's inbound attenuation as the dest's outbound attenuation
      if ( (*it)->from == dest) 
      {
        return (*it)->inBoundAttenuation;
      }
    }
    //if no record has been found in the signal map, return 0
    return 0;
  }

  /* param dest The mac48address at the 'from' field in the signal map which correspondent to a neighbor mac address
   * of the current node.
   * returns the in-bound SNR (not sinr_ regarding this 'dest' neighbor.
   * added by Chuan
   */
  double WifiImacPhy::GetSinrForDest(Mac48Address dest) const
  {

    std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    for( ; it != m_signalMap.end(); ++ it)
    {//if there is a record about the destination address, return itself's inbound attenuation as the dest's outbound attenuation
      if ( (*it)->from == dest)
      {
        return (*it)->inSinr;
      }
    }
    //if no record has been found in the signal map, return 0
    return 0;
  }

  /* param dest The mac48address at the 'from' field in the signal map which correspondent to a neighbor mac address
   * of the current node.
   * returns the out-bound SNR (not sinr_ regarding this 'dest' neighbor.
   * added by Chuan
   */
  double WifiImacPhy::GetOutBoundSinrForDest(Mac48Address dest) const
  {

    std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    for( ; it != m_signalMap.end(); ++ it)
    {
      if ( (*it)->from == dest)
      {
        return (*it)->outSinr;
      }
    }
    //if no record has been found in the signal map, return 0
    return 0;
  }
  Ptr<SignalMap> WifiImacPhy::GetSignalMapItem (Mac48Address addr) const
  {
    Ptr<SignalMap> returnItem ;
    std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    for ( ; it != m_signalMap.end(); ++ it)
    {
      if ( (*it)->from == addr )
      {
        returnItem = (*it);
        break;
      }
    }
    return returnItem;
  }

  /* param item The item will be a row in the signal map table. According to the address in the 'from' field, if there exists this address in the 
   * current signal map, we update its attenuation and snr values. If there is no such address in the current signal map, we insert this item as a
   * new row in the signal map.
   * After updating or inserting the item, we sort the signal map according to the inbound attenuation value in ascending way.
   * added by Chuan
   */
  void WifiImacPhy::SetSignalMap(Ptr<SignalMap> item, double noise) // noise is in the unit of Watt
  {	

    item->outSinr = item->inSinr = GetPowerDbm (DATA_TX_POWER_LEVEL) + GetTxGain () - item->inBoundAttenuation - WToDbm (noise);
    if (item->inSinr < 0)
    {
      item->inSinr = item->outSinr = 0; // for those node that has the snr less than 0, we won't choose those nodes.
    }
    std::vector<Ptr<SignalMap> >::iterator it = m_signalMap.begin();
    for ( ; it != m_signalMap.end(); ++ it)
    {
      if( (*it)->from == item->from)
      {
        if ( item->inBoundAttenuation != 0)
        {
          (*it)->inBoundAttenuation= item->inBoundAttenuation;
        }
        if (item->outBoundAttenuation != 0)
        {
          (*it)->outBoundAttenuation = item->outBoundAttenuation;
        }

        if (item->noisePlusInterferenceW > 0 && (*it)->noisePlusInterferenceW != m_interference.GetNoise ())
        {
          (*it)->noisePlusInterferenceW = item->noisePlusInterferenceW;
        }

        (*it)->inSinr = item->inSinr;
        (*it)->outSinr = item->outSinr;
        (*it)->supposedInterferenceW = item->supposedInterferenceW;
        sort (m_signalMap.begin (), m_signalMap.end (), myCompare);
        NotifySignalMapChange();
        return;
      }
    }

    // initialization
    item->noisePlusInterferenceW = m_interference.GetNoise ();
    bool added = false;
    for (it = m_signalMap.begin (); it != m_signalMap.end (); ++ it)
    {
      if ( (*it)->inBoundAttenuation < item->inBoundAttenuation ) 
      {
        m_signalMap.insert (it, item);
        added = true;
        break;
      }
    }

    if (m_signalMap.size () == 0 || added == false)// if no element in the vector, just add the item into the vector
    {
      m_signalMap.push_back(item);
    }
    sort (m_signalMap.begin (), m_signalMap.end (), myCompare);
    NotifySignalMapChange();

    return;
  }


  Mac48Address WifiImacPhy::NeighborSelectionBySinr(double lowerBound)
  {
    Mac48Address returnAddress = Mac48Address::GetBroadcast();
    if (m_signalMap.size() == 0)
    {
      return returnAddress;
    }

    sort (m_signalMap.begin (), m_signalMap.end (), outSinrDecreasingCompare);
    std::vector< Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    Ptr<SignalMap> currentNode = (*it);
    for( ; it != m_signalMap.end(); ++ it)
    {
      if ((*it)->outSinr > lowerBound && ((*it)->outSinr - lowerBound) < (currentNode->outSinr - lowerBound))
      {
        currentNode = (*it);
        returnAddress = (*it)->from;
      }
    }
    sort (m_signalMap.begin (), m_signalMap.end (), myCompare);

    if ( returnAddress != Mac48Address::GetBroadcast())
    {
      double distance = m_channel->GetDistanceBetweenNodes (m_self, returnAddress);
#ifndef RID_INITIAL_ER
      double edgePowerDbm = m_channel->GetRxPowerByDistance (GetPowerDbm (0) + GetTxGain (), distance * 1.5) + GetRxGain (); 
      double initialErInterferenceW = DbmToW (edgePowerDbm);
#endif
#ifdef RID_INITIAL_ER
      double rxPowerDbm = m_channel->GetRxPowerByDistance (GetPowerDbm (0) + GetTxGain (), distance) + GetRxGain ();
      double niDbm = rxPowerDbm - m_snr;
      double niW = DbmToW (niDbm);
      double initialErInterferenceW = niW - NOISE_POWER_W;
#endif
      m_initialErCallback (m_self.GetNodeId (), returnAddress.GetNodeId (),initialErInterferenceW);
    }

    std::vector<TdmaLink> relatedLinks = Simulator::FindRelatedLinks (m_self.ToString ());
    for (std::vector<TdmaLink>::iterator _it = relatedLinks.begin (); _it != relatedLinks.end (); ++ _it)
    {
      uint16_t sender = Mac48Address (_it->senderAddr.c_str ()).GetNodeId ();
      uint16_t receiver = Mac48Address (_it->receiverAddr.c_str ()).GetNodeId ();
      double distance = m_channel->GetDistanceBetweenNodes (Mac48Address (_it->senderAddr.c_str ()), Mac48Address (_it->receiverAddr.c_str ()));
#ifndef RID_INITIAL_ER
      double edgePowerDbm = m_channel->GetRxPowerByDistance (GetPowerDbm (0) + GetTxGain (), distance * 1.5) + GetRxGain ();
      double initialErInterferenceW = DbmToW (edgePowerDbm);
#endif

#ifdef RID_INITIAL_ER
      double rxPowerDbm = m_channel->GetRxPowerByDistance (GetPowerDbm (0) + GetTxGain (), distance) + GetRxGain ();
      double niDbm = rxPowerDbm - m_snr;
      double niW = DbmToW (niDbm);
      double initialErInterferenceW = niW - NOISE_POWER_W;
#endif

      m_initialErCallback (sender, receiver, initialErInterferenceW);
    }
    return returnAddress;
  }


  /*
   * param sinr The reference sinr by which we select neighbors.
   * return the mac address of the selected node.
   * added by Chuan
   */
  Mac48Address WifiImacPhy::NeighborSelectionBySinr(double lowerBound, double upperBound)
  {
    Mac48Address returnAddress = Mac48Address::GetBroadcast();
    if (m_signalMap.size() == 0)
    {
      return returnAddress;
    }

    sort (m_signalMap.begin (), m_signalMap.end (), outSinrDecreasingCompare);
    std::vector< Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    Ptr<SignalMap> currentNode = (*it);
    for( ; it != m_signalMap.end(); ++ it)
    {
      if ((*it)->outSinr > lowerBound && ((*it)->outSinr - lowerBound) < (currentNode->outSinr - lowerBound) && (*it)->outSinr <= upperBound)
      {
        currentNode = (*it);
        returnAddress = (*it)->from;
      }
    }
    sort (m_signalMap.begin (), m_signalMap.end (), myCompare);

    return returnAddress;
  }

  /* Whenever the signal map changes, we invoke this method such that the trace source mechanism works.
   * added by Chuan
   */
  void 
    WifiImacPhy::NotifySignalMapChange()
    {
      m_signalMapCallback(this);
    }

  void WifiImacPhy::SetAddress (Mac48Address addr)
  {
    m_self = addr;
    m_interference.SetAddress (m_self);
  }

  double WifiImacPhy::GetRxPowerDbm () const
  {
    return m_rxPowerDbm;
  }

  void
    WifiImacPhy::StartReceivePacket (uint32_t deviceIndex, Ptr<Packet> packet,
        double rxPowerDbm,
        WifiMode txMode,
        enum WifiPreamble preamble)
    {
      NS_LOG_FUNCTION (this << packet << rxPowerDbm << txMode << preamble);


      rxPowerDbm += m_rxGainDb;
      m_rxPowerDbm = rxPowerDbm;
      double rxPowerW = DbmToW (rxPowerDbm);
      Time rxDuration = CalculateTxDuration (packet->GetSize (), txMode, preamble);
      Time endRx = Simulator::Now () + rxDuration;
      //Mac48Address myAddr = GetChannel()->GetDevice(deviceIndex) ->GetObject<WifiNetDevice> ()->GetMac()->GetObject<AdhocWifiMac> ()-> GetAddress ();

      WifiMacHeader hdr;
      packet->PeekHeader(hdr); 
      /*
      if ( Simulator::Now () > Simulator::LearningTimeDuration && m_self == hdr.GetAddr1 ())
      {
        std::cout<<"__from: "<<hdr.GetAddr2 () <<" to: "<<hdr.GetAddr1 () << " rxpower: "<< m_rxPowerDbm << std::endl;
      }
      */

      Ptr<InterferenceHelper::Event> event;
      // Register the concurrent transmitters.
      event = m_interference.Add (packet->GetSize (), txMode, preamble, rxDuration, rxPowerW);

      switch (m_state->GetState ())
      {
        case WifiImacPhy::SWITCHING:
          NS_LOG_DEBUG ("drop packet because of channel switching");
          NotifyRxDrop (packet);
          /*
           * Packets received on the upcoming channel are added to the event list
           * during the switching state. This way the medium can be correctly sensed
           * when the device listens to the channel for the first time after the
           * switching e.g. after channel switching, the channel may be sensed as
           * busy due to other devices' tramissions started before the end of
           * the switching.
           */
          if (endRx > Simulator::Now () + m_state->GetDelayUntilIdle ())
          {
            // that packet will be noise _after_ the completion of the
            // channel switching.
            goto maybeCcaBusy;
          }
          break;
        case WifiImacPhy::RX:
          NS_LOG_DEBUG ("drop packet because already in Rx (power=" <<
              rxPowerW << "W)");
          NotifyRxDrop (packet);
          if (endRx > Simulator::Now () + m_state->GetDelayUntilIdle ())
          {
            // that packet will be noise _after_ the reception of the
            // currently-received packet.
            goto maybeCcaBusy;
          }
          break;
        case WifiImacPhy::TX:
          NS_LOG_DEBUG ("drop packet because already in Tx (power=" <<
              rxPowerW << "W)");
          NotifyRxDrop (packet);
          if (endRx > Simulator::Now () + m_state->GetDelayUntilIdle ())
          {
            // that packet will be noise _after_ the transmission of the
            // currently-transmitted packet.
            goto maybeCcaBusy;
          }
          break;
        case WifiImacPhy::CCA_BUSY:
        case WifiImacPhy::IDLE:
          // if the simulation is still in the learning process, receive packets as usual, if it is not in the learning process,
          // only when the node is active, can we receive packets
          if (rxPowerW >= m_edThresholdW && (Simulator::Now () < Simulator::LearningTimeDuration || m_nodeActiveStatusCallback () == true || GetChannelNumber () == CONTROL_CHANNEL))
          { 
            NS_LOG_DEBUG ("sync to signal (power=" << rxPowerW << "W)");
            Mac48Address addr2;
            m_currentReceivedPowerDbm = rxPowerDbm; 

            //------------------------------------ Enable interference sampling -----------------------------------------//
            //this is needed because our minimum variance controller need the N+I value.
            //Only enbale this interference sampling in data channel.
            if (((hdr.IsData () && !hdr.GetAddr1 ().IsGroup ()) || hdr.IsAck () ) && 
                hdr.GetAddr1 () == m_self && Simulator::Now () >= Simulator::LearningTimeDuration && GetChannelNumber () == DATA_CHANNEL )
            {
              if (hdr.IsData ())
              {
                m_interference.NotifyDataStartReceiving ();
              }
              else if (hdr.IsAck ())
              {
                m_interference.NotifyAckStartReceiving ();
              }
            }
            if (Simulator::Now () > Simulator::LearningTimeDuration && GetChannelNumber () == 2)
            {
              if ( m_senderInTxErCallback (hdr.GetAddr2 ()) == true )
              {
                m_interference.ControlChannel_NotifyDataStartReceiving ();
              }
            }

            // sync to signal
            m_state->SwitchToRx (rxDuration);
            NS_ASSERT (m_endRxEvent.IsExpired ());
            NotifyRxBegin (packet);
            m_interference.NotifyRxStart ();
            m_interference.UpdateReceivedPowerW ( rxPowerW);

            WifiMacHeader hdr;
            packet->PeekHeader(hdr);
            //___________________________________________________________________________________________________________________________
            //____________________________Signal Map Logic_______________________________________________________________________________
            //___________________________________________________________________________________________________________________________
            //                  Only when in the learning process and in control channel, update signal map.
            //___________________________________________________________________________________________________________________________
            if (Simulator::Now () < Simulator::LearningTimeDuration && GetChannelNumber () == CONTROL_CHANNEL)
            {
              //std::cout<<" receiving message from control channel. "<< std::endl; //working!
              Ptr<SignalMap> item = CreateObject<SignalMap> ();
              double attenuation = 0.0;
              //-----------------------Broadcast message in the learning process ----------------------------//
              attenuation = WifiImacPhy::GetPowerDbm ( m_initialPowerLevel ) + m_txGainDb - rxPowerDbm;
              if ( hdr.IsData () )
              {
                item->from = hdr.GetAddr2();
              }
              else if (hdr.IsAck ())
              {
                ReceiverAddressTag receiverAddress; // receiver means data packet receiver, i.e., the ack sender
                if (packet->FindFirstMatchingByteTag (receiverAddress) )
                {
                  addr2 = receiverAddress.Get ();
                  item->from = addr2;
                }
              }
              item->inBoundAttenuation = attenuation;
              item->outBoundAttenuation = item->inBoundAttenuation; //FOR symmetric channel, the out bound attenuation is the same as the inBoundAttenuation
              item->noisePlusInterferenceW = m_interference.GetNoise (); // only noise?
              item->supposedInterferenceW = DbmToW (GetPowerDbm (DATA_TX_POWER_LEVEL) + GetTxGain () - attenuation); 
              WifiImacPhy::SetSignalMap(item, m_interference.GetNoise ());
            }
            //_______________________________________________________________________________________________________
            //        Control channel logic after the learning process
            //_______________________________________________________________________________________________________
            else if ( Simulator::Now () >= Simulator::LearningTimeDuration && GetChannelNumber () == CONTROL_CHANNEL)
            {
            }
            //_______________________________________________________________________________________________________
            //        Data channel logic after the learning process
            //_______________________________________________________________________________________________________
            else if ( Simulator::Now () >= Simulator::LearningTimeDuration && GetChannelNumber () == DATA_CHANNEL)
            {
            }

            m_endRxEvent = Simulator::Schedule (rxDuration, &WifiImacPhy::EndReceive, this,
                packet,
                event);
          }
          else
          {
            NS_LOG_DEBUG ("drop packet because signal power too Small (" <<
                rxPowerW << "<" << m_edThresholdW << ")");
            NotifyRxDrop (packet);
            goto maybeCcaBusy;
          }
          break;
      }

      return;

maybeCcaBusy:
      // We are here because we have received the first bit of a packet and we are
      // not going to be able to synchronize on it
      // In this model, CCA becomes busy when the aggregation of all signals as
      // tracked by the InterferenceHelper class is higher than the CcaBusyThreshold

      Time delayUntilCcaEnd = m_interference.GetEnergyDuration (m_ccaMode1ThresholdW);
      if (!delayUntilCcaEnd.IsZero ())
      {
        m_state->SwitchMaybeToCcaBusy (delayUntilCcaEnd);
      }
    }

  void WifiImacPhy::SendPacket (Ptr<const Packet> packet, WifiMode mode, enum WifiPreamble preamble, double txPower)
  {
    NS_LOG_FUNCTION (this << packet << txMode << preamble << txPower);
    /* Transmission can happen if:
     *  - we are syncing on a packet. It is the responsability of the
     *    MAC layer to avoid doing this but the PHY does nothing to
     *    prevent it.
     *  - we are idle
     */

    NS_ASSERT (!m_state->IsStateTx () && !m_state->IsStateSwitching ());

    Ptr<Packet> pkt = packet->Copy();


    WifiMacHeader hdr;
    packet->PeekHeader (hdr);

    if (hdr.GetAddr1() != Mac48Address::GetBroadcast())
    {
      //dest's outbound snr
      OutBoundSnrTag snrTag;
      snrTag.Set (GetSinrForDest (hdr.GetAddr1()));
      pkt->AddByteTag (snrTag);
      AttenuationTag attenuationTag;
      attenuationTag.SetAttenuation (GetAttenuationForDest(hdr.GetAddr1()));
      pkt->AddByteTag (attenuationTag);

    }
    else
    {
      OutBoundSnrTag snrTag;
      snrTag.Set (0.0);
      pkt->AddByteTag (snrTag);
      AttenuationTag attenuationTag;
      attenuationTag.SetAttenuation (0.0);
      pkt->AddByteTag (attenuationTag);

    }
    Time txDuration = CalculateTxDuration (pkt->GetSize (), mode, preamble);
    if (Simulator::Now () >Simulator::LearningTimeDuration)
    {
      if (GetChannelNumber () == CONTROL_CHANNEL)
      {
        std::cout<<"control_txDuration: "<< txDuration <<" time: "<<Simulator::Now () <<" packet length: "<< pkt->GetSize ()<<" hdr length: "<< hdr.GetSize () << std::endl;
      }
    }
    if (m_state->IsStateRx ())
    {
      m_endRxEvent.Cancel ();
      m_interference.NotifyRxEnd ();
    }
    NotifyTxBegin (pkt);
    uint32_t dataRate500KbpsUnits = mode.GetDataRate () / 500000;
    bool isShortPreamble = (WIFI_PREAMBLE_SHORT == preamble);
    NotifyMonitorSniffTx (pkt, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble);

    //std::cout<<" using double, "<< " state: "<< m_state->GetState ()<<" channel: "<<GetChannelNumber () << std::endl;
    m_state->SwitchToTx (txDuration, pkt, mode, preamble, DATA_TX_POWER_LEVEL);
    if (hdr.GetAddr1 ().IsGroup ()) // in the initial process, if a data is broadcast data, send in power level m_initialPowerLevel, otherwise, send in normal powerlevel
    {
      if (Simulator::Now () < Simulator::LearningTimeDuration )
      {
        txPower = m_initialPowerLevel; // send packet in a large enough power level to enable the learning process 
      }
    }
    m_channel->Send (this, pkt, txPower + m_txGainDb, mode, preamble);// by default, txPower (powerlevel) is 0. See MacLow::ForwardDown
  }
  void
    WifiImacPhy::SendPacket (Ptr<const Packet> packet, WifiMode txMode, WifiPreamble preamble, uint8_t txPower)
    {
      if (GetChannelNumber () == CONTROL_CHANNEL && Simulator::Now () < Simulator::LearningTimeDuration)
      {
        txPower = m_initialPowerLevel;
      }
      NS_LOG_FUNCTION (this << packet << txMode << preamble << (uint32_t)txPower);
      /* Transmission can happen if:
       *  - we are syncing on a packet. It is the responsability of the
       *    MAC layer to avoid doing this but the PHY does nothing to
       *    prevent it.
       *  - we are idle
       */

      NS_ASSERT (!m_state->IsStateTx () && !m_state->IsStateSwitching ());

      Ptr<Packet> pkt = packet->Copy();


      WifiMacHeader hdr;
      packet->PeekHeader (hdr);

      if (hdr.GetAddr1() != Mac48Address::GetBroadcast())
      {
        //dest's outbound snr
        OutBoundSnrTag snrTag;
        snrTag.Set (GetSinrForDest (hdr.GetAddr1()));
        pkt->AddByteTag (snrTag);
        AttenuationTag attenuationTag;
        attenuationTag.SetAttenuation (GetAttenuationForDest(hdr.GetAddr1()));
        pkt->AddByteTag (attenuationTag);

      }
      else
      {
        OutBoundSnrTag snrTag;
        snrTag.Set (0.0);
        pkt->AddByteTag (snrTag);
        AttenuationTag attenuationTag;
        attenuationTag.SetAttenuation (0.0);
        pkt->AddByteTag (attenuationTag);

      }
      Time txDuration = CalculateTxDuration (pkt->GetSize (), txMode, preamble);
      if (Simulator::Now () >Simulator::LearningTimeDuration)
      {
        if (GetChannelNumber () == CONTROL_CHANNEL)
        {
          std::cout<<"control_txDuration: "<< txDuration <<" time: "<<Simulator::Now () <<" packet length: "<< pkt->GetSize ()<<" hdr length: "<< hdr.GetSize () << std::endl;
        }
      }
      if (m_state->IsStateRx ())
      {
        m_endRxEvent.Cancel ();
        m_interference.NotifyRxEnd ();
      }
      NotifyTxBegin (pkt);
      uint32_t dataRate500KbpsUnits = txMode.GetDataRate () / 500000;
      bool isShortPreamble = (WIFI_PREAMBLE_SHORT == preamble);
      NotifyMonitorSniffTx (pkt, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble);
      m_state->SwitchToTx (txDuration, pkt, txMode, preamble, DATA_TX_POWER_LEVEL); //Notice, the last parameter is used only for logging purpose as far as I know.
      if (hdr.GetAddr1 ().IsGroup ()) // in the initial process, if a data is broadcast data, send in power level m_initialPowerLevel, otherwise, send in normal powerlevel
      {
        if (Simulator::Now () < Simulator::LearningTimeDuration )
        {
          txPower = m_initialPowerLevel; // send packet in a large enough power level to enable the learning process 
          //std::cout<<" sending using: "<< GetPowerDbm (txPower) + m_txGainDb << std::endl;
        }
      }
      m_channel->Send (this, pkt, GetPowerDbm (txPower) + m_txGainDb, txMode, preamble);// by default, txPower (powerlevel) is 0. See MacLow::ForwardDown


    }

  uint32_t
    WifiImacPhy::GetNModes (void) const
    {
      return m_deviceRateSet.size ();
    }
  WifiMode
    WifiImacPhy::GetMode (uint32_t mode) const
    {
      return m_deviceRateSet[mode];
    }
  uint32_t
    WifiImacPhy::GetNTxPower (void) const
    {
      return m_nTxPower;
    }

  void
    WifiImacPhy::Configure80211a (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 5e3; // 5.000 GHz

      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate250Kbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate48Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
    }


  void
    WifiImacPhy::Configure80211b (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 2407; // 2.407 GHz

      m_deviceRateSet.push_back (WifiPhy::GetDsssRate1Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetDsssRate2Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetDsssRate5_5Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetDsssRate11Mbps ());
    }

  void
    WifiImacPhy::Configure80211g (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 2407; // 2.407 GHz

      m_deviceRateSet.push_back (WifiPhy::GetDsssRate1Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetDsssRate2Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetDsssRate5_5Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate6Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate9Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetDsssRate11Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate12Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate18Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate24Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate36Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate48Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetErpOfdmRate54Mbps ());
    }

  void
    WifiImacPhy::Configure80211_10Mhz (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 5e3; // 5.000 GHz, suppose 802.11a

      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
    }

  void
    WifiImacPhy::Configure80211_5Mhz (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 5e3; // 5.000 GHz, suppose 802.11a

      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate1_5MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate2_25MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW5MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate13_5MbpsBW5MHz ());
    }

  void
    WifiImacPhy::ConfigureHolland (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 5e3; // 5.000 GHz
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate36Mbps ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate54Mbps ());
    }

  void
    WifiImacPhy::Configure80211p_CCH (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 5e3; // 802.11p works over the 5Ghz freq range

      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
    }

  void
    WifiImacPhy::Configure80211p_SCH (void)
    {
      NS_LOG_FUNCTION (this);
      m_channelStartingFrequency = 5e3; // 802.11p works over the 5Ghz freq range

      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate3MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate4_5MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate6MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate9MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate12MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate18MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate24MbpsBW10MHz ());
      m_deviceRateSet.push_back (WifiPhy::GetOfdmRate27MbpsBW10MHz ());
    }

  void
    WifiImacPhy::RegisterListener (WifiPhyListener *listener)
    {
      m_state->RegisterListener (listener);
    }

  bool
    WifiImacPhy::IsStateCcaBusy (void)
    {
      return m_state->IsStateCcaBusy ();
    }

  bool
    WifiImacPhy::IsStateIdle (void)
    {
      return m_state->IsStateIdle ();
    }
  bool
    WifiImacPhy::IsStateBusy (void)
    {
      return m_state->IsStateBusy ();
    }
  bool
    WifiImacPhy::IsStateRx (void)
    {
      return m_state->IsStateRx ();
    }
  bool
    WifiImacPhy::IsStateTx (void)
    {
      return m_state->IsStateTx ();
    }
  bool
    WifiImacPhy::IsStateSwitching (void)
    {
      return m_state->IsStateSwitching ();
    }

  Time
    WifiImacPhy::GetStateDuration (void)
    {
      return m_state->GetStateDuration ();
    }
  Time
    WifiImacPhy::GetDelayUntilIdle (void)
    {
      return m_state->GetDelayUntilIdle ();
    }

  Time
    WifiImacPhy::GetLastRxStartTime (void) const
    {
      return m_state->GetLastRxStartTime ();
    }

  double
    WifiImacPhy::DbToRatio (double dB) const
    {
      double ratio = pow (10.0,dB / 10.0);
      return ratio;
    }

  double
    WifiImacPhy::DbmToW (double dBm) const
    {
      double mW = pow (10.0,dBm / 10.0);
      return mW / 1000.0;
    }

  double
    WifiImacPhy::WToDbm (double w) const
    {
      return 10.0 * log10 (w * 1000.0);
    }

  double
    WifiImacPhy::RatioToDb (double ratio) const
    {
      return 10.0 * log10 (ratio);
    }

  double
    WifiImacPhy::GetEdThresholdW (void) const
    {
      return m_edThresholdW;
    }

  double WifiImacPhy::GetPowerDbmByLevel (uint8_t powerLevel) const
  {
    return GetPowerDbm (powerLevel);
  }
  double
    WifiImacPhy::GetPowerDbm (uint8_t power) const
    {
      NS_ASSERT (m_txPowerBaseDbm <= m_txPowerEndDbm);
      NS_ASSERT (m_nTxPower > 0);
      double dbm;
      if (m_nTxPower > 1)
      {
        //dbm = m_txPowerBaseDbm + power * (m_txPowerEndDbm - m_txPowerBaseDbm) / (m_nTxPower - 1);
        //dbm = m_txPowerBaseDbm + power * (40-m_txPowerBaseDbm)/256.0;
        dbm = m_txPowerBaseDbm + power;
      }
      else
      {
        NS_ASSERT_MSG (m_txPowerBaseDbm == m_txPowerEndDbm, "cannot have TxPowerEnd != TxPowerStart with TxPowerLevels == 1");
        dbm = m_txPowerBaseDbm;
      }
      return dbm;
    }

  void
    WifiImacPhy::EndReceive (Ptr<Packet> packet, Ptr<InterferenceHelper::Event> event)
    {
      NS_LOG_FUNCTION (this << packet << event);
      WifiMacHeader hdr;
      packet->PeekHeader (hdr);
      NS_ASSERT (IsStateRx ());
      NS_ASSERT (event->GetEndTime () == Simulator::Now ());

      struct InterferenceHelper::SnrPer snrPer;
      snrPer = m_interference.CalculateSnrPer (event);
      m_interference.NotifyRxEnd ();

      NS_LOG_DEBUG ("mode=" << (event->GetPayloadMode ().GetDataRate ()) <<
          ", snr=" << snrPer.snr << ", per=" << snrPer.per << ", size=" << packet->GetSize ());
      m_noiseW = m_interference.GetNoise (); // in watt;
      if (((hdr.IsData () && !hdr.GetAddr1 ().IsGroup ()) || hdr.IsAck () ) && 
          hdr.GetAddr1 () == m_self && Simulator::Now () >= Simulator::LearningTimeDuration && GetChannelNumber () == DATA_CHANNEL)// data channel
      {
        if (hdr.IsData ())
        {
          m_interference.NotifyDataEndReceiving ();
        }
        else if (hdr.IsAck ())
        {
          m_interference.NotifyAckEndReceiving ();
        }
      }
      if (Simulator::Now () > Simulator::LearningTimeDuration && GetChannelNumber () == CONTROL_CHANNEL)
      {
        m_interference.ControlChannel_NotifyDataEndReceiving ();
      }

      if (hdr.IsData () && !hdr.GetAddr1 ().IsGroup () && GetChannelNumber () == DATA_CHANNEL && hdr.GetAddr1 () == m_self)
      {
        std::cout<<"from "<<hdr.GetAddr2 ()<<" to "<< m_self<<" snr=" << 10*log10(snrPer.snr) << ", per=" << snrPer.per <<" concurrentTxNO: "<<GetConcurrentTxNo ()<< std::endl;
      }

      if ((m_random.GetValue () > snrPer.per))
      {
        NotifyRxEnd (packet);
        uint32_t dataRate500KbpsUnits = event->GetPayloadMode ().GetDataRate () / 500000;
        bool isShortPreamble = (WIFI_PREAMBLE_SHORT == event->GetPreambleType ());
        double signalDbm = RatioToDb (event->GetRxPowerW ()) + 30;
        double noiseDbm = RatioToDb (event->GetRxPowerW () / snrPer.snr) - GetRxNoiseFigure () + 30;
        NotifyMonitorSniffRx (packet, (uint16_t)GetChannelFrequencyMhz (), GetChannelNumber (), dataRate500KbpsUnits, isShortPreamble, signalDbm, noiseDbm);
        m_state->SwitchFromRxEndOk (packet, snrPer.snr, event->GetPayloadMode (), event->GetPreambleType ());
      }
      else
      {
        /* failure. */
        NotifyRxDrop (packet);
        m_state->SwitchFromRxEndError (packet, snrPer.snr);
      }
    }




  /* return the current noise power in watt
  */
  double WifiImacPhy::GetCurrentNoiseW () const
  {
    return m_noiseW;
  }
  double WifiImacPhy::GetCurrentInterferenceW ()
  {
    return m_interference.GetInterference (); //Watt
  }

  double WifiImacPhy::GetCurrentReceivedPower () const
  {
    return m_currentReceivedPowerDbm;
  }


  uint32_t WifiImacPhy::GetErSize (double maxInterferenceW) const
  {
    uint32_t erSize = 0;
    double attenuation = GetPowerDbm (DATA_TX_POWER_LEVEL) + GetTxGain () -  WToDbm(maxInterferenceW);  // the attenuation from the node at the edge of the ER region to the target node. Note that the maxInterferenceDbm is less than the m_edThresholdW. Therefore, we should increase the power level to send control message to guarantee that the nodes around the edge of the region could correctly receive the control packet.
    for (std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin (); it != m_signalMap.end (); ++ it)
    {
      if ((*it)->outBoundAttenuation <= attenuation ) //the nodes satisfy this condition is within the ER region
      {
        erSize ++;
      }
    }
    return erSize ;
  }

  bool operator != (const std::vector<SignalMap> &a, const std::vector<SignalMap> &b)
  {
    if( (a.end() - a.begin()) == (b.end() - b.begin())) //if total number of items is the same
    {
      return false;
    }

    std::vector<SignalMap>::const_iterator it_a = a.begin();
    std::vector<SignalMap>::const_iterator it_b = b.begin();
    bool flag;
    for(; it_a != a.end(); ++it_a)
    {
      flag = false;
      for( ; it_b != b.end(); ++ it_b) // for each fiitem in vector a, check whether there is an identical item in vector b.
      {
        if( it_a->from == it_b->from && it_a->outBoundAttenuation == it_b->outBoundAttenuation && it_a->inBoundAttenuation== it_b->inBoundAttenuation)
        {
          flag = true;
          break;
        }
      }
      if( flag == true) // if yes, find the next item
      {
        continue;
      }
      else // if not, the two vectors are not the same, return false;
      {
        return false;
      }
    }
    return true; // if for every item in vector a, we can find an identical item in vector b, this means these two vectors are the same


  }


  /* param currentInterference the current interference power // not used any more
   * param deltaInterference the delta interference computed accroding to the delta snr 
   * param lastErEdgeInterference the interference power at the boundray of the ER region for a specific link. Therefore, for each link, the variable 
   * @lastErEdgeInterference is separately maintained
   * return the boundary interference of the Exclusive region
   */
  double WifiImacPhy::GetErEdgeInterference (double deltaInterferenceW, double lastErEdgeInterferenceW, Mac48Address *edgeNode, bool conditionTwoMeet, bool risingAchieved) // both parameters are in the unit of Watt
  {
    bool isEdgeFound = false;
    *edgeNode = Mac48Address::GetBroadcast ();
    std::cout<<"delta interference: "<<deltaInterferenceW<<" last er edge interference power: " << lastErEdgeInterferenceW;
    m_erEdgeInterferenceW = lastErEdgeInterferenceW;
    double supposedInterferenceW = 0.0;
    if ( deltaInterferenceW < 0) // expand the ER region
    {
      for (vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin (); it != m_signalMap.end (); ++ it)
      {
        double supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*it)->inBoundAttenuation; //dBm
        supposedInterferenceW = DbmToW (supposedInterferenceDbm);
        if (supposedInterferenceW < m_erEdgeInterferenceW) // if the supposed interference is less than the last ER edge interference, we should start computing the delta interference power; 
        {
          isEdgeFound = true;
#ifdef TX_PROBABILITY_1 
          supposedInterferenceW = DbmToW (supposedInterferenceDbm);
#else
          //also consider the tx Probability;
          if (risingAchieved == false)
          {
            supposedInterferenceW = DbmToW (supposedInterferenceDbm) * m_nodeTxProbabilityCallback ( (*it)->from.GetNodeId () ); 
          }
#endif
          deltaInterferenceW += supposedInterferenceW; //Watt 
          if ( deltaInterferenceW >= 0)
          {
            m_erEdgeInterferenceW = supposedInterferenceW; // if the delta interference becomes positive, that means we reached the boundray of the new ER region; record the interference power at the boundray of the new ER region in the unit of dBm and break the loop;
            *edgeNode = (*it)->from;
            break;
          }
          /*
             else if ( deltaInterferenceW > 0 && it != m_signalMap.begin ())
             {
             supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*(it-1))->inBoundAttenuation;//Dbm
             supposedInterferenceW = DbmToW (supposedInterferenceDbm);
             m_erEdgeInterferenceW = supposedInterferenceW;
           *edgeNode = (*(it-1))->from;
           break;
           }
           */
        }
      }
      if (m_erEdgeInterferenceW != supposedInterferenceW || isEdgeFound == false)
      {
        m_erEdgeInterferenceW = supposedInterferenceW;
        *edgeNode = (*(m_signalMap.end () - 1))->from;
        NS_ASSERT (*edgeNode != Mac48Address::GetBroadcast ());
      }
    }
    else if ( deltaInterferenceW > 0) // shrink the ER region {delta interference power is positive}
    {
      vector<Ptr<SignalMap> >::const_reverse_iterator erEdgeIt;
      for (vector<Ptr<SignalMap> >::const_reverse_iterator it = m_signalMap.rbegin (); it != m_signalMap.rend (); ++ it)
      {
        double supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*it)->inBoundAttenuation;//Dbm
        supposedInterferenceW = DbmToW (supposedInterferenceDbm);
        if (supposedInterferenceW > m_erEdgeInterferenceW ) // since this time, the delta interference is positive, we want to shrink the ER region, that means when the supposed interference is greater than or equal to the last ER region edge interference, we should start computing the delta interference power. Note that here we use the reverse_iterator, so we are iterating the vector from end to the beginning
        {
          isEdgeFound = true;
#ifdef TX_PROBABILITY_1
          supposedInterferenceW = DbmToW (supposedInterferenceDbm);
#else
          //also consider the tx Probability;
          if (risingAchieved == false)
          {
            supposedInterferenceW = DbmToW (supposedInterferenceDbm) * m_nodeTxProbabilityCallback ( (*it)->from.GetNodeId () ); 
          }
#endif
          deltaInterferenceW -= supposedInterferenceW; 
          if ( deltaInterferenceW == 0) 
          {
            //supposedInterferenceW = GetPowerDbm (0) + GetTxGain () - (*(it-1))->inBoundAttenuation;
            m_erEdgeInterferenceW = supposedInterferenceW;
            *edgeNode = (*it)->from;
            break;
          }
          else if (deltaInterferenceW < 0 && it != m_signalMap.rbegin ()) // decrease too much, rollback to the previous node.
          {
#ifndef NO_PROTECTION
            if (conditionTwoMeet == true)
            {
              supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*(it))->inBoundAttenuation;//Dbm
              supposedInterferenceW = DbmToW (supposedInterferenceDbm);
              m_erEdgeInterferenceW = supposedInterferenceW;
              *edgeNode = (*(it))->from;
              break;
            }
            supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*(it-1))->inBoundAttenuation;//Dbm
            supposedInterferenceW = DbmToW (supposedInterferenceDbm);
            m_erEdgeInterferenceW = supposedInterferenceW;
            *edgeNode = (*(it-1))->from;
            break;
#endif
#ifdef NO_PROTECTION 
            supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*(it))->inBoundAttenuation;//Dbm
            supposedInterferenceW = DbmToW (supposedInterferenceDbm);
            m_erEdgeInterferenceW = supposedInterferenceW;
            *edgeNode = (*(it))->from;
            break;
#endif
          }
          /*
             else if ( deltaInterferenceW < 0)  // if the deltaInterferenceW becomes negative, roll back to the previous node and regard the node as the new boundray node.
             {
             double supposedInterferenceDbm = GetPowerDbm (0) + GetTxGain () - (*(it-1))->inBoundAttenuation;//dBm
             supposedInterferenceW = DbmToW (supposedInterferenceDbm); 
             m_erEdgeInterferenceW = supposedInterferenceW;
             break;
             }
             */
        }
      }
      if ( m_erEdgeInterferenceW != supposedInterferenceW || isEdgeFound == false) // if these two variable does not equal to each other, that means we have computed all the neighbors in the SignalMap. In this case, we can only set the ER region as the coverage area of the coverage of the signal map.
      {
        m_erEdgeInterferenceW = supposedInterferenceW;
        *edgeNode = (*(m_signalMap.begin ()))->from;
        NS_ASSERT (*edgeNode != Mac48Address::GetBroadcast ());
        //m_erEdgeInterferenceW = DbmToW (GetPowerDbm (0)); // this means the ER is empty
      }
    }
    double distance = m_channel->GetDistanceBetweenNodes(m_self, *edgeNode);
    std::cout<<" new edge interference: "<< m_erEdgeInterferenceW<<" er_radius: "<<distance<< " er_size: "<< Simulator::ListNodesInEr (m_self.ToString (), m_erEdgeInterferenceW).size ()<<std::endl;
    if ( lastErEdgeInterferenceW == m_erEdgeInterferenceW && deltaInterferenceW != 0)
    {
      std::cout<<" equal  happens! "<<" is edge found? "<<isEdgeFound <<std::endl;
    }
    return m_erEdgeInterferenceW;  // returned boundray interference is in the unit of Watt
  }


  void WifiImacPhy::SetNodeActiveStatusCallback (NodeActiveStatusCallback callback)
  {
    m_nodeActiveStatusCallback = callback;
  }

  void WifiImacPhy::SetSenderInTxErCallback (SenderInTxErCallback callback)
  {
    m_senderInTxErCallback = callback;
  }

  void WifiImacPhy::ComputeSampledInterferenceW ()
  {
    return m_interference.ComputeSampledInterferenceW ();
  }

  double WifiImacPhy::ComputeInterferenceWhenReceivingData ()
  {
    return m_interference.ComputeInterferenceWhenReceivingData (); 
  }
  double WifiImacPhy::ControlChannel_ComputeInterferenceWhenReceivingData ()
  {
    return m_interference.ControlChannel_ComputeInterferenceWhenReceivingData (); 
  }
  double WifiImacPhy::ComputeInterferenceWhenReceivingAck ()
  {
    return m_interference.ComputeInterferenceWhenReceivingAck (); 
  }
  uint32_t WifiImacPhy::GetConcurrentTxNo () const
  {
    return m_interference.GetConcurrentTxNo ();
  }
  void WifiImacPhy::RegisterSignalMap ()
  {  
    std::vector<Ptr<SignalMap> >::const_iterator it = m_signalMap.begin();
    for( ; it != m_signalMap.end(); ++ it)
    {
      Simulator::AddSignalMapItem (m_self.ToString (),(*it)->from.ToString (), (*it)->outBoundAttenuation, (*it)->inBoundAttenuation, (*it)->outSinr, (*it)->inSinr, (*it)->noisePlusInterferenceW, (*it)->supposedInterferenceW);
    }
  }

  void WifiImacPhy::SetNodeInitialErCallback ( InitialErCallback callback )
  {
    m_initialErCallback = callback;
  }

  Mac48Address WifiImacPhy::GetAddress ()
  {
    return m_self;
  }

  // Get node txProbability from the MacLow class
  void WifiImacPhy::SetNodeTxProbabilityCallback (NodeTxProbabilityCallback callback)
  {
    m_nodeTxProbabilityCallback = callback;
  }

  void WifiImacPhy::SetThresholdSnr (double snr)
  {
    m_snr = snr;
  }

} // namespace ns3
