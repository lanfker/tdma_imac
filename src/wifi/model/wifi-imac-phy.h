#ifndef WIFI_IMAC_PHY_H
#define WIFI_IMAC_PHY_H

#include <stdint.h>
#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/packet.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/random-variable.h"
#include "wifi-phy.h"
#include "wifi-mode.h"
#include "wifi-preamble.h"
#include "wifi-phy-standard.h"
#include "interference-helper.h"
#include <vector>
#include "ns3/address.h"
#include "ns3/mac48-address.h"
#include "ns3/traced-value.h"
#include "math-helper.h"
#include "controller.h"
#include "settings.h"

namespace ns3 {




  class RandomUniform;
  class RxEvent;
  class WifiImacChannel;
  class WifiPhyStateHelper;

  /**
   * \brief 802.11 PHY layer model
   * \ingroup wifi
   *
   * This PHY implements a model of 802.11a. The model
   * implemented here is based on the model described
   * in "Yet Another Network Simulator",
   * (http://cutebugs.net/files/wns2-yans.pdf).
   *
   *
   * This PHY model depends on a channel loss and delay
   * model as provided by the ns3::PropagationLossModel
   * and ns3::PropagationDelayModel classes, both of which are
   * members of the ns3::YansWifiChannel class.
   */

  class SignalMap: public Object
  {
    public:
      // from: where the packet is originated from. The node that has this mac48address should be a neighbor of
      // the current node. By saying neighbor, I mean the current node can receive the neighbor's packet with 
      // a reception power >= a certain threshold.
      Mac48Address from;
      // outBoundAttenuation: Suppose A is the sender, B is the receiver. The outBoundAttenuation is the value
      // maintained by B indicating the power attenuation from A to B. Since A also needs this value, B should
      // piggyback the value to A in some manner.
      double outBoundAttenuation;
      // inBoundAttenuation: Again, A is the sender, and B is the receiver, the inBoundAttenuation is maintained
      // by A, indicating the attenuation from B to A. This value is also the outBoundAttenuation in B regarding
      // his one of his neighbor:A
      double inBoundAttenuation;
      // outSinr means: for a packet P sent by A to B, what is the sinr at B when receiving the packet p.
      double outSinr;
      // inSInr means: For a packet P send by B to A, what is the sinr A maintains when receving the 
      // packet P from B
      double inSinr;

      double noisePlusInterferenceW;// when want to guarantee the control signal can be received by a certain PDR, we need to use this

      double supposedInterferenceW; // if only the node @from is transmitting, this is the amount of receiving power.


      SignalMap();

      ~SignalMap();

      static TypeId 
        GetTypeId (void);
  };

  class WifiImacPhy : public WifiPhy
  {
    public:
      static TypeId GetTypeId (void);

      typedef Callback<bool> NodeActiveStatusCallback;
      typedef Callback<void, uint16_t, uint16_t,  double> InitialErCallback;
      typedef Callback<bool, Mac48Address> SenderInTxErCallback;
      typedef Callback<double, uint16_t> NodeTxProbabilityCallback;
      WifiImacPhy ();
      virtual ~WifiImacPhy ();

      void SetChannel (Ptr<WifiImacChannel> channel);

      /**
       * \brief Set channel number.
       *
       * Channel center frequency = Channel starting frequency + 5 MHz * (nch - 1)
       *
       * where Starting channel frequency is standard-dependent, see SetStandard()
       * as defined in IEEE 802.11-2007 17.3.8.3.2.
       *
       * YansWifiPhy can switch among different channels. Basically, YansWifiPhy
       * has a private attribute m_channelNumber that identifies the channel the
       * PHY operates on. Channel switching cannot interrupt an ongoing transmission.
       * When PHY is in TX state, the channel switching is postponed until the end
       * of the current transmission. When the PHY is in RX state, the channel
       * switching causes the drop of the synchronized packet.
       */
      void SetChannelNumber (uint16_t id);
      /// Return current channel number, see SetChannelNumber()
      uint16_t GetChannelNumber () const;
      /// Return current center channel frequency in MHz, see SetChannelNumber()
      double GetChannelFrequencyMhz () const;

      void StartReceivePacket (uint32_t deviceIndex, Ptr<Packet> packet,
          double rxPowerDbm,
          WifiMode mode,
          WifiPreamble preamble);

      void SetRxNoiseFigure (double noiseFigureDb);
      void SetTxPowerStart (double start);
      void SetTxPowerEnd (double end);
      void SetNTxPower (uint32_t n);
      void SetTxGain (double gain);
      void SetRxGain (double gain);
      void SetEdThreshold (double threshold);
      void SetCcaMode1Threshold (double threshold);
      void SetErrorRateModel (Ptr<ErrorRateModel> rate);
      void SetDevice (Ptr<Object> device);
      void SetMobility (Ptr<Object> mobility);
      double GetRxNoiseFigure (void) const;
      double GetTxGain (void) const;
      double GetRxGain (void) const;
      double GetEdThreshold (void) const;
      double GetCcaMode1Threshold (void) const;
      Ptr<ErrorRateModel> GetErrorRateModel (void) const;
      Ptr<Object> GetDevice (void) const;
      Ptr<Object> GetMobility (void);
      /*
       * create new item in signal map or update the existing signal map item
       * in descending order.
       * \param item: The item to be added or to be updated according to the value of item. 
       */
      void SetSignalMap(Ptr<SignalMap> item, double noise);
      Ptr<SignalMap> GetSignalMapItem (Mac48Address addr) const;
      /*
       * return the signal map maintained by the phy layer.
       */
      std::vector<Ptr<SignalMap> > GetSignalMap(void) const;

      /*
       * \param dest: The address of the target node of the packet which is going to sent to the dest address
       * Since we want to send back the outBoundAttenuation to a sender, for example A, when a node B gets the 
       * opportunity to send packet to A, it also try to find its inBoundAttenuation for A as A's 
       * outBoundAttenuation for B. This method is trying to get the outBoundAttenuation for a neighbor
       * indicated by the dest (mac-48-address)
       */
      double GetAttenuationForDest(Mac48Address dest) const;


      double GetOutBoundAttenuationForDest(Mac48Address dest) const;
      /*
       * This method is similar to the above one, the difference is this method trys to get the outSinr for a
       * neighbor specified by dest (mac48address)
       *
       */
      double GetSinrForDest(Mac48Address dest) const;
      double GetOutBoundSinrForDest(Mac48Address dest) const;
      /*
       * this method provides a neighbor selection standard (according to sinr=8 dB) to ensure a certain 
       * link reliability. See the relationship between the SNR and PDR for more information
       *
       */
      Mac48Address NeighborSelectionBySinr(double lowerBound);
      void ScheduleSwitchChannel (Time delay, uint16_t number); 


      virtual double GetTxPowerStart (void) const;
      virtual double GetTxPowerEnd (void) const;
      virtual uint32_t GetNTxPower (void) const;
      virtual void SetReceiveOkCallback (WifiPhy::RxOkCallback callback);
      virtual void SetReceiveErrorCallback (WifiPhy::RxErrorCallback callback);
      virtual void SendPacket (Ptr<const Packet> packet, WifiMode mode, enum WifiPreamble preamble, uint8_t txPowerLevel);
      virtual void RegisterListener (WifiPhyListener *listener);
      virtual bool IsStateCcaBusy (void);
      virtual bool IsStateIdle (void);
      virtual bool IsStateBusy (void);
      virtual bool IsStateRx (void);
      virtual bool IsStateTx (void);
      virtual bool IsStateSwitching (void);
      virtual Time GetStateDuration (void);
      virtual Time GetDelayUntilIdle (void);
      virtual Time GetLastRxStartTime (void) const;
      virtual uint32_t GetNModes (void) const;
      virtual WifiMode GetMode (uint32_t mode) const;
      virtual double CalculateSnr (WifiMode txMode, double ber) const;
      virtual Ptr<WifiChannel> GetChannel (void) const;
      virtual void ConfigureStandard (enum WifiPhyStandard standard);
      //double ExpectedPdr(double snr, double pathlossExponent );

      double GetCurrentPdr () const;
      /* return the current interference power (should be the average interference power)
      */
      double GetCurrentInterferenceW () ;
      /* return the current noise power in watt
      */
      double GetCurrentNoiseW () const;
      double GetCurrentReceivedPower () const;

      double GetTransmissionPowerDbm (double maxInterference);
      uint32_t GetErSize (double maxInterferenceW) ;

      void SendPacket (Ptr<const Packet> packet, WifiMode txMode, enum WifiPreamble preamble, double txPower);
      /* 
      */
      double GetErEdgeInterference ( double deltaInterference, double lastErEdgeInterference, Mac48Address *edgeNode, bool conditionTwoMeet, bool risingAchieved);
      struct Compare {
        bool operator () (Ptr<SignalMap> a, Ptr<SignalMap> b) { return a->inBoundAttenuation < b->inBoundAttenuation; }
      } myCompare;
      struct InterferenceCompare {
        bool operator () (Ptr<SignalMap> a, Ptr<SignalMap> b) { return a->supposedInterferenceW > b->supposedInterferenceW;}
      } myInterferenceCompare;

      struct SinrCompare {
        bool operator () (Ptr<SignalMap> a, Ptr<SignalMap> b) { return a->outSinr > b->outSinr; }
      } outSinrDecreasingCompare;


      /* Because the default method @GetPowerDbm is private, we introduce another method, of course, public, to let other class be 
       * able to call.
       */
      double GetPowerDbmByLevel (uint8_t powerLevel) const;

      /* 
      */
      void SetNodeActiveStatusCallback (NodeActiveStatusCallback callback);
      void SetNodeInitialErCallback ( InitialErCallback callback );
      void SetSenderInTxErCallback (SenderInTxErCallback callback);
      void SetNodeTxProbabilityCallback (NodeTxProbabilityCallback callback);
      void ComputeSampledInterferenceW ();
      double DbmToW (double dbm) const;
      double WToDbm (double w) const;
      double ComputeInterferenceWhenReceivingData ();
      double ControlChannel_ComputeInterferenceWhenReceivingData ();
      double ComputeInterferenceWhenReceivingAck ();
      uint32_t GetConcurrentTxNo () const;
      /* Store all the signal map items to the @ns3::Simulator class for information sharing
      */
      void RegisterSignalMap ();

      void SetAddress (Mac48Address addr);
      Mac48Address GetAddress ();
      double GetRxPowerDbm () const;
      void SetThresholdSnr (double snr);
      double GetLinkDistance (Mac48Address sender, Mac48Address receiver );
      double GetPowerDbmWithFixedSnr (Mac48Address sender, Mac48Address receiver , double snr=8.9370);
#if defined (CONVERGECAST)
      uint32_t m_linkParent[NETWORK_SIZE];
      double m_linkRequirement[NETWORK_SIZE];
#endif
      std::string IntToMacAddress (uint16_t nodeId);
      /********************************************************PRIVATE**********************************************************/
    private:
      WifiImacPhy (const WifiImacPhy &o);
      virtual void DoDispose (void);
      void Configure80211a (void);
      void Configure80211b (void);
      void Configure80211g (void);
      void Configure80211_10Mhz (void);
      void Configure80211_5Mhz ();
      void ConfigureHolland (void);
      void Configure80211p_CCH (void);
      void Configure80211p_SCH (void);
      double GetEdThresholdW (void) const;
      double DbToRatio (double db) const;
      double RatioToDb (double ratio) const;
      double GetPowerDbm (uint8_t power) const;
      void EndReceive (Ptr<Packet> packet, Ptr<InterferenceHelper::Event> event);
      TracedValue<int> m_intTrace;
      /*
       * whenever a signal map item changes, this method is invoked. By doing this, the tracing system of the 
       * signal map variable is enabled.
       */
      void NotifySignalMapChange();


    private:


      double   m_edThresholdW;
      double   m_ccaMode1ThresholdW;
      double   m_txGainDb;
      double   m_rxGainDb;
      double   m_txPowerBaseDbm;
      double   m_txPowerEndDbm;
      uint32_t m_nTxPower;

      Ptr<WifiImacChannel> m_channel;
      uint16_t m_channelNumber;
      Ptr<Object> m_device;
      Ptr<Object> m_mobility;
      std::vector<Ptr<SignalMap> > m_signalMap;
      TracedCallback<Ptr<WifiImacPhy> >  m_signalMapCallback;
      NodeActiveStatusCallback m_nodeActiveStatusCallback;
      InitialErCallback m_initialErCallback;
      SenderInTxErCallback m_senderInTxErCallback;
      NodeTxProbabilityCallback m_nodeTxProbabilityCallback;
      WifiModeList m_deviceRateSet;

      EventId m_endRxEvent;
      UniformVariable m_random;
      /// Standard-dependent center frequency of 0-th channel, MHz
      double m_channelStartingFrequency;
      Ptr<WifiPhyStateHelper> m_state;
      InterferenceHelper m_interference;
      Time m_channelSwitchDelay;
      MathHelper m_mathHelper;


      double m_erEdgeInterferenceW; 
      Time m_sifsTime;
      Time m_ctsTxTime;
      Time m_ackTxTime;
      uint8_t m_initialPowerLevel;
      Time m_dataTxTime;
      double m_currentReceivedPowerDbm; 
      double m_currentInterferenceW;
      double m_noiseW;
      double m_pathlossExponent;
      uint8_t m_lastPowerLevel;
      Mac48Address m_self;
      Time m_lifeTime;
      double m_initialErEdgeInterferenceW;
      double m_rxPowerDbm;
      double m_snr;
  };

  bool operator != (const std::vector<SignalMap> &a, const std::vector<SignalMap> &b);

} // namespace ns3


#endif /* WIFI_IMAC_PHY_H */
