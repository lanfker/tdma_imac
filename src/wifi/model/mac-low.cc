/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 * Copyright (c) 2009 MIRKO BANCHI *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Mirko Banchi <mk.banchi@gmail.com>
 */
#include "ack-sequence-no-tag.h"
#include <iostream>
#include <vector>
#include <sstream>
#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/double.h"
#include "controller.h"
#include "mac-low.h"
#include "wifi-phy.h"
#include "wifi-mac-trailer.h"
#include "qos-utils.h"
#include "wifi-imac-channel.h"
#include "edca-txop-n.h"
#include "interference-tag.h"
#include "math-helper.h"
#include "wifi-net-device.h"
#include "receiver-address-tag.h"
#include "tx-power-dbm-tag.h"
#include "ns3/mac48-address.h"
#include "wifi-imac-phy.h"
#include "noise-plus-interference-tag.h"
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include "quick-sort.h"
NS_LOG_COMPONENT_DEFINE ("MacLow");

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT std::clog << "[mac=" << m_self << "] "


namespace ns3 {


  TypeId SnrTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::SnrTag")
      .SetParent<Tag> ()
      .AddConstructor<SnrTag> ()
      .AddAttribute ("Snr", "The snr of the last packet received",
          DoubleValue (0.0),
          MakeDoubleAccessor (&SnrTag::Get),
          MakeDoubleChecker<double> ())
      ;
    return tid;
  }
  TypeId SnrTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t SnrTag::GetSerializedSize (void) const
  {
    return sizeof (double);
  }
  void SnrTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_snr);
  }
  void SnrTag::Deserialize (TagBuffer i)
  {
    m_snr = i.ReadDouble ();
  }
  void SnrTag::Print (std::ostream &os) const
  {
    os << "Snr=" << m_snr;
  }
  void SnrTag::Set (double snr)
  {
    m_snr = snr;
  }
  double SnrTag::Get (void) const
  {
    return m_snr;
  }


  MacLowTransmissionListener::MacLowTransmissionListener ()
  {
  }
  MacLowTransmissionListener::~MacLowTransmissionListener ()
  {
  }
  void MacLowTransmissionListener::GotBlockAck (const CtrlBAckResponseHeader *blockAck,
      Mac48Address source)
  {
  }
  void MacLowTransmissionListener::MissedBlockAck (void)
  {
  }
  MacLowDcfListener::MacLowDcfListener ()
  {
  }
  MacLowDcfListener::~MacLowDcfListener ()
  {
  }

  MacLowBlockAckEventListener::MacLowBlockAckEventListener ()
  {
  }
  MacLowBlockAckEventListener::~MacLowBlockAckEventListener ()
  {
  }

  MacLowTransmissionParameters::MacLowTransmissionParameters ()
    : m_nextSize (0),
    m_waitAck (ACK_NONE),
    m_sendRts (false),
    m_overrideDurationId (Seconds (0))
  {
  }
  void MacLowTransmissionParameters::EnableNextData (uint32_t size)
  {
    m_nextSize = size;
  }
  void MacLowTransmissionParameters::DisableNextData (void)
  {
    m_nextSize = 0;
  }
  void MacLowTransmissionParameters::EnableOverrideDurationId (Time durationId)
  {
    m_overrideDurationId = durationId;
  }
  void MacLowTransmissionParameters::DisableOverrideDurationId (void)
  {
    m_overrideDurationId = Seconds (0);
  }
  void MacLowTransmissionParameters::EnableSuperFastAck (void)
  {
    m_waitAck = ACK_SUPER_FAST;
  }
  void MacLowTransmissionParameters::EnableBasicBlockAck (void)
  {
    m_waitAck = BLOCK_ACK_BASIC;
  }
  void MacLowTransmissionParameters::EnableCompressedBlockAck (void)
  {
    m_waitAck = BLOCK_ACK_COMPRESSED;
  }
  void MacLowTransmissionParameters::EnableMultiTidBlockAck (void)
  {
    m_waitAck = BLOCK_ACK_MULTI_TID;
  }
  void MacLowTransmissionParameters::EnableFastAck (void)
  {
    m_waitAck = ACK_FAST;
  }
  void MacLowTransmissionParameters::EnableAck (void)
  {
    m_waitAck = ACK_NORMAL;
  }
  void MacLowTransmissionParameters::DisableAck (void)
  {
    m_waitAck = ACK_NONE;
  }
  void MacLowTransmissionParameters::EnableRts (void)
  {
    m_sendRts = true;
  }
  void MacLowTransmissionParameters::DisableRts (void)
  {
    m_sendRts = false;
  }
  bool MacLowTransmissionParameters::MustWaitAck (void) const
  {
    return (m_waitAck != ACK_NONE);
  }
  bool MacLowTransmissionParameters::MustWaitNormalAck (void) const
  {
    return (m_waitAck == ACK_NORMAL);
  }
  bool MacLowTransmissionParameters::MustWaitFastAck (void) const
  {
    return (m_waitAck == ACK_FAST);
  }
  bool MacLowTransmissionParameters::MustWaitSuperFastAck (void) const
  {
    return (m_waitAck == ACK_SUPER_FAST);
  }
  bool MacLowTransmissionParameters::MustWaitBasicBlockAck (void) const
  {
    return (m_waitAck == BLOCK_ACK_BASIC) ? true : false;
  }
  bool MacLowTransmissionParameters::MustWaitCompressedBlockAck (void) const
  {
    return (m_waitAck == BLOCK_ACK_COMPRESSED) ? true : false;
  }
  bool MacLowTransmissionParameters::MustWaitMultiTidBlockAck (void) const
  {
    return (m_waitAck == BLOCK_ACK_MULTI_TID) ? true : false;
  }
  bool MacLowTransmissionParameters::MustSendRts (void) const
  {
    return m_sendRts;
  }
  bool MacLowTransmissionParameters::HasDurationId (void) const
  {
    return (m_overrideDurationId != Seconds (0));
  }
  Time MacLowTransmissionParameters::GetDurationId (void) const
  {
    NS_ASSERT (m_overrideDurationId != Seconds (0));
    return m_overrideDurationId;
  }
  bool MacLowTransmissionParameters::HasNextPacket (void) const
  {
    return (m_nextSize != 0);
  }
  uint32_t MacLowTransmissionParameters::GetNextPacketSize (void) const
  {
    NS_ASSERT (HasNextPacket ());
    return m_nextSize;
  }

  std::ostream &operator << (std::ostream &os, const MacLowTransmissionParameters &params)
  {
    os << "["
      << "send rts=" << params.m_sendRts << ", "
      << "next size=" << params.m_nextSize << ", "
      << "dur=" << params.m_overrideDurationId << ", "
      << "ack=";
    switch (params.m_waitAck)
    {
      case MacLowTransmissionParameters::ACK_NONE:
        os << "none";
        break;
      case MacLowTransmissionParameters::ACK_NORMAL:
        os << "normal";
        break;
      case MacLowTransmissionParameters::ACK_FAST:
        os << "fast";
        break;
      case MacLowTransmissionParameters::ACK_SUPER_FAST:
        os << "super-fast";
        break;
      case MacLowTransmissionParameters::BLOCK_ACK_BASIC:
        os << "basic-block-ack";
        break;
      case MacLowTransmissionParameters::BLOCK_ACK_COMPRESSED:
        os << "compressed-block-ack";
        break;
      case MacLowTransmissionParameters::BLOCK_ACK_MULTI_TID:
        os << "multi-tid-block-ack";
        break;
    }
    os << "]";
    return os;
  }


  /***************************************************************
   *         Listener for PHY events. Forwards to MacLow
   ***************************************************************/


  class PhyMacLowListener : public ns3::WifiPhyListener
  {
    public:
      PhyMacLowListener (ns3::MacLow *macLow)
        : m_macLow (macLow)
      {
      }
      virtual ~PhyMacLowListener ()
      {
      }
      virtual void NotifyRxStart (Time duration)
      {
      }
      virtual void NotifyRxEndOk (void)
      {
      }
      virtual void NotifyRxEndError (void)
      {
      }
      virtual void NotifyTxStart (Time duration)
      {
      }
      virtual void NotifyMaybeCcaBusyStart (Time duration)
      {
      }
      virtual void NotifySwitchingStart (Time duration)
      {
        m_macLow->NotifySwitchingStartNow (duration);
      }
    private:
      ns3::MacLow *m_macLow;
  };


  MacLow::MacLow ()
    : m_normalAckTimeoutEvent (),
    m_fastAckTimeoutEvent (),
    m_superFastAckTimeoutEvent (),
    m_fastAckFailedTimeoutEvent (),
    m_blockAckTimeoutEvent (),
    m_ctsTimeoutEvent (),
    m_sendCtsEvent (),
    m_sendAckEvent (),
    m_sendDataEvent (),
    m_waitSifsEvent (),
    m_currentPacket (0),
    m_listener (0),
    m_uniform (0.0, 1.0)
  {
    NS_LOG_FUNCTION (this);
    m_lastNavDuration = Seconds (0);
    m_lastNavStart = Seconds (0);
    m_promisc = false;  
#ifndef MIXED_PDR_REQUIREMENTS
    m_desiredDataPdr = DESIRED_DATA_PDR;
#endif
#ifdef MIXED_PDR_REQUIREMENTS
    double pdrArr[] = {0.7,0.8,0.9,0.95};
    uint32_t indx = rand () % 4; // 0--3
    m_desiredDataPdr = pdrArr[indx];
#endif
    m_ackPdr = DESIRED_ACK_PDR;
    m_estimatorWindow = ESTIMATION_WINDOW;
    m_ewmaCoefficient = EWMA_COEFFICIENT;
    m_nodesCountUpperBound = Simulator::NodesCountUpperBound;
    m_timeslot = MilliSeconds (TIME_SLOT_LENGTH);
    Simulator::Schedule (Simulator::LearningTimeDuration, &MacLow::CalcPriority, this);
    m_nodeActive = false;
    m_controlPacketPayload.clear ();
    m_currentTimeslot = 0; //for initialization only
    m_initialErEdgeInterferenceW = 0;
    m_dataReceiverAddr = Mac48Address::GetBroadcast ();
    m_controlMessagePriority = m_defaultPriority; 
    m_defaultPriority = MAX_CONTROL_PACKET_PRIORITY;
    m_maxBiDirectionalErChangeInformTimes = m_defaultInformTimes;
    m_defaultInformTimes = MAX_ER_INFORM_TIMES;
    m_d0Default = DEFAULT_D0_VALUE;
    m_maxD0SampleSize = MAX_D0_SAMPLE_SIZE; // 
    m_maxTimeslotInPayload = MAX_TIME_SLOT_IN_PAYLOAD; // since we are using 12 bits to express the current time slot,  it is 4096;
    m_maxSeqNo = MAX_SEQ_NO;
    m_impossibleD0Value = IMPOSSIBLE_D0_VALUE;
    m_maxItemPriority = DEFAULT_INFO_ITEM_PRIORITY;
    m_sendProbLastComputedTimeSlot = 0;
    m_selfSendingProbability = DEFAULT_TX_PROBABILITY;
    m_sendingCount = 0;
    m_previousSendingPower = (double)NORMAL_TX_POWER;
    for (uint i = 0; i < INFO_ITEM_SUM; ++ i)
    {
      m_othersControlInformationCopy[i].sender = 0;
      m_othersControlInformationCopy[i].receiver = 0;
    }
    m_packetGenreationProbability = DEFAULT_PACKET_GENERATION_PROBABILITY;
    Simulator::Schedule (Simulator::LearningTimeDuration, &MacLow::GeneratePacket, this );
    m_nextSendingSlot = 0; 
  }

  MacLow::~MacLow ()
  {
    NS_LOG_FUNCTION (this);
  }

  void MacLow::SetupPhyMacLowListener (Ptr<WifiPhy> phy)
  {
    m_phyMacLowListener = new PhyMacLowListener (this);
    phy->RegisterListener (m_phyMacLowListener);
  }


  void MacLow::DoDispose (void)
  {
    NS_LOG_FUNCTION (this);
    m_normalAckTimeoutEvent.Cancel ();
    m_fastAckTimeoutEvent.Cancel ();
    m_superFastAckTimeoutEvent.Cancel ();
    m_fastAckFailedTimeoutEvent.Cancel ();
    m_blockAckTimeoutEvent.Cancel ();
    m_ctsTimeoutEvent.Cancel ();
    m_sendCtsEvent.Cancel ();
    m_sendAckEvent.Cancel ();
    m_sendDataEvent.Cancel ();
    m_waitSifsEvent.Cancel ();
    m_phy = 0;
    m_stationManager = 0;
    delete m_phyMacLowListener;
    m_phyMacLowListener = 0;
  }

  void MacLow::CancelAllEvents (void)
  {
    NS_LOG_FUNCTION (this);
    bool oneRunning = false;
    if (m_normalAckTimeoutEvent.IsRunning ())
    {
      m_normalAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
    if (m_fastAckTimeoutEvent.IsRunning ())
    {
      m_fastAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
    if (m_superFastAckTimeoutEvent.IsRunning ())
    {
      m_superFastAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
    if (m_fastAckFailedTimeoutEvent.IsRunning ())
    {
      m_fastAckFailedTimeoutEvent.Cancel ();
      oneRunning = true;
    }
    if (m_blockAckTimeoutEvent.IsRunning ())
    {
      m_blockAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
    if (m_ctsTimeoutEvent.IsRunning ())
    {
      m_ctsTimeoutEvent.Cancel ();
      oneRunning = true;
    }
    if (m_sendCtsEvent.IsRunning ())
    {
      m_sendCtsEvent.Cancel ();
      oneRunning = true;
    }
    if (m_sendAckEvent.IsRunning ())
    {
      m_sendAckEvent.Cancel ();
      oneRunning = true;
    }
    if (m_sendDataEvent.IsRunning ())
    {
      m_sendDataEvent.Cancel ();
      oneRunning = true;
    }
    if (m_waitSifsEvent.IsRunning ())
    {
      m_waitSifsEvent.Cancel ();
      oneRunning = true;
    }
    if (oneRunning && m_listener != 0)
    {
      m_listener->Cancel ();
      m_listener = 0;
    }
  }

  void MacLow::SetPhy (Ptr<WifiPhy> phy)
  {
    m_phy = phy->GetObject<WifiImacPhy> ();
    m_phy->SetReceiveOkCallback (MakeCallback (&MacLow::ReceiveOk, this));
    m_phy->GetObject<WifiImacPhy> ()->SetSenderInTxErCallback (MakeCallback (&MacLow::CheckSenderInTxEr, this));
    m_phy->SetReceiveErrorCallback (MakeCallback (&MacLow::ReceiveError, this));
    m_phy->GetObject<WifiImacPhy> ()->SetNodeActiveStatusCallback (MakeCallback (&MacLow::GetNodeActiveStatus, this));
    m_phy->GetObject<WifiImacPhy> ()->SetNodeInitialErCallback (MakeCallback (&MacLow::SetInitialEr, this ));
    m_phy->GetObject<WifiImacPhy> ()->SetNodeTxProbabilityCallback (MakeCallback (&MacLow::GetNodeTxProbability, this));
    m_phy->GetObject<WifiImacPhy> ()->SetAddress (m_self);
    double thresholdSnr = Controller().GetSnrByPdr (m_desiredDataPdr);
    m_phy->GetObject<WifiImacPhy> ()->SetThresholdSnr (thresholdSnr);
    if (!m_phy->IsStateSwitching ())   
    {
      m_phy->SetChannelNumber (CONTROL_CHANNEL); // as by default
    }
    SetupPhyMacLowListener (phy);
  }
  void MacLow::SetWifiRemoteStationManager (Ptr<WifiRemoteStationManager> manager)
  {
    m_stationManager = manager;
  }

  void MacLow::SetAddress (Mac48Address ad)
  {
    m_self = ad;
  }
  void MacLow::SetAckTimeout (Time ackTimeout)
  {
    m_ackTimeout = ackTimeout;
  }
  void MacLow::SetBasicBlockAckTimeout (Time blockAckTimeout)
  {
    m_basicBlockAckTimeout = blockAckTimeout;
  }
  void MacLow::SetCompressedBlockAckTimeout (Time blockAckTimeout)
  {
    m_compressedBlockAckTimeout = blockAckTimeout;
  }
  void MacLow::SetCtsTimeout (Time ctsTimeout)
  {
    m_ctsTimeout = ctsTimeout;
  }
  void MacLow::SetSifs (Time sifs)
  {
    m_sifs = sifs;
  }
  void MacLow::SetSlotTime (Time slotTime)
  {
    m_slotTime = slotTime;
  }
  void MacLow::SetPifs (Time pifs)
  {
    m_pifs = pifs;
  }
  void MacLow::SetBssid (Mac48Address bssid)
  {
    m_bssid = bssid;
  }
  void MacLow::SetPromisc (void)
  {
    m_promisc = true;
  }
  Mac48Address MacLow::GetAddress (void) const
  {
    return m_self;
  }
  Time MacLow::GetAckTimeout (void) const
  {
    return m_ackTimeout;
  }
  Time MacLow::GetBasicBlockAckTimeout () const
  {
    return m_basicBlockAckTimeout;
  }
  Time MacLow::GetCompressedBlockAckTimeout () const
  {
    return m_compressedBlockAckTimeout;
  }
  Time MacLow::GetCtsTimeout (void) const
  {
    return m_ctsTimeout;
  }
  Time MacLow::GetSifs (void) const
  {
    return m_sifs;
  }
  Time MacLow::GetSlotTime (void) const
  {
    return m_slotTime;
  }
  Time MacLow::GetPifs (void) const
  {
    return m_pifs;
  }
  Mac48Address MacLow::GetBssid (void) const
  {
    return m_bssid;
  }

  void MacLow::SetRxCallback (Callback<void,Ptr<Packet>,const WifiMacHeader *> callback)
  {
    m_rxCallback = callback;
  }
  void MacLow::SetStartTxCallback (StartTxCallback callback)
  {
    m_startTxCallback = callback;
  }

  void MacLow::SetQueueEmptyCallback (BooleanCallback callback)
  {
    m_queueEmptyCallback = callback;
  }

  void MacLow::RegisterDcfListener (MacLowDcfListener *listener)
  {
    m_dcfListeners.push_back (listener);
  }


  void MacLow::StartTransmission (Ptr<const Packet> packet,
      const WifiMacHeader* hdr,
      MacLowTransmissionParameters params,
      MacLowTransmissionListener *listener)
  {
    NS_LOG_FUNCTION (this << packet << hdr << params << listener);
    /* m_currentPacket is not NULL because someone started
     * a transmission and was interrupted before one of:
     *   - ctsTimeout
     *   - sendDataAfterCTS
     * expired. This means that one of these timers is still
     * running. They are all cancelled below anyway by the
     * call to CancelAllEvents (because of at least one
     * of these two timer) which will trigger a call to the
     * previous listener's cancel method.
     *
     * This typically happens because the high-priority
     * QapScheduler has taken access to the channel from
     * one of the Edca of the QAP.
     */
    m_currentPacket = packet->Copy ();
    m_currentHdr = *hdr;

    CancelAllEvents ();
    m_listener = listener;
    m_txParams = params;

    /*
     * Here we manually set the transmission parameters to enable rts-cts-ack transmission
     * mechanism for our simulation purpose.
     * Basically, we let the sender send RTS before send data, and let the receiver send ACK after
     * receiving data from transmitters.
     * ----------by Chuan Mar 07, 2012
     */

    if (m_currentHdr.IsData () && !m_currentHdr.GetAddr1().IsGroup ()) // data packet but not broadcast packet
    {
      //m_txParams.EnableRts ();
      m_txParams.EnableAck ();
    }
    //NS_ASSERT (m_phy->IsStateIdle ());

    NS_LOG_DEBUG ("startTx size=" << GetSize (m_currentPacket, &m_currentHdr) <<
        ", to=" << m_currentHdr.GetAddr1 () << ", listener=" << m_listener);

    if (Simulator::Now () < Simulator::LearningTimeDuration && m_controlInformation.size () > 0)
    {
      TrySendControlPacket ();
    }
    else
    {
      SendDataPacket ();
    }

    /* When this method completes, we have taken ownership of the medium. */
    NS_ASSERT (m_phy->IsStateTx ());
  }

  void MacLow::ReceiveError (Ptr<const Packet> packet, double rxSnr)
  {
    NS_LOG_FUNCTION (this << packet << rxSnr);
    NS_LOG_DEBUG ("rx failed ");
    if (m_txParams.MustWaitFastAck ())
    {
      NS_ASSERT (m_fastAckFailedTimeoutEvent.IsExpired ());
      m_fastAckFailedTimeoutEvent = Simulator::Schedule (GetSifs (),
          &MacLow::FastAckFailedTimeout, this);
    }
    return;
  }

  void MacLow::NotifySwitchingStartNow (Time duration)
  {
    NS_LOG_DEBUG ("switching channel. Cancelling MAC pending events");
    m_stationManager->Reset ();
    CancelAllEvents ();
    if (m_navCounterResetCtsMissed.IsRunning ())
    {
      m_navCounterResetCtsMissed.Cancel ();
    }
    m_lastNavStart = Simulator::Now ();
    m_lastNavDuration = Seconds (0);
    m_currentPacket = 0;
    m_listener = 0;
  }

  // do not consider the case where @m_self is the sender 
  void MacLow::InitiateNextTxSlotInfo  ()
  {
    std::vector<TdmaLink> selfRelatedLinks =  Simulator::FindRelatedLinks (m_self.ToString ());
    for (std::vector<TdmaLink>::iterator it = selfRelatedLinks.begin (); it != selfRelatedLinks.end (); ++ it)
    {
      if (it->senderAddr == m_self.ToString ())
      {
        continue;
      }
      NextTxSlotInfo info;
      info.linkId = it->linkId;
      info.nextSlot = UNDEFINED_NEXT_TX_SLOT; 
      m_nextTxSlotInfo.push_back (info);
    }
  }

  // do not consider the case where @m_self is the sender 
  int64_t MacLow::GetNextTxSlot (int64_t linkId)
  {
    for (std::vector<NextTxSlotInfo>::iterator it = m_nextTxSlotInfo.begin (); it != m_nextTxSlotInfo.end (); ++ it)
    {
      if (it->linkId == linkId)
      {
        return it->nextSlot;
      }
    }
    return UNDEFINED_NEXT_TX_SLOT; // this sentence should never be executed.
  }

  void MacLow::UpdateNextRxSlot (int64_t linkId, int64_t nextRxSlot)
  {
    for (std::vector<NextTxSlotInfo>::iterator it = m_nextTxSlotInfo.begin (); it != m_nextTxSlotInfo.end (); ++ it)
    {
      if (it->linkId == linkId)
      {
        it->nextSlot = nextRxSlot;
      }
    }
  }


  /* At the very beginning, every node is in CONSERVATIVE state, 
   * If the @m_self is a receiver, and one of the links where @m_self acts as a receiver has the largest priority, stay in data channel
   * Sender always knows when to send
   * For a link, if receiver knows when to stay in data channel, the receiver do not have to be CONSERVATIVE for this link
   *
   *
   *
   */
  void MacLow::CalcPriority ()
  {
    int64_t currentSlot = (Simulator::Now ().GetNanoSeconds () - Simulator::LearningTimeDuration.GetNanoSeconds ()) / m_timeslot.GetNanoSeconds ();
    m_currentTimeslot = currentSlot;
    Time scheduleDelay = MicroSeconds (DELAY_BEFORE_SWITCH_CHANNEL); // 6.7 ms
    if (Simulator::NodesWillSend.size () != 0 && Simulator::SlotBeginningTime != Simulator::Now ())
    {
      // for a new time slot, clear the data for the previous time slot and update the slot beginning time to identify that
      // we are working on a new slot now;
      Simulator::ClearSendingNodes ();
      Simulator::SlotBeginningTime = Simulator::Now (); 
      Simulator::CountFinalControlReliability ();
      Simulator::m_nodesInDataChannel.clear ();
    }



    //--------------------------------------------------------------------------------------------------
    // For all the links the current node is related to, find the link with max priority
    //--------------------------------------------------------------------------------------------------
    std::vector<TdmaLink> linkRelatedLinks = Simulator::m_nodeLinkDetails[m_self.GetNodeId ()].relatedLinks; 
    if ( linkRelatedLinks.size () != 0)
    {
      TdmaLink maxLink = *(linkRelatedLinks.begin ());
      //Except the link with the maximum priority, try to add the rest links into preempted links, such that
      //in the future, we may save some time by not calculting for these links.
      for (std::vector<TdmaLink>::iterator it = linkRelatedLinks.begin (); it != linkRelatedLinks.end (); ++ it)
      {
        if (DoCalculatePriority (it->linkId) > DoCalculatePriority (maxLink.linkId))
        {
          maxLink = *it;
        }
      }


      //-----------------------------------------------------------------------------------------------
      //If for the max priority link, the current node is the sender, 
      //-----------------------------------------------------------------------------------------------
      // DO NOT WORRY ABOUT CONSERVATIVE PROBLEM
      if (maxLink.senderAddr == m_self.ToString ()) 
      {
        bool initialSlotTxStatus = false;
        if ( m_currentTimeslot == 0)
        {
          CollectConfilictingLinks (m_conflictingSet);
          initialSlotTxStatus = SenderComputeThePriority (m_self.ToString ());
        }
        if (m_currentTimeslot == m_nextSendingSlot || initialSlotTxStatus == true)
        {
          NS_ASSERT (m_conflictingSet.size () != 0);
          CollectConfilictingLinks (m_conflictingSet);
          SenderComputeThePriority (m_self.ToString ()); //Compute for the next sending timeslot
          Simulator::m_nodesInDataChannel.push_back (m_self.ToString());
          m_nodeActive = true; 
          // once the node knows it could be active in the slot, stop computing
          Simulator::Schedule (scheduleDelay, &MacLow::SetNodeActiveFalse, this);


          if (!m_phy->IsStateSwitching ())   
          {
            m_phy->GetObject<WifiImacPhy> ()->SetChannelNumber (DATA_CHANNEL); // switch to data channel;
            m_phy->GetObject<WifiImacPhy> ()->ScheduleSwitchChannel (scheduleDelay, CONTROL_CHANNEL); // switch to control channel;
            if (m_queueEmptyCallback () == true && m_dataReceiverAddr != Mac48Address::GetBroadcast ())
            {
              Simulator::Schedule (MicroSeconds (TIME_TO_TX_IN_SLOT),&MacLow::GenerateDataPacketAndSend, this);
            }
            else
            {
              Simulator::Schedule (MicroSeconds (TIME_TO_TX_IN_SLOT), &MacLow::m_startTxCallback, this);
            }
          }
        }
        else if (m_controlInformation.size () != 0 )
        {
          ScheduleControlSignalTransmission ();
        } 
      }
      //-----------------------------------------------------------------------------------------------
      // If for the max priority link, the current node is the receiver
      //-----------------------------------------------------------------------------------------------
      else if ( maxLink.receiverAddr == m_self.ToString ())
      {
        bool nodeStatus = false;
        int64_t nextRxSlot = GetNextTxSlot (maxLink.linkId);
        if ( nextRxSlot == UNDEFINED_NEXT_TX_SLOT)
        {
          nodeStatus = true;  //HERE, WE ARE TRYING TO BE CONSERVATIVE
        }
        else if ( nextRxSlot == m_currentTimeslot) //it's time for @m_self to receive data packet from a sender
        {
          nodeStatus = true;
          UpdateNextRxSlot (maxLink.linkId, UNDEFINED_NEXT_TX_SLOT); // Even though we set this back to initial value, meaning undefined rx timeslot. as long as the node successfully receives a data packet in this current timeslot, this value will be re-written.
        }
        else
        {
          nodeStatus = false;
        }
        //nodeStatus = SenderComputeThePriority (maxLink.senderAddr);
        if (nodeStatus == true ) // as receiver
        {
          m_nodeActive = true; 
          // once the node knows it could be active in the slot, stop computing
          Simulator::Schedule (scheduleDelay, &MacLow::SetNodeActiveFalse, this);
          if (!m_phy->IsStateSwitching ())   
          {
            m_phy->GetObject<WifiImacPhy> ()->SetChannelNumber (DATA_CHANNEL); // switch to data channel;
            m_phy->GetObject<WifiImacPhy> ()->ScheduleSwitchChannel (scheduleDelay, CONTROL_CHANNEL); // switch to control channel;
          }
        } // failed to have the maximum priority, try to send control signal
        else if (m_controlInformation.size () != 0 )
        {
          ScheduleControlSignalTransmission ();
        }
      }
    }

    // No matter the @maxLink is related to the current node or not, we need to schedule the priority calculation event for the
    // next timeslot
    if ( Simulator::Now () <= Simulator::SimulationStopTime )
    {
      Simulator::Schedule (m_timeslot, &MacLow::CalcPriority, this);
    }
    return;

  }

  void MacLow::CollectConfilictingLinks ( std::vector<int64_t> &vec)
  {
    // This for loop was belong to @CalcPriority
    for (std::vector<ErInfoItem>::iterator it = m_othersControlInformation.begin (); it != m_othersControlInformation.end (); ++ it)
    {
      uint32_t indx = it->sender * 10;
      indx += 1; // for data er
      CopyErInfoItem (it, &m_othersControlInformationCopy[indx]);
    }



    // clear pervious results
    vec.clear ();
    TdmaLink linkInfo = Simulator::m_nodeLinkDetails[m_self.GetNodeId ()].selfInitiatedLink;
    // if no link is initiated by @m_self, return;
    if ( linkInfo.linkId == 0)
    {
      return;
    }

    Mac48Address sender = m_self;
    Mac48Address receiver = Mac48Address (linkInfo.receiverAddr.c_str ());

    // if the receiver is the broadcast addr, that also means there is no valid link initiated by the sender, return 
    if ( receiver == Mac48Address::GetBroadcast ())
    {
      return;
    }

    // we set the data packet receiver address here. Later when we need to generate data packet to send at the mac layer,
    // we need this m_dataReceiverAddr
    // Note, this @m_dataReceiverAddr will not change during simulations, at least, according to our current settings. 12/26/2012
    if ( m_dataReceiverAddr == Mac48Address::GetBroadcast ())
    {
      m_dataReceiverAddr = receiver;
    }

    //????????????????????????????
    std::vector<double> ers = GetErInforItemForLink (sender, receiver, Mac48Address (linkInfo.senderAddr.c_str ()), Mac48Address (linkInfo.receiverAddr.c_str ()));// get ER information (edge interference)


    std::vector<std::string> nodesInEr;
#ifdef ENABLE_ACK_ER
    nodesInEr = Simulator::ListNodesInEr (sender.ToString (), ers[1], receiver.ToString (), ers[0]); // get all the nodes in ER. This vector also contains the sender and receiver of the target link
#else
    nodesInEr = Simulator::ListNodesInEr (receiver.ToString (), ers[0]); 
#endif

    // has not added the target link itself 
    for (std::vector<std::string>::iterator it = nodesInEr.begin (); it != nodesInEr.end (); ++ it) // for every node in ER.
    {
      std::vector<TdmaLink> relatedLinks = Simulator::m_nodeLinkDetails[Mac48Address (it->c_str ()).GetNodeId ()].relatedLinks;
      for (std::vector<TdmaLink>::iterator _it = relatedLinks.begin (); _it != relatedLinks.end (); ++ _it)
      { 
        // for all its related link, check if the link has been previously added?
        bool added = false;
        for (std::vector<int64_t>::iterator pri_it = vec.begin (); pri_it != vec.end (); ++ pri_it)
        {
          if (_it->linkId == *pri_it)//this link has been added
          {
            added = true;
            break;
          }
        }
        if ( added == false) // if not, calculate the priority
        {
          vec.push_back (_it->linkId);
        }
      }
    }


    // ____________________________________________________________________________________________________________________________
    //                      This part is similar to the Bi-directional ER logic
    // ____________________________________________________________________________________________________________________________
    std::vector<TdmaLink> allLinks = Simulator::ListAllLinks ();
    for (std::vector<TdmaLink>::iterator it = allLinks.begin (); it != allLinks.end (); ++ it)
    {
      if (it->linkId == linkInfo.linkId ) // if the current link is the target link, ignore it
      {
        continue;
      }
      bool added = false;
      for (std::vector<int64_t>::iterator pri_it = vec.begin (); pri_it != vec.end (); ++ pri_it)
      {
        if (it->linkId == *pri_it )
        {
          added = true;
          break;
        }
      }
      if (added == true)//this link has been added, ignore it
      {
        continue;
      }
      if (added == false )// if the current link has not been added yet, 
      {
        //Find nodes in ER

        std::vector<double> _ers = GetErInforItemForLink (Mac48Address (it->senderAddr.c_str ()) ,Mac48Address (it->receiverAddr.c_str ()), Mac48Address (linkInfo.senderAddr.c_str ()), Mac48Address (linkInfo.receiverAddr.c_str ()));// get ER information (edge interference)

        std::vector<std::string> _nodesInEr;

#ifdef ENABLE_ACK_ER
        _nodesInEr = Simulator::ListNodesInEr (it->senderAddr, _ers[1], it->receiverAddr, _ers[0]); // get all the nodes in ER. This vector also contains the sender and receiver of the target link
#else
        _nodesInEr = Simulator::ListNodesInEr (it->receiverAddr, _ers[0]); 
#endif
        //if the ER of the un-added link contains the target link (no matter the sender or receiver or both)
        if ( find (_nodesInEr.begin (), _nodesInEr.end (), linkInfo.senderAddr)!= _nodesInEr.end () ||
            find (_nodesInEr.begin (), _nodesInEr.end (), linkInfo.receiverAddr) != _nodesInEr.end ()) 
        {
          vec.push_back (it->linkId);
        }
      } 
    }
    // add target_link
    vec.push_back (linkInfo.linkId);
  }

  //if conflicting set size is 0, do not send packet
  //if return value is false and m_currentTimeslot = 0, do not send
  bool MacLow::SenderComputeThePriority (std::string addr)
  {
    bool returnValue = false;
    if (m_conflictingSet.size () == 0)
    {
      return false; // the node is not a sender, CONSERVATIVE? later to consider
    }
    // the linkId won't be 0
    TdmaLink linkInfo = Simulator::m_nodeLinkDetails[Mac48Address (addr.c_str ()).GetNodeId ()].selfInitiatedLink;
    int64_t maxPriorityLinkId = 0;
    for (int64_t slot = m_currentTimeslot; ; ++slot)
    {
      int64_t maxPriority = 0;
      for (std::vector<int64_t>::iterator it = m_conflictingSet.begin (); it != m_conflictingSet.end (); ++ it)
      {
        if (DoCalculatePriority (*it) > maxPriority)
        {
          maxPriorityLinkId = *it;
        }
      }
      if (maxPriorityLinkId == linkInfo.linkId ) // target link has the largest link priority
      {
        if ( m_currentTimeslot == 0)
        {
          returnValue = true; // in the first timeslot, will transmit
          continue;
        }
        else
        {
          m_nextSendingSlot = slot;
          break;
        }
      }
    }
    return returnValue;
  }


  bool MacLow::GetNodeActiveStatus () const
  {
    return m_nodeActive;
  }

  void MacLow::SetNodeActiveFalse ()
  {
    m_nodeActive = false;
  }
  /* Calculate link priority according to link.id
  */
  int64_t MacLow::DoCalculatePriority (int64_t linkId)
  {
    int64_t base = 10;
    while (true)
    {
      if ( m_currentTimeslot / base == 0)
      {
        break;
      }
      base *= 10;
    }
    int64_t seed = linkId * base + m_currentTimeslot;
    srand (seed ); // set rand seed;
    int64_t priority = rand () * Simulator::NodesCountUpperBound + linkId; 
    return abs (priority);
  }

  void MacLow::ProcessControlPayload (uint8_t buffer[], WifiMacHeader hdr)
  {

    Ptr<WifiImacPhy> imacPhy = m_phy->GetObject<WifiImacPhy> ();
    Payload payload = ParseControlPacketPayload (buffer, hdr );
    TdmaLink linkInfo = Simulator::m_nodeLinkDetails[hdr.GetAddr2 ().GetNodeId ()].selfInitiatedLink;
    if ( linkInfo.receiverAddr == m_self.ToString ())
    {
      UpdateNextRxSlot (linkInfo.linkId, payload.nextRxTimeslot);
    }
    for (std::vector<ErInfoItem>::iterator _it = payload.vec.begin (); _it != payload.vec.end (); ++ _it)
    {

      Mac48Address originalSender = Mac48Address ( IntToMacAddress (_it->sender).c_str ());
      if ( originalSender == m_self) // if is an Info Item of my own when as sender, @m_self always has the latest information
      {
        continue;
      }
      _it->itemId = (_it->sender * Simulator::NodesCountUpperBound + _it->receiver) * 10;
      _it->itemId += 1; //for data er

      //-------------------------------------------------------------------------------------
      // Check the ER edge stored locally, use the larger ER to decide if the current node
      // need to receive the ER info item or not. Use the larger Er region to decide
      //-------------------------------------------------------------------------------------
      double erEdgeW = _it->edgeInterferenceW;
      for (std::vector<ErInfoItem>::iterator it = m_othersControlInformation.begin (); it != m_othersControlInformation.end (); ++ it)
      {
        if ( it->itemId == _it->itemId)
        {
          if ( it->edgeInterferenceW < _it->edgeInterferenceW)
          {
            erEdgeW = it->edgeInterferenceW;
            break;
          }
        }
      }

      std::vector<std::string> nodesInEr;
      nodesInEr = Simulator::ListNodesInEr (originalSender.ToString (), erEdgeW);

      bool receiveOrNot = false;
      std::vector<TdmaLink> relatedLinks = Simulator::m_nodeLinkDetails[m_self.GetNodeId ()].relatedLinks;
      std::vector<std::string> nodesNeedCheck;
      for (std::vector<TdmaLink>::iterator iter = relatedLinks.begin (); iter != relatedLinks.end (); ++ iter)
      {
        if ( iter->senderAddr != m_self.ToString ())
        {
          if (iter->senderAddr == hdr.GetAddr2 ().ToString ())
          {
            receiveOrNot = true;
          }
          nodesNeedCheck.push_back (iter->senderAddr);
        }
        else if ( iter->receiverAddr != m_self.ToString ())
        {
          if ( iter->receiverAddr == hdr.GetAddr2 ().ToString ())
          {
            receiveOrNot = true;
          }
          nodesNeedCheck.push_back (iter->receiverAddr);
        }
      }
      nodesNeedCheck.push_back (m_self.ToString ());
      for (std::vector<std::string>::iterator iter = nodesNeedCheck.begin (); iter != nodesNeedCheck.end () && receiveOrNot == false; ++ iter)
      {
        if ( find (nodesInEr.begin (), nodesInEr.end (), *iter) != nodesInEr.end ())
        {
          receiveOrNot = true;
          break;
        }
      }
      if (receiveOrNot == true) 
      {
        UpdateReceivedErInfoItem (*_it, hdr.GetAddr2 ());
      }
    }

    if (payload.vec.size () != 0)
    {
      NodeMaxErItem maxErItem;
      maxErItem.address = hdr.GetAddr2 ();
      maxErItem.erEdgeInterferenceW = payload.maxErEdgeInterferenceW;
      UpdateMaxEr (maxErItem);
      Ptr<SignalMap> senderSignalMap = imacPhy-> GetSignalMapItem (hdr.GetAddr2 ()); 
      senderSignalMap->noisePlusInterferenceW =  payload.controlChannelInterferenceW + imacPhy->GetCurrentNoiseW (); //ewma of the received N+I
      imacPhy->SetSignalMap (senderSignalMap, imacPhy->GetCurrentNoiseW ());
      senderSignalMap = imacPhy-> GetSignalMapItem (hdr.GetAddr2 ()); 
    }

  }

  void MacLow::ReceiveOk (Ptr<Packet> packet, double rxSnr, WifiMode txMode, WifiPreamble preamble)
  {
    NS_LOG_FUNCTION (this << packet << rxSnr << txMode << preamble);
    /* A packet is received from the PHY.
     * When we have handled this packet,
     * we handle any packet present in the
     * packet queue.
     */
    Ptr<WifiImacPhy> imacPhy = m_phy->GetObject<WifiImacPhy> ();
    Ptr<WifiImacChannel> channel = imacPhy->GetChannel()->GetObject<WifiImacChannel> ();
    WifiMacHeader hdr; 
    packet->RemoveHeader (hdr); 

    //_____________________________________________________________________________________________________________________
    //                    WHEN receiving a control message
    //_____________________________________________________________________________________________________________________

    if (Simulator::Now () >= Simulator::LearningTimeDuration && imacPhy->GetChannelNumber () == CONTROL_CHANNEL)// after learning process, in control channel
    {
      uint8_t buffer[CONTROL_PAYLOAD_LENGTH];
      packet->CopyData (buffer, CONTROL_PAYLOAD_LENGTH);
      ProcessControlPayload (buffer, hdr);

      Simulator::ReceiverRegisterControlReliability (hdr.GetAddr2 ().ToString (), m_self.ToString ());
    }

    //_____________________________________________________________________________________________________________________
    //                       When receiving an ACK
    //_____________________________________________________________________________________________________________________
    if (hdr.IsAck ()
        && hdr.GetAddr1 () == m_self
        && m_txParams.MustWaitAck () && imacPhy->GetChannelNumber () == DATA_CHANNEL) // ack received from the data channel
    {
      NS_LOG_DEBUG ("receive ack from=" << m_currentHdr.GetAddr1 ());

      // report ACK been received and update received ack seq number
      ReceiverAddressTag receiverAddress;
      // ack, for me, after learning process, in data channel
      if (packet->FindFirstMatchingByteTag ( receiverAddress) && Simulator::Now () >= Simulator::LearningTimeDuration)
      {
        Mac48Address ackSenderAddress = receiverAddress.Get ();
        std::cout<<"4: "<<Simulator::Now ()<<" ack from " << ackSenderAddress << " to "<<m_self <<" is received" <<std::endl;
        AckSequenceNoTag ackSequenceNoTag; 
        /*  //DISABLE ACK ER AND ACK PDR 
        if (packet->FindFirstMatchingByteTag (ackSequenceNoTag) )
        {
          //for ack link, the sender is the receiver of the data link and the receiver is the sender of the data link
          //so m_self as the sender of the data link should be the first parameter
          UpdateReceivedAckPacketNumbers (m_self, ackSenderAddress, ackSequenceNoTag.Get ()); //?
        }
        */
      }


      SnrTag tag;
      packet->RemovePacketTag (tag);
      m_stationManager->ReportRxOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
          rxSnr, txMode);
      m_stationManager->ReportDataOk (m_currentHdr.GetAddr1 (), &m_currentHdr,
          rxSnr, txMode, tag.Get ());
      bool gotAck = false;
      if (m_txParams.MustWaitNormalAck ()
          && m_normalAckTimeoutEvent.IsRunning ())
      {
        m_normalAckTimeoutEvent.Cancel ();
        NotifyAckTimeoutResetNow ();
        gotAck = true;
      }
      if (m_txParams.MustWaitFastAck ()
          && m_fastAckTimeoutEvent.IsRunning ())
      {
        m_fastAckTimeoutEvent.Cancel ();
        NotifyAckTimeoutResetNow ();
        gotAck = true;
      }
      if (gotAck)
      {
        m_listener->GotAck (rxSnr, txMode);
      }
      if (m_txParams.HasNextPacket ())
      {
        m_waitSifsEvent = Simulator::Schedule (GetSifs (),
            &MacLow::WaitSifsAfterEndTx, this);
      }
    }
    else if (hdr.IsCtl ())
    {
      NS_LOG_DEBUG ("rx drop " << hdr.GetTypeString ());
    }
    //____________________________________________________________________________________________________________________________________
    //                            WHEN receiving a DATA
    //____________________________________________________________________________________________________________________________________
    else if (hdr.GetAddr1 () == m_self && imacPhy->GetChannelNumber () == DATA_CHANNEL) // data packet received in the data channel
    {
      // after learning process, data packet and in data channel
      if (Simulator::Now () >= Simulator::LearningTimeDuration && hdr.IsData ()) 
      {
        //for data, Addr2 is the data link sender, and addr1==@m_self is the data link receiver
        LinkEstimatorItem estimatorItem = GetEstimatorTableItemByNeighborAddr (hdr.GetAddr2 (), m_self);
        std::cout<<"2: "<<Simulator::Now () <<" received data packet from: "<< hdr.GetAddr2 ()<<" to: "<<m_self<<" seq: "<< hdr.GetSequenceNumber ()<<" er_edge: "<< estimatorItem.LastDataErEdgeInterferenceW<<" nodeid: "<<m_self.GetNodeId () << std::endl;
        UpdateReceivedDataPacketNumbers (hdr.GetAddr2 (), m_self, hdr.GetSequenceNumber ()); //

        uint8_t buffer[DATA_PACKET_PAYLOAD_LENGTH];
        packet->CopyData (buffer, DATA_PACKET_PAYLOAD_LENGTH);
        ProcessControlPayload (buffer, hdr);

      }
      m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr, rxSnr, txMode);
      if (hdr.IsQosData () && StoreMpduIfNeeded (packet, hdr))
      {
        /* From section 9.10.4 in IEEE802.11:
           Upon the receipt of a QoS data frame from the originator for which
           the Block Ack agreement exists, the recipient shall buffer the MSDU
           regardless of the value of the Ack Policy subfield within the
           QoS Control field of the QoS data frame. */
        if (hdr.IsQosAck ())
        {

          AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
          RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequence (), hdr.GetAddr2 (), hdr.GetQosTid ());
          RxCompleteBufferedPacketsUntilFirstLost (hdr.GetAddr2 (), hdr.GetQosTid ());
          NS_ASSERT (m_sendAckEvent.IsExpired ());
          m_sendAckEvent = Simulator::Schedule (GetSifs (), &MacLow::SendAckAfterData, this, hdr.GetAddr2 (), hdr.GetDuration (), txMode, rxSnr);
        }
        else if (hdr.IsQosBlockAck ())
        {
          AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
          /* See section 11.5.3 in IEEE802.11 for mean of this timer */
          ResetBlockAckInactivityTimerIfNeeded (it->second.first);
        }
        return;
      }
      else if (hdr.IsQosData () && hdr.IsQosBlockAck ())
      {
        /* This happens if a packet with ack policy Block Ack is received and a block ack
           agreement for that packet doesn't exist.

           From section 11.5.3 in IEEE802.11e:
           When a recipient does not have an active Block ack for a TID, but receives
           data MPDUs with the Ack Policy subfield set to Block Ack, it shall discard
           them and shall send a DELBA frame using the normal access
           mechanisms. */
        AcIndex ac = QosUtilsMapTidToAc (hdr.GetQosTid ());
        m_edcaListeners[ac]->BlockAckInactivityTimeout (hdr.GetAddr2 (), hdr.GetQosTid ());
        return;
      }
      else if (hdr.IsQosData () && hdr.IsQosNoAck ())
      {
        NS_LOG_DEBUG ("rx unicast/noAck from=" << hdr.GetAddr2 ());
      }
      else if (hdr.IsData () || hdr.IsMgt ())
      {
        // iMAC, send ack from here
        NS_LOG_DEBUG ("rx unicast/sendAck from=" << hdr.GetAddr2 ());
        NS_ASSERT (m_sendAckEvent.IsExpired ());
        //m_sendAckEvent = Simulator::Schedule (GetSifs (), &MacLow::SendAckAfterData, this, hdr.GetAddr2 (), hdr.GetDuration (), txMode, rxSnr);
        m_sendAckEvent = Simulator::Schedule (NanoSeconds (1), &MacLow::SendAckAfterData, this, hdr.GetAddr2 (), hdr.GetDuration (), txMode, rxSnr);
      }
      goto rxPacket;
    }
    else if (hdr.GetAddr1 ().IsGroup ())
    {
      if (hdr.IsData () || hdr.IsMgt ())
      {
        NS_LOG_DEBUG ("rx group from=" << hdr.GetAddr2 ());
        goto rxPacket;
      }
      else
      {
        // DROP
      }
    }
    else if (m_promisc)
    {
      NS_ASSERT (hdr.GetAddr1 () != m_self);
      if (hdr.IsData ())
      {
        goto rxPacket;
      }
    }
    else
    {
      //NS_LOG_DEBUG_VERBOSE ("rx not-for-me from %d", GetSource (packet));
    }
    return;
rxPacket:
    WifiMacTrailer fcs;
    packet->RemoveTrailer (fcs);
    m_rxCallback (packet, &hdr);
    return;
  }

  uint32_t MacLow::GetAckSize (void) const
  {
    WifiMacHeader ack;
    ack.SetType (WIFI_MAC_CTL_ACK);
    return ack.GetSize () + 4;
  }
  uint32_t MacLow::GetBlockAckSize (enum BlockAckType type) const
  {
    WifiMacHeader hdr;
    hdr.SetType (WIFI_MAC_CTL_BACKRESP);
    CtrlBAckResponseHeader blockAck;
    if (type == BASIC_BLOCK_ACK)
    {
      blockAck.SetType (BASIC_BLOCK_ACK);
    }
    else if (type == COMPRESSED_BLOCK_ACK)
    {
      blockAck.SetType (COMPRESSED_BLOCK_ACK);
    }
    else if (type == MULTI_TID_BLOCK_ACK)
    {
      //Not implemented
      NS_ASSERT (false);
    }
    return hdr.GetSize () + blockAck.GetSerializedSize () + 4;
  }
  uint32_t MacLow::GetRtsSize (void) const
  {
    WifiMacHeader rts;
    rts.SetType (WIFI_MAC_CTL_RTS);
    return rts.GetSize () + 4;
  }
  Time MacLow::GetAckDuration (Mac48Address to, WifiMode dataTxMode) const
  {
    WifiMode ackMode = GetAckTxModeForData (to, dataTxMode);
    return m_phy->CalculateTxDuration (GetAckSize (), ackMode, WIFI_PREAMBLE_LONG);
  }
  Time MacLow::GetBlockAckDuration (Mac48Address to, WifiMode blockAckReqTxMode, enum BlockAckType type) const
  {
    /*
     * For immediate BlockAck we should transmit the frame with the same WifiMode
     * as the BlockAckReq.
     *
     * from section 9.6 in IEEE802.11e:
     * The BlockAck control frame shall be sent at the same rate and modulation class as
     * the BlockAckReq frame if it is sent in response to a BlockAckReq frame.
     */
    return m_phy->CalculateTxDuration (GetBlockAckSize (type), blockAckReqTxMode, WIFI_PREAMBLE_LONG);
  }
  Time MacLow::GetCtsDuration (Mac48Address to, WifiMode rtsTxMode) const
  {
    WifiMode ctsMode = GetCtsTxModeForRts (to, rtsTxMode);
    return m_phy->CalculateTxDuration (GetCtsSize (), ctsMode, WIFI_PREAMBLE_LONG);
  }
  uint32_t MacLow::GetCtsSize (void) const
  {
    WifiMacHeader cts;
    cts.SetType (WIFI_MAC_CTL_CTS);
    return cts.GetSize () + 4;
  }
  uint32_t MacLow::GetSize (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
  {
    WifiMacTrailer fcs;
    return packet->GetSize () + hdr->GetSize () + fcs.GetSerializedSize ();
  }

  WifiMode MacLow::GetRtsTxMode (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
  {
    Mac48Address to = hdr->GetAddr1 ();
    return m_stationManager->GetRtsMode (to, hdr, packet);
  }
  WifiMode MacLow::GetDataTxMode (Ptr<const Packet> packet, const WifiMacHeader *hdr) const
  {
    Mac48Address to = hdr->GetAddr1 ();
    WifiMacTrailer fcs;
    uint32_t size =  packet->GetSize () + hdr->GetSize () + fcs.GetSerializedSize ();
    return m_stationManager->GetDataMode (to, hdr, packet, size);
  }

  WifiMode MacLow::GetCtsTxModeForRts (Mac48Address to, WifiMode rtsTxMode) const
  {
    return m_stationManager->GetCtsMode (to, rtsTxMode);
  }
  WifiMode MacLow::GetAckTxModeForData (Mac48Address to, WifiMode dataTxMode) const
  {
    return m_stationManager->GetAckMode (to, dataTxMode);
  }


  Time MacLow::CalculateOverallTxTime (Ptr<const Packet> packet, const WifiMacHeader* hdr, const MacLowTransmissionParameters& params) const
  {
    Time txTime = Seconds (0);
    WifiMode rtsMode = GetRtsTxMode (packet, hdr);
    WifiMode dataMode = GetDataTxMode (packet, hdr);
    if (params.MustSendRts ())
    {
      txTime += m_phy->CalculateTxDuration (GetRtsSize (), rtsMode, WIFI_PREAMBLE_LONG);
      txTime += GetCtsDuration (hdr->GetAddr1 (), rtsMode);
      txTime += Time (GetSifs () * 2);
    }
    uint32_t dataSize = GetSize (packet, hdr);
    txTime += m_phy->CalculateTxDuration (dataSize, dataMode, WIFI_PREAMBLE_LONG);
    if (params.MustWaitAck ())
    {
      txTime += GetSifs ();
      txTime += GetAckDuration (hdr->GetAddr1 (), dataMode);
    }
    return txTime;
  }

  Time MacLow::CalculateTransmissionTime (Ptr<const Packet> packet, const WifiMacHeader* hdr, const MacLowTransmissionParameters& params) const
  {
    Time txTime = CalculateOverallTxTime (packet, hdr, params);
    if (params.HasNextPacket ())
    {
      WifiMode dataMode = GetDataTxMode (packet, hdr);
      txTime += GetSifs ();
      txTime += m_phy->CalculateTxDuration (params.GetNextPacketSize (), dataMode, WIFI_PREAMBLE_LONG);
    }
    return txTime;
  }

  void MacLow::NotifyNav (const WifiMacHeader &hdr, WifiMode txMode, WifiPreamble preamble)
  {
    NS_ASSERT (m_lastNavStart <= Simulator::Now ());
    Time duration = hdr.GetDuration ();

    if (hdr.IsCfpoll ()
        && hdr.GetAddr2 () == m_bssid)
    {
      // see section 9.3.2.2 802.11-1999
      DoNavResetNow (duration);
      return;
    }
    // XXX Note that we should also handle CF_END specially here
    // but we don't for now because we do not generate them.
    else if (hdr.GetAddr1 () != m_self)
    {
      // see section 9.2.5.4 802.11-1999
      bool navUpdated = DoNavStartNow (duration);
      if (hdr.IsRts () && navUpdated)
      {
        /**
         * A STA that used information from an RTS frame as the most recent basis to update its NAV setting
         * is permitted to reset its NAV if no PHY-RXSTART.indication is detected from the PHY during a
         * period with a duration of (2 * aSIFSTime) + (CTS_Time) + (2 * aSlotTime) starting at the
         * PHY-RXEND.indication corresponding to the detection of the RTS frame. The “CTS_Time” shall
         * be calculated using the length of the CTS frame and the data rate at which the RTS frame
         * used for the most recent NAV update was received.
         */
        WifiMacHeader cts;
        cts.SetType (WIFI_MAC_CTL_CTS);
        Time navCounterResetCtsMissedDelay =
          m_phy->CalculateTxDuration (cts.GetSerializedSize (), txMode, preamble) +
          Time (2 * GetSifs ()) + Time (2 * GetSlotTime ());
        m_navCounterResetCtsMissed = Simulator::Schedule (navCounterResetCtsMissedDelay, &MacLow::NavCounterResetCtsMissed, this, Simulator::Now ());
      }
    }
  }

  void MacLow::NavCounterResetCtsMissed (Time rtsEndRxTime)
  {
    if (m_phy->GetLastRxStartTime () > rtsEndRxTime)
    {
      DoNavResetNow (Seconds (0.0));
    }
  }

  void MacLow::DoNavResetNow (Time duration)
  {
    for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->NavReset (duration);
    }
    m_lastNavStart = Simulator::Now ();
    m_lastNavStart = duration;
  }
  bool MacLow::DoNavStartNow (Time duration)
  {
    for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->NavStart (duration);
    }
    Time newNavEnd = Simulator::Now () + duration;
    Time oldNavEnd = m_lastNavStart + m_lastNavDuration;
    if (newNavEnd > oldNavEnd)
    {
      m_lastNavStart = Simulator::Now ();
      m_lastNavDuration = duration;
      return true;
    }
    return false;
  }
  void MacLow::NotifyAckTimeoutStartNow (Time duration)
  {
    for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->AckTimeoutStart (duration);
    }
  }
  void MacLow::NotifyAckTimeoutResetNow ()
  {
    for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->AckTimeoutReset ();
    }
  }
  void MacLow::NotifyCtsTimeoutStartNow (Time duration)
  {
    for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->CtsTimeoutStart (duration);
    }
  }
  void MacLow::NotifyCtsTimeoutResetNow ()
  {
    for (DcfListenersCI i = m_dcfListeners.begin (); i != m_dcfListeners.end (); i++)
    {
      (*i)->CtsTimeoutReset ();
    }
  }

  void MacLow::ForwardDown (Ptr<const Packet> packet, const WifiMacHeader* hdr, WifiMode txMode)
  {
    NS_LOG_FUNCTION (this << packet << hdr << txMode);
    NS_LOG_DEBUG ("send " << hdr->GetTypeString () <<
        ", to=" << hdr->GetAddr1 () <<
        ", size=" << packet->GetSize () <<
        ", mode=" << txMode <<
        ", duration=" << hdr->GetDuration () <<
        ", seq=0x" << std::hex << m_currentHdr.GetSequenceControl () << std::dec);
    if ( m_initialErEdgeInterferenceW != 0) // has been set according to calculation in the PHY layer
    {
      InterferenceTag interferenceTag;
      interferenceTag.Set (m_initialErEdgeInterferenceW);
      packet->AddByteTag (interferenceTag);
    }
    if (Simulator::Now () <= Simulator::LearningTimeDuration || m_phy->GetObject<WifiImacPhy> ()->GetChannelNumber () == DATA_CHANNEL)
      //Please notice that when the condition (Simulator::Now () <= Simulator::LearningTimeDuration) is satisfied, we are using the  maximum transmission power. 
    {
      m_phy->SendPacket (packet, txMode, WIFI_PREAMBLE_LONG, (uint8_t)0); 
    }
    else if (Simulator::Now () > Simulator::LearningTimeDuration && m_phy->GetObject<WifiImacPhy> ()->GetChannelNumber () == CONTROL_CHANNEL)
    {
#ifdef POWER_CONTROL
      double txPower = NORMAL_TX_POWER;
      double maxNI = 0;
      Ptr<WifiImacPhy> imacPhy = m_phy->GetObject<WifiImacPhy> ();
      std::vector<std::string> nodesInEr = Simulator::ListNodesInEr (m_self.ToString (), m_informingRange);
      for (std::vector<std::string>::iterator it = nodesInEr.begin (); it != nodesInEr.end (); ++ it)
      {
        //std::vector<TdmaLink> relatedLinks = Simulator::FindRelatedLinks (it->c_str ());
        std::vector<TdmaLink> relatedLinks = Simulator::m_nodeLinkDetails[ Mac48Address (it->c_str ()).GetNodeId ()].relatedLinks;
        std::vector<std::string> nodesNeedToCheck; 
        for (std::vector<TdmaLink>::iterator _it = relatedLinks.begin (); _it != relatedLinks.end (); ++ _it)
        {
          if (_it->senderAddr == *it)
          {
            nodesNeedToCheck.push_back (_it->receiverAddr);
          }
          else if (_it->receiverAddr == *it)
          {
            nodesNeedToCheck.push_back (_it->senderAddr);
          }
        }
        nodesNeedToCheck.push_back (it->c_str ());

        for (std::vector<std::string>::iterator _it = nodesNeedToCheck.begin (); _it != nodesNeedToCheck.end (); ++ _it)
        {
          Ptr<SignalMap> signalMapItem = imacPhy->GetSignalMapItem (_it->c_str ());
          if (signalMapItem == 0)
          {
            continue;
          }
          double nPlusIDbm = 0;
          nPlusIDbm = imacPhy->WToDbm (signalMapItem->noisePlusInterferenceW);
          double tempTxPower = DELTA_SNR_MARGIN + SNR_WITH_100_PERCENT_PDR - imacPhy->GetTxGain () + signalMapItem->outBoundAttenuation + nPlusIDbm;
          if ( signalMapItem->noisePlusInterferenceW - maxNI > 0)
          {
            maxNI = signalMapItem->noisePlusInterferenceW;
          }
          if (tempTxPower > txPower)
          {
            txPower = tempTxPower;
          }
        }
      }
      if ( m_previousSendingPower > txPower && m_maxBiDirectionalErChangeInformTimes > 0)
      {
        txPower = m_previousSendingPower;
        m_maxBiDirectionalErChangeInformTimes --;
      }
      m_previousSendingPower = txPower;
      std::cout<<" transmission power for control signal: "<< txPower<<" maxNI: "<< maxNI << std::endl;
#endif
#ifdef MAX_POWER_LEVEL
      uint8_t txPower = MAX_TX_POWER_LEVEL;
#endif
      m_phy->GetObject<WifiImacPhy> ()->SendPacket (packet, txMode, WIFI_PREAMBLE_LONG, txPower);
    }
  }

  void MacLow::CtsTimeout (void)
  {
    NS_LOG_FUNCTION (this);
    NS_LOG_DEBUG ("cts timeout");
    // XXX: should check that there was no rx start before now.
    // we should restart a new cts timeout now until the expected
    // end of rx if there was a rx start before now.
    m_stationManager->ReportRtsFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
    m_currentPacket = 0;
    MacLowTransmissionListener *listener = m_listener;
    m_listener = 0;
    listener->MissedCts ();
  }
  void MacLow::NormalAckTimeout (void)
  {
    NS_LOG_FUNCTION (this);
    NS_LOG_DEBUG ("normal ack timeout");
    // XXX: should check that there was no rx start before now.
    // we should restart a new ack timeout now until the expected
    // end of rx if there was a rx start before now.
    m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
    MacLowTransmissionListener *listener = m_listener;
    m_listener = 0;
    listener->MissedAck ();
  }
  void MacLow::FastAckTimeout (void)
  {
    NS_LOG_FUNCTION (this);
    m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
    MacLowTransmissionListener *listener = m_listener;
    m_listener = 0;
    if (m_phy->IsStateIdle ())
    {
      NS_LOG_DEBUG ("fast Ack idle missed");
      listener->MissedAck ();
    }
    else
    {
      NS_LOG_DEBUG ("fast Ack ok");
    }
  }
  void MacLow::BlockAckTimeout (void)
  {
    NS_LOG_FUNCTION (this);
    NS_LOG_DEBUG ("block ack timeout");

    m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
    MacLowTransmissionListener *listener = m_listener;
    m_listener = 0;
    listener->MissedBlockAck ();
  }
  void MacLow::SuperFastAckTimeout ()
  {
    NS_LOG_FUNCTION (this);
    m_stationManager->ReportDataFailed (m_currentHdr.GetAddr1 (), &m_currentHdr);
    MacLowTransmissionListener *listener = m_listener;
    m_listener = 0;
    if (m_phy->IsStateIdle ())
    {
      NS_LOG_DEBUG ("super fast Ack failed");
      listener->MissedAck ();
    }
    else
    {
      NS_LOG_DEBUG ("super fast Ack ok");
      listener->GotAck (0.0, WifiMode ());
    }
  }

  /*
   * THIS method won't be used in the TDMA version of iMAC
   */
  void MacLow::SendRtsForPacket (void)
  {
    NS_LOG_FUNCTION (this);
    /* send an RTS for this packet. */
    WifiMacHeader rts;
    rts.SetType (WIFI_MAC_CTL_RTS);
    rts.SetDsNotFrom ();
    rts.SetDsNotTo ();
    rts.SetNoRetry ();
    rts.SetNoMoreFragments ();
    rts.SetAddr1 (m_currentHdr.GetAddr1 ());
    rts.SetAddr2 (m_self);
    WifiMode rtsTxMode = GetRtsTxMode (m_currentPacket, &m_currentHdr);
    Time duration = Seconds (0);
    if (m_txParams.HasDurationId ())
    {
      duration += m_txParams.GetDurationId ();
    }
    else
    {
      WifiMode dataTxMode = GetDataTxMode (m_currentPacket, &m_currentHdr);
      duration += GetSifs ();
      duration += GetCtsDuration (m_currentHdr.GetAddr1 (), rtsTxMode);
      duration += GetSifs ();
      duration += m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr),
          dataTxMode, WIFI_PREAMBLE_LONG);
      duration += GetSifs ();
      duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxMode);
    }
    rts.SetDuration (duration);

    Time txDuration = m_phy->CalculateTxDuration (GetRtsSize (), rtsTxMode, WIFI_PREAMBLE_LONG);
    Time timerDelay = txDuration + GetCtsTimeout ();

    NS_ASSERT (m_ctsTimeoutEvent.IsExpired ());
    NotifyCtsTimeoutStartNow (timerDelay);
    m_ctsTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::CtsTimeout, this);

    Ptr<Packet> packet = Create<Packet> ();
    packet->AddHeader (rts);
    WifiMacTrailer fcs;
    packet->AddTrailer (fcs);

    ForwardDown (packet, &rts, rtsTxMode);
  }

  void MacLow::StartDataTxTimers (void)
  {
    WifiMode dataTxMode = GetDataTxMode (m_currentPacket, &m_currentHdr);
    Time txDuration = m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr), dataTxMode, WIFI_PREAMBLE_LONG);
    if (m_txParams.MustWaitNormalAck ())
    {
      Time timerDelay = txDuration + GetAckTimeout ();
      NS_ASSERT (m_normalAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_normalAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::NormalAckTimeout, this);
    }
    else if (m_txParams.MustWaitFastAck ())
    {
      Time timerDelay = txDuration + GetPifs ();
      NS_ASSERT (m_fastAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_fastAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::FastAckTimeout, this);
    }
    else if (m_txParams.MustWaitSuperFastAck ())
    {
      Time timerDelay = txDuration + GetPifs ();
      NS_ASSERT (m_superFastAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_superFastAckTimeoutEvent = Simulator::Schedule (timerDelay,
          &MacLow::SuperFastAckTimeout, this);
    }
    else if (m_txParams.MustWaitBasicBlockAck ())
    {
      Time timerDelay = txDuration + GetBasicBlockAckTimeout ();
      NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
      m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
    }
    else if (m_txParams.MustWaitCompressedBlockAck ())
    {
      Time timerDelay = txDuration + GetCompressedBlockAckTimeout ();
      NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
      m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
    }
    else if (m_txParams.HasNextPacket ())
    {
      Time delay = txDuration + GetSifs ();
      NS_ASSERT (m_waitSifsEvent.IsExpired ());
      m_waitSifsEvent = Simulator::Schedule (delay, &MacLow::WaitSifsAfterEndTx, this);
    }
    else
    {
      // since we do not expect any timer to be triggered.
      m_listener = 0;
    }
  }

  void MacLow::SendDataPacket (void)
  {
    NS_LOG_FUNCTION (this);
    /* send this packet directly. No RTS is needed. */
    StartDataTxTimers ();
    if ( !m_phy->IsStateIdle () )
    {
      return;
    }

    WifiMode dataTxMode = GetDataTxMode (m_currentPacket, &m_currentHdr);
    Time duration = Seconds (0.0);
    if (m_txParams.HasDurationId ())
    {
      duration += m_txParams.GetDurationId ();
    }
    else
    {
      if (m_txParams.MustWaitBasicBlockAck ())
      {
        duration += GetSifs ();
        duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxMode, BASIC_BLOCK_ACK);
      }
      else if (m_txParams.MustWaitCompressedBlockAck ())
      {
        duration += GetSifs ();
        duration += GetBlockAckDuration (m_currentHdr.GetAddr1 (), dataTxMode, COMPRESSED_BLOCK_ACK);
      }
      else if (m_txParams.MustWaitAck ())
      {
        duration += GetSifs ();
        duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxMode);
      }
      if (m_txParams.HasNextPacket ())
      {
        duration += GetSifs ();
        duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
            dataTxMode, WIFI_PREAMBLE_LONG);
        if (m_txParams.MustWaitAck ())
        {
          duration += GetSifs ();
          duration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxMode);
        }
      }
    }
    m_currentHdr.SetDuration (duration);
    // parameter: sender, receiver
    if (m_phy->GetChannelNumber () == DATA_CHANNEL ) // we only set sequence number when the node is sending data packet
    {
      LinkEstimatorItem estimatorItem = GetEstimatorTableItemByNeighborAddr (m_currentHdr.GetAddr2 (), m_currentHdr.GetAddr1 () );
      m_currentHdr.SetSequenceNumber (estimatorItem.DataSequenceNo);
      estimatorItem.DataSequenceNo ++;
      UpdateEstimatorTableItem (estimatorItem);
    }

    m_currentPacket->AddHeader (m_currentHdr);
    WifiMacTrailer fcs;
    m_currentPacket->AddTrailer (fcs);
    if (Simulator::Now () >= Simulator::LearningTimeDuration && m_phy->GetChannelNumber () == DATA_CHANNEL) // when node is in data channel, type this message
    {
      std::cout<<"1: " <<Simulator::Now ()<<" sending data packet from: "<<m_self<<" to: "<<m_currentHdr.GetAddr1 () <<" seq: "<<m_currentHdr.GetSequenceNumber () <<"  with outSnr: "<< m_phy->GetObject<WifiImacPhy> ()->GetOutBoundSinrForDest (m_currentHdr.GetAddr1 ()) <<" nodeid: "<<m_self.GetNodeId () <<std::endl;
    }
    ForwardDown (m_currentPacket, &m_currentHdr, dataTxMode);
    m_currentPacket = 0;
  }

  bool MacLow::IsNavZero (void) const
  {
    if (m_lastNavStart + m_lastNavDuration < Simulator::Now ())
    {
      return true;
    }
    else
    {
      return false;
    }
  }


  /*
   * THIS method won't be used in the version of iMAC
   */
  void MacLow::SendCtsAfterRts (Mac48Address source, Time duration, WifiMode rtsTxMode, double rtsSnr)
  {
    NS_LOG_FUNCTION (this << source << duration << rtsTxMode << rtsSnr);
    /* send a CTS when you receive a RTS
     * right after SIFS.
     */
    WifiMode ctsTxMode = GetCtsTxModeForRts (source, rtsTxMode);
    WifiMacHeader cts;
    cts.SetType (WIFI_MAC_CTL_CTS);
    cts.SetDsNotFrom ();
    cts.SetDsNotTo ();
    cts.SetNoMoreFragments ();
    cts.SetNoRetry ();
    cts.SetAddr1 (source);
    duration -= GetCtsDuration (source, rtsTxMode);
    duration -= GetSifs ();
    NS_ASSERT (duration >= MicroSeconds (0));
    cts.SetDuration (duration);

    Ptr<Packet> packet = Create<Packet> ();
    packet->AddHeader (cts);
    WifiMacTrailer fcs;
    packet->AddTrailer (fcs);

    SnrTag tag;
    tag.Set (rtsSnr);
    packet->AddPacketTag (tag);

    ForwardDown (packet, &cts, ctsTxMode);
  }

  /*
   * THIS method won't be used in the TDMA version of iMAC, since we are going to use @SendDataPacket instead;
   */
  void MacLow::SendDataAfterCts (Mac48Address source, Time duration, WifiMode txMode)
  {
    NS_LOG_FUNCTION (this);
    /* send the third step in a
     * RTS/CTS/DATA/ACK hanshake
     */
    NS_ASSERT (m_currentPacket != 0);
    StartDataTxTimers ();

    WifiMode dataTxMode = GetDataTxMode (m_currentPacket, &m_currentHdr);
    Time newDuration = Seconds (0);
    newDuration += GetSifs ();
    newDuration += GetAckDuration (m_currentHdr.GetAddr1 (), dataTxMode);
    Time txDuration = m_phy->CalculateTxDuration (GetSize (m_currentPacket, &m_currentHdr),
        dataTxMode, WIFI_PREAMBLE_LONG);
    duration -= txDuration;
    duration -= GetSifs ();

    duration = std::max (duration, newDuration);
    NS_ASSERT (duration >= MicroSeconds (0));
    m_currentHdr.SetDuration (duration);

    m_currentPacket->AddHeader (m_currentHdr);
    WifiMacTrailer fcs;
    m_currentPacket->AddTrailer (fcs);

    ForwardDown (m_currentPacket, &m_currentHdr, dataTxMode);
    m_currentPacket = 0;
  }

  void MacLow::WaitSifsAfterEndTx (void)
  {
    m_listener->StartNext ();
  }

  void MacLow::FastAckFailedTimeout (void)
  {
    NS_LOG_FUNCTION (this);
    MacLowTransmissionListener *listener = m_listener;
    m_listener = 0;
    listener->MissedAck ();
    NS_LOG_DEBUG ("fast Ack busy but missed");
  }

  void MacLow::SendAckAfterData (Mac48Address source, Time duration, WifiMode dataTxMode, double dataSnr)
  {
    NS_LOG_FUNCTION (this);
    /* send an ACK when you receive
     * a packet after SIFS.
     */
    WifiMode ackTxMode = GetAckTxModeForData (source, dataTxMode);
    WifiMacHeader ack;
    ack.SetType (WIFI_MAC_CTL_ACK);
    ack.SetDsNotFrom ();
    ack.SetDsNotTo ();
    ack.SetNoRetry ();
    ack.SetNoMoreFragments ();
    ack.SetAddr1 (source);
    // for ack, @source is the receiver, but for data, @source is the sender, so @source is the first parameter
    LinkEstimatorItem estimatorItem = GetEstimatorTableItemByNeighborAddr (source, m_self);
    AckSequenceNoTag ackSequenceNoTag;
    ackSequenceNoTag.Set (estimatorItem.AckSequenceNo);
    estimatorItem.AckSequenceNo ++;
    UpdateEstimatorTableItem (estimatorItem);
    duration -= GetAckDuration (source, dataTxMode);
    duration -= GetSifs ();
    NS_ASSERT (duration >= MicroSeconds (0));
    ack.SetDuration (duration);

    Ptr<Packet> packet = Create<Packet> ();
    packet->AddHeader (ack);
    WifiMacTrailer fcs;
    packet->AddTrailer (fcs);

    InterferenceTag interferenceTag;
    interferenceTag.Set (estimatorItem.LastDataErEdgeInterferenceW);
    packet->AddByteTag (interferenceTag);

    SnrTag tag;
    tag.Set (dataSnr);
    packet->AddPacketTag (tag);

    ReceiverAddressTag receiverAddress;
    receiverAddress.Set (m_self);
    packet->AddByteTag (receiverAddress);
    packet->AddByteTag (ackSequenceNoTag);

    if (Simulator::Now () >= Simulator::LearningTimeDuration)
    {
      std::cout<<"3: "<<Simulator::Now ()<<" sending ack back to: "<<source<< " from: "<<m_self<<std::endl;
      m_sendingCount ++;
    }
    ForwardDown (packet, &ack, ackTxMode);
  }

  bool MacLow::StoreMpduIfNeeded (Ptr<Packet> packet, WifiMacHeader hdr)
  {
    AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
    if (it != m_bAckAgreements.end ())
    {
      WifiMacTrailer fcs;
      packet->RemoveTrailer (fcs);
      BufferedPacket bufferedPacket (packet, hdr);

      uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
      uint16_t mappedSeqControl = QosUtilsMapSeqControlToUniqueInteger (hdr.GetSequenceControl (), endSequence);

      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end ()
          && QosUtilsMapSeqControlToUniqueInteger ((*i).second.GetSequenceControl (), endSequence) < mappedSeqControl; i++)
      {
        ;
      }
      (*it).second.second.insert (i, bufferedPacket);

      //Update block ack cache
      BlockAckCachesI j = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
      NS_ASSERT (j != m_bAckCaches.end ());
      (*j).second.UpdateWithMpdu (&hdr);

      return true;
    }
    return false;
  }

  void MacLow::CreateBlockAckAgreement (const MgtAddBaResponseHeader *respHdr, Mac48Address originator,
      uint16_t startingSeq)
  {
    uint8_t tid = respHdr->GetTid ();
    BlockAckAgreement agreement (originator, tid);
    if (respHdr->IsImmediateBlockAck ())
    {
      agreement.SetImmediateBlockAck ();
    }
    else
    {
      agreement.SetDelayedBlockAck ();
    }
    agreement.SetAmsduSupport (respHdr->IsAmsduSupported ());
    agreement.SetBufferSize (respHdr->GetBufferSize () + 1);
    agreement.SetTimeout (respHdr->GetTimeout ());
    agreement.SetStartingSequence (startingSeq);

    std::list<BufferedPacket> buffer (0);
    AgreementKey key (originator, respHdr->GetTid ());
    AgreementValue value (agreement, buffer);
    m_bAckAgreements.insert (std::make_pair (key, value));

    BlockAckCache cache;
    cache.Init (startingSeq, respHdr->GetBufferSize () + 1);
    m_bAckCaches.insert (std::make_pair (key, cache));

    if (respHdr->GetTimeout () != 0)
    {
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, respHdr->GetTid ()));
      Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

      AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());

      it->second.first.m_inactivityEvent = Simulator::Schedule (timeout,
          &MacLowBlockAckEventListener::BlockAckInactivityTimeout,
          m_edcaListeners[ac],
          originator, tid);
    }
  }

  void MacLow::DestroyBlockAckAgreement (Mac48Address originator, uint8_t tid)
  {
    AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
    if (it != m_bAckAgreements.end ())
    {
      RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequence (), originator, tid);
      RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
      m_bAckAgreements.erase (it);

      BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
      NS_ASSERT (i != m_bAckCaches.end ());
      m_bAckCaches.erase (i);
    }
  }

  void MacLow::RxCompleteBufferedPacketsWithSmallerSequence (uint16_t seq, Mac48Address originator, uint8_t tid)
  {
    AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
    if (it != m_bAckAgreements.end ())
    {
      uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
      uint16_t mappedStart = QosUtilsMapSeqControlToUniqueInteger (seq, endSequence);
      uint16_t guard = (*it).second.second.begin ()->second.GetSequenceControl () & 0xfff0;
      BufferedPacketI last = (*it).second.second.begin ();

      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end ()
          && QosUtilsMapSeqControlToUniqueInteger ((*i).second.GetSequenceNumber (), endSequence) < mappedStart;)
      {
        if (guard == (*i).second.GetSequenceControl ())
        {
          if (!(*i).second.IsMoreFragments ())
          {
            while (last != i)
            {
              m_rxCallback ((*last).first, &(*last).second);
              last++;
            }
            m_rxCallback ((*last).first, &(*last).second);
            last++;
            /* go to next packet */
            while (i != (*it).second.second.end () && ((guard >> 4) & 0x0fff) == (*i).second.GetSequenceNumber ())
            {
              i++;
            }
            if (i != (*it).second.second.end ())
            {
              guard = (*i).second.GetSequenceControl () & 0xfff0;
              last = i;
            }
          }
          else
          {
            guard++;
          }
        }
        else
        {
          /* go to next packet */
          while (i != (*it).second.second.end () && ((guard >> 4) & 0x0fff) == (*i).second.GetSequenceNumber ())
          {
            i++;
          }
          if (i != (*it).second.second.end ())
          {
            guard = (*i).second.GetSequenceControl () & 0xfff0;
            last = i;
          }
        }
      }
      (*it).second.second.erase ((*it).second.second.begin (), i);
    }
  }

  void MacLow::RxCompleteBufferedPacketsUntilFirstLost (Mac48Address originator, uint8_t tid)
  {
    AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
    if (it != m_bAckAgreements.end ())
    {
      uint16_t startingSeqCtrl = ((*it).second.first.GetStartingSequence () << 4) & 0xfff0;
      uint16_t guard = startingSeqCtrl;

      BufferedPacketI lastComplete = (*it).second.second.begin ();
      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end () && guard == (*i).second.GetSequenceControl (); i++)
      {
        if (!(*i).second.IsMoreFragments ())
        {
          while (lastComplete != i)
          {
            m_rxCallback ((*lastComplete).first, &(*lastComplete).second);
            lastComplete++;
          }
          m_rxCallback ((*lastComplete).first, &(*lastComplete).second);
          lastComplete++;
        }
        guard = (*i).second.IsMoreFragments () ? (guard + 1) : ((guard + 16) & 0xfff0);
      }
      (*it).second.first.SetStartingSequence ((guard >> 4) & 0x0fff);
      /* All packets already forwarded to WifiMac must be removed from buffer:
         [begin (), lastComplete) */
      (*it).second.second.erase ((*it).second.second.begin (), lastComplete);
    }
  }

  void MacLow::SendBlockAckResponse (const CtrlBAckResponseHeader* blockAck, Mac48Address originator, bool immediate,
      Time duration, WifiMode blockAckReqTxMode)
  {
    Ptr<Packet> packet = Create<Packet> ();
    packet->AddHeader (*blockAck);

    WifiMacHeader hdr;
    hdr.SetType (WIFI_MAC_CTL_BACKRESP);
    hdr.SetAddr1 (originator);
    hdr.SetAddr2 (GetAddress ());
    hdr.SetDsNotFrom ();
    hdr.SetDsNotTo ();
    hdr.SetNoRetry ();
    hdr.SetNoMoreFragments ();

    m_currentPacket = packet;
    m_currentHdr = hdr;
    if (immediate)
    {
      m_txParams.DisableAck ();
      duration -= GetSifs ();
      if (blockAck->IsBasic ())
      {
        duration -= GetBlockAckDuration (originator, blockAckReqTxMode, BASIC_BLOCK_ACK);
      }
      else if (blockAck->IsCompressed ())
      {
        duration -= GetBlockAckDuration (originator, blockAckReqTxMode, COMPRESSED_BLOCK_ACK);
      }
      else if (blockAck->IsMultiTid ())
      {
        NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
      }
    }
    else
    {
      m_txParams.EnableAck ();
      duration += GetSifs ();
      duration += GetAckDuration (originator, blockAckReqTxMode);
    }
    m_txParams.DisableNextData ();

    StartDataTxTimers ();

    NS_ASSERT (duration >= MicroSeconds (0));
    hdr.SetDuration (duration);
    //here should be present a control about immediate or delayed block ack
    //for now we assume immediate
    packet->AddHeader (hdr);
    WifiMacTrailer fcs;
    packet->AddTrailer (fcs);
    ForwardDown (packet, &hdr, blockAckReqTxMode);
    m_currentPacket = 0;
  }

  void MacLow::SendBlockAckAfterBlockAckRequest (const CtrlBAckRequestHeader reqHdr, Mac48Address originator,
      Time duration, WifiMode blockAckReqTxMode)
  {
    NS_LOG_FUNCTION (this);
    CtrlBAckResponseHeader blockAck;
    uint8_t tid;
    bool immediate = false;
    if (!reqHdr.IsMultiTid ())
    {
      tid = reqHdr.GetTidInfo ();
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
      if (it != m_bAckAgreements.end ())
      {
        blockAck.SetStartingSequence (reqHdr.GetStartingSequence ());
        blockAck.SetTidInfo (tid);
        immediate = (*it).second.first.IsImmediateBlockAck ();
        if (reqHdr.IsBasic ())
        {
          blockAck.SetType (BASIC_BLOCK_ACK);
        }
        else if (reqHdr.IsCompressed ())
        {
          blockAck.SetType (COMPRESSED_BLOCK_ACK);
        }
        BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
        NS_ASSERT (i != m_bAckCaches.end ());
        (*i).second.FillBlockAckBitmap (&blockAck);

        /* All packets with smaller sequence than starting sequence control must be passed up to Wifimac
         * See 9.10.3 in IEEE8022.11e standard.
         */
        RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequence (), originator, tid);
        RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
      }
      else
      {
        NS_LOG_DEBUG ("there's not a valid block ack agreement with " << originator);
      }
    } else
    {
      NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
    }

    SendBlockAckResponse (&blockAck, originator, immediate, duration, blockAckReqTxMode);
  }

  void MacLow::ResetBlockAckInactivityTimerIfNeeded (BlockAckAgreement &agreement)
  {
    if (agreement.GetTimeout () != 0)
    {
      NS_ASSERT (agreement.m_inactivityEvent.IsRunning ());
      agreement.m_inactivityEvent.Cancel ();
      Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

      AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());
      //std::map<AcIndex, MacLowTransmissionListener*>::iterator it = m_edcaListeners.find (ac);
      //NS_ASSERT (it != m_edcaListeners.end ());

      agreement.m_inactivityEvent = Simulator::Schedule (timeout,
          &MacLowBlockAckEventListener::BlockAckInactivityTimeout,
          m_edcaListeners[ac],
          agreement.GetPeer (),
          agreement.GetTid ());
    }
  }

  void MacLow::RegisterBlockAckListenerForAc (enum AcIndex ac, MacLowBlockAckEventListener *listener)
  {
    m_edcaListeners.insert (std::make_pair (ac, listener));
  }

  ErInfoItem MacLow::GenerateDefaultErInfoItem (uint16_t sender, uint16_t receiver, bool isDataEr)
  {
    ErInfoItem item;
    item.sender = sender;
    item.receiver = receiver;
    item.edgeInterferenceW = IMPOSSIBLE_ER;
    item.updateSeqNo = 0;
    item.itemPriority = 0;
    item.itemId = (sender * Simulator::NodesCountUpperBound + receiver) * 10;
    if ( isDataEr == true )
    {
      item.itemId += 1;
    }
    else if (isDataEr == false)
    {
      item.itemId += 0;
    }
    return item;
  }

  /* If the ER info item expired, we return IMPOSSIBLE_ER, which makes the ER size be zero. 
   * For d_0 implementation, if a new ER item has been received for some time smaller than d_0, we still use the previous ER info item,
   * In this way, we are trying to avoid channel inconsistency
   * NO D_0 CONCEPT ANY MORE
   */
  std::vector<double> MacLow::GetErInforItemForLink (Mac48Address sender, Mac48Address receiver, Mac48Address targetSender, Mac48Address targetReceiver)
  {
    bool dataFound = false;    
    std::vector<double> retVector; // the first element is the data er, the second element is the ack er
    // er info of the current node its own
    for (std::vector<ErInfoItem>::iterator it = m_controlInformation.begin (); it != m_controlInformation.end (); ++ it)
    {
      if (it->sender == sender.GetNodeId () && it->receiver == receiver.GetNodeId ())
      {
        dataFound = true;
        retVector.insert ( retVector.begin (), it->edgeInterferenceW);
        return retVector;
      }
    }


    ErInfoItem dataErItem = m_othersControlInformationCopy[sender.GetNodeId () * 10 + 1];
    if ( dataErItem.sender != 0)
    {
      dataFound = true;
      retVector.insert ( retVector.begin (), (double)dataErItem.edgeInterferenceW);
      return retVector;
    }


    if (dataFound == false)
    {
      retVector.insert (retVector.begin (), IMPOSSIBLE_ER); //IMPOSSIBLE_ER makes ER size to be zero
    }

    NS_ASSERT (retVector.size () == 1);
    return retVector;
  }


  LinkEstimatorItem MacLow::GenerateLinkEstimatorItem (Mac48Address sender, Mac48Address receiver )
  {
    LinkEstimatorItem newItem;
    newItem.DataSequenceNo = 0;
    newItem.AckSequenceNo = 0;
    newItem.PreviousDataPdr = -1;
    newItem.PreviousAckPdr = -1;
    newItem.DataPdr = -1;
    newItem.AckPdr = -1;
    newItem.LastDataSequenceNo = 0;
    newItem.LastAckSequenceNo = 0;
    newItem.Sender = sender;
    newItem.Receiver = receiver;
    newItem.previousDataErW = IMPOSSIBLE_ER;
    newItem.previousAckErW = IMPOSSIBLE_ER;
    newItem.LastDataErEdgeInterferenceW = IMPOSSIBLE_ER; //Watt
    newItem.LastAckErEdgeInterferenceW = IMPOSSIBLE_ER; // the initial value
    newItem.DataInterferenceW = 0;
    newItem.AckInterferenceW = 0;
    return newItem;
  }

  /* Although the current node also keeps data for other links
   * , but the current node only updates the sequence number for the links that are related to the current node
   * When as a receiver, update the data packet sequence number and data ER edge interference
   * when as a sender, update the ack packet sequence number and ack ER edge interference
   */
  LinkEstimatorItem MacLow::GetEstimatorTableItemByNeighborAddr (Mac48Address sender, Mac48Address receiver )
  {
    std::vector<LinkEstimatorItem>::iterator it = m_linkEstimatorTable.begin ();
    for ( ; it != m_linkEstimatorTable.end () ; ++ it)
    {
      if ( it->Sender == sender && it->Receiver == receiver )
      {
        return (*it);
      }
    }
    // if the program reaches here, that means there is no record for the addr. Then, create a new one for this addr
    LinkEstimatorItem newItem = GenerateLinkEstimatorItem (sender, receiver);
    m_linkEstimatorTable.insert ( m_linkEstimatorTable.begin (), newItem);
    return newItem;
  }

  void MacLow::UpdateEstimatorTableItem (LinkEstimatorItem estimatorItem)
  {
    std::vector<LinkEstimatorItem>::iterator it = m_linkEstimatorTable.begin ();
    for ( ; it != m_linkEstimatorTable.end () ; ++ it)
    {
      if ( it->Sender == estimatorItem.Sender && it->Receiver == estimatorItem.Receiver )
      {
        it->DataSequenceNo = estimatorItem.DataSequenceNo;
        it->AckSequenceNo = estimatorItem.AckSequenceNo;
        it->DataPdr = estimatorItem.DataPdr;
        it->AckPdr = estimatorItem.AckPdr;
        it->previousDataErW = estimatorItem.previousDataErW;
        it->previousAckErW = estimatorItem.previousAckErW;
        it->LastDataErEdgeInterferenceW = estimatorItem.LastDataErEdgeInterferenceW;
        it->LastAckErEdgeInterferenceW = estimatorItem.LastAckErEdgeInterferenceW;
        it->DataInterferenceW = estimatorItem.DataInterferenceW;
        it->AckInterferenceW = estimatorItem.AckInterferenceW;
        it->ReceivedDataPacketNumbers = estimatorItem.ReceivedDataPacketNumbers;
        it->ReceivedAckPacketNumbers = estimatorItem.ReceivedAckPacketNumbers;
        break;
      }
    }
  }

  void MacLow::UpdateReceivedDataPacketNumbers (Mac48Address sender, Mac48Address receiver, uint16_t seqNo)
  {
    seqNo = seqNo % m_maxSeqNo;
    double estimatedPdr = m_desiredDataPdr;
    double deltaInterferenceDb = 0;
    std::vector<LinkEstimatorItem>::iterator it = m_linkEstimatorTable.begin ();
    for ( ; it != m_linkEstimatorTable.end () ; ++ it)
    {
      if ( it->Sender == sender && it->Receiver == receiver)
      {
        if ( seqNo == it->LastDataSequenceNo)
        {
          break;
        }
        if (find (it->ReceivedDataPacketNumbers.begin (), it->ReceivedDataPacketNumbers.end (), seqNo) == it->ReceivedDataPacketNumbers.end ())
        {
          it->ReceivedDataPacketNumbers.insert (it->ReceivedDataPacketNumbers.begin (), seqNo);
        }
        NS_ASSERT ( find (it->ReceivedDataPacketNumbers.begin (), it->ReceivedDataPacketNumbers.end (), seqNo) != it->ReceivedDataPacketNumbers.end ());
        // estimate link reliability (data pdr)
        double receivedCount = 0.0;
        int difference = 0;
        if ( it->LastDataSequenceNo > seqNo ) // overflow
        {
          difference = seqNo + m_maxSeqNo - it->LastDataSequenceNo;
        }
        else  // not overflow
        {
          difference = seqNo - it->LastDataSequenceNo;
        }
        if (difference >= m_estimatorWindow)
        {
          ErInfoItem payloadItem = GetErInfoItem (sender.GetNodeId (), receiver.GetNodeId (), true, true);
          receivedCount = it->ReceivedDataPacketNumbers.size ();
          if ( it->DataPdr >= 0)
          {
            it->PreviousDataPdr = it->DataPdr;
            it->DataPdr = (1 - m_ewmaCoefficient) * receivedCount / (double)difference + m_ewmaCoefficient * it->DataPdr;
          }
          else
          {
            it->PreviousDataPdr = it->DataPdr;
            it->DataPdr = receivedCount / (double)difference;
          }
          estimatedPdr = receivedCount / (double) difference;
          std::cout<<m_self<<" from: "<<sender<<" real segment pdr: "<< receivedCount / (double)difference << std::endl;
          // -----------------UPDATE SENDING PROBABILITY --------------------------
#ifdef TX_DURATION_PROBABILITY
          double slotDifference = m_currentTimeslot - m_sendProbLastComputedTimeSlot;
          m_selfSendingProbability = m_ewmaCoefficient * m_selfSendingProbability + (1 - m_ewmaCoefficient) * ((double)(++ m_sendingCount) / slotDifference); // we use (++ m_sendingCount) is because we need to consider the ack that will be sent out immediately.
          m_sendingCount = 0;
          m_sendProbLastComputedTimeSlot = m_currentTimeslot;
#endif
          if (it->DataPdr > 1)
          {
            it->DataPdr = 1;
          }
          /* data estimate the ER region*/
          Ptr<WifiImacPhy> tempPhy = m_phy->GetObject<WifiImacPhy> ();
          Ptr<SignalMap> signalMapItem = tempPhy->GetSignalMapItem (sender);
          it->DataInterferenceW = tempPhy->ComputeInterferenceWhenReceivingData ();
          //double supposedReceivePowerDbm = tempPhy->GetPowerDbmByLevel (0) + tempPhy->GetTxGain () - signalMapItem->outBoundAttenuation;
          tempPhy->ComputeSampledInterferenceW ();
          bool conditionTwoMeet = false;
          double expectedInterferenceDbm = 0;
          double expectedInterferenceWatt = 0;
          double deltaInterferenceWatt = 0;
          LinkMetaData linkMetaData = GetLinkMetaDataByLink (m_self, sender);

#ifdef MIN_VARIANCE_CONTROLLER
          if (linkMetaData.interferencePreviousDbm != 0 && linkMetaData.interferenceNowDbm != 0)
          {
            deltaInterferenceDb = m_minVarController.GetDeltaInterference (m_desiredDataPdr, it->DataPdr, estimatedPdr, conditionTwoMeet);
            std::cout<<m_self<<" from: "<<sender<<" desiredPdr: "<<m_desiredDataPdr << " ewmaCurrentPdr: "<<it->DataPdr << " estimatedCurrentPdr: "<< estimatedPdr <<" deltaInterferenceDb: "<<deltaInterferenceDb<< std::endl;
          }
          if ( linkMetaData.interferencePreviousDbm == 0 || linkMetaData.interferenceNowDbm == 0) // the first time we compute \Delta_I
          {
            linkMetaData.interferencePreviousDbm = linkMetaData.interferenceNowDbm;
            linkMetaData.interferenceNowDbm = tempPhy->WToDbm (it->DataInterferenceW + tempPhy->GetCurrentNoiseW ()); 
            linkMetaData.lastComputedDeltaInterferenceDb = deltaInterferenceDb;
          }
          else
          {

            linkMetaData.interferencePreviousDbm = linkMetaData.interferenceNowDbm;
            linkMetaData.interferenceNowDbm = tempPhy->WToDbm (it->DataInterferenceW + tempPhy->GetCurrentNoiseW ());

            double deltaIM = tempPhy->DbmToW (linkMetaData.interferenceNowDbm) - tempPhy->DbmToW (linkMetaData.interferencePreviousDbm);
            double previousDeltaIR = tempPhy->DbmToW (linkMetaData.interferencePreviousDbm + linkMetaData.lastComputedDeltaInterferenceDb) - tempPhy->DbmToW (linkMetaData.interferencePreviousDbm) - linkMetaData.muBWatt;
            double actualDeltaU = deltaIM - previousDeltaIR;


            std::cout<<m_self<<" from: "<< sender<<" previous muBWatt: "<< linkMetaData.muBWatt<< " delta_I_m: "<<deltaIM <<" computed_delta_I(dB): "<<deltaInterferenceDb<<" actual delta_U: "<<actualDeltaU <<" previousDeltaIR: "<<previousDeltaIR;
            //linkMetaData.muBWatt = (1 - m_ewmaCoefficient) * actualDeltaU  + m_ewmaCoefficient * linkMetaData.muBWatt;
            linkMetaData.muBWatt = 0; 
            std::cout<<" new muBWatt: "<< linkMetaData.muBWatt<<std::endl;
            linkMetaData.lastComputedDeltaInterferenceDb = deltaInterferenceDb;
          }
          expectedInterferenceDbm = tempPhy->WToDbm (it->DataInterferenceW + tempPhy->GetCurrentNoiseW ()) + deltaInterferenceDb;
          expectedInterferenceWatt = tempPhy->DbmToW (expectedInterferenceDbm);
          deltaInterferenceWatt = expectedInterferenceWatt - it->DataInterferenceW  - tempPhy->GetCurrentNoiseW ()- linkMetaData.muBWatt;
          std::cout<<m_self<<" from: "<<sender<< " expected interference "<< expectedInterferenceWatt<< " newDelta_I_R: "<<deltaInterferenceWatt<<" deltaInterferenceW before muBWatt: "<<expectedInterferenceWatt - it->DataInterferenceW  - tempPhy->GetCurrentNoiseW ()<< " interferenceW: "<<it->DataInterferenceW  + tempPhy->GetCurrentNoiseW ()<<" erSize: "<<tempPhy->GetErSize (it->LastDataErEdgeInterferenceW)<< " concurrentTxNO: "<< tempPhy->GetConcurrentTxNo ()<<" pure_interferenceW: "<<it->DataInterferenceW << std::endl; 
          if (deltaInterferenceDb != 0 && deltaInterferenceWatt == 0)
          {
            std::cout<<" abnormal: delta_interference_db: "<< deltaInterferenceDb <<" expected_interference_watt: "<<expectedInterferenceWatt << " currentInterference: "<<it->DataInterferenceW + tempPhy->GetCurrentNoiseW () <<" delta_interference_watt: "<< (expectedInterferenceWatt - it->DataInterferenceW - tempPhy->GetCurrentNoiseW ()) <<std::endl;
          }
#endif
#ifdef P_CONTROLLER_DESIRED_PDR

          deltaInterferenceDb = m_pControllerWithDesiredPdr.GetDeltaInterference(m_desiredDataPdr, estimatedPdr);//in dB
          expectedInterferenceDbm = tempPhy->WToDbm (it->DataInterferenceW + tempPhy->GetCurrentNoiseW ()) + deltaInterferenceDb;
          expectedInterferenceWatt = tempPhy->DbmToW (expectedInterferenceDbm);
          deltaInterferenceWatt = expectedInterferenceWatt - it->DataInterferenceW  - tempPhy->GetCurrentNoiseW ();
          std::cout<<"P1_controller: "<<"current_pdr: "<< estimatedPdr <<" delta_interference_db: "<< deltaInterferenceDb<<" erSize: "<<tempPhy->GetErSize (it->LastDataErEdgeInterferenceW)<<" expected_interference: "<<expectedInterferenceWatt << std::endl;
#endif

#ifdef P_CONTROLLER_REFERENCE_I
          double currentNplusI = tempPhy->WToDbm (it->DataInterferenceW + tempPhy->GetCurrentNoiseW ());

          double supposedRxPowerDbm = tempPhy->GetPowerDbmByLevel (0) + tempPhy->GetTxGain () - signalMapItem->inBoundAttenuation;
          deltaInterferenceDb = m_pControllerWithReferenceInterference.GetDeltaInterference (supposedRxPowerDbm, m_desiredDataPdr, currentNplusI);
          expectedInterferenceDbm = tempPhy->WToDbm (it->DataInterferenceW + tempPhy->GetCurrentNoiseW ()) + deltaInterferenceDb;
          expectedInterferenceWatt = tempPhy->DbmToW (expectedInterferenceDbm);
          deltaInterferenceWatt = expectedInterferenceWatt - it->DataInterferenceW  - tempPhy->GetCurrentNoiseW ();
          std::cout<<"P2_controller: "<<"current_pdr: "<< estimatedPdr <<" delta_interference_db: "<< deltaInterferenceDb<<" erSize: "<<tempPhy->GetErSize (it->LastDataErEdgeInterferenceW)<<" expected_interference: "<<expectedInterferenceWatt<<" rx_power: "<<supposedRxPowerDbm <<" current_ni: "<<currentNplusI << std::endl;
#endif


          it->previousDataErW = it->LastDataErEdgeInterferenceW;
          Mac48Address edgeNode;
          if (deltaInterferenceDb != 0)
          {
            it->LastDataErEdgeInterferenceW = tempPhy-> GetErEdgeInterference (deltaInterferenceWatt, it->LastDataErEdgeInterferenceW, &edgeNode, conditionTwoMeet); //Since the data link pdr has changed, we also update the data ER edge interference. /Watt
          }
          if (it->previousDataErW < it->LastDataErEdgeInterferenceW) // when ER size decreases, always reset this;
          {
            m_maxBiDirectionalErChangeInformTimes = m_defaultInformTimes;
          }

          /*  Need a method to get the item, should not create a new one
           *
           */
          payloadItem.sender = sender.GetNodeId ();
          payloadItem.receiver = receiver.GetNodeId ();
          //payloadItem.isDataEr = true;
          payloadItem.edgeInterferenceW = it->LastDataErEdgeInterferenceW;
          payloadItem.updateSeqNo = (payloadItem.updateSeqNo + 1 ) % MAX_VERSION_NUMBER;
          payloadItem.itemPriority = DEFAULT_INFO_ITEM_PRIORITY + EXTRA_HIGHEST_PRIORITY_SENDING_TRIALS;
          UpdateControlInformation (payloadItem);

          m_controlMessagePriority = m_defaultPriority;
          if ( m_erInfoUpdatedTimeSlot == 0)
          {
            m_erInfoUpdatedTimeSlot = m_currentTimeslot;
          }

          linkMetaData.addr1 = m_self;
          linkMetaData.addr2 = sender;
          linkMetaData.edgeInterferenceW = it->LastDataErEdgeInterferenceW;
          AddOrUpdateErRegion (linkMetaData);
          break;
        }
      }
    }
  }

  /* Add or update ER edge interference for the current node. sort the vector according to the edge interference power
   * in a decreasing fashion. Note that if we want to get the maximum ER region for a specific node, we just have to fetch the very 
   * first item in this vector @m_linkMetaData; 
   * Since the network topology does not change during the simulation and there is no node failure, we suppose the link does not change
   * over time
   */
  void MacLow::AddOrUpdateErRegion (LinkMetaData item)
  {
    for(std::vector<LinkMetaData>::iterator iter = m_linkMetaData.begin (); iter != m_linkMetaData.end (); ++ iter)
    {
      if ( iter->addr1 == item.addr1 && iter->addr2 == item.addr2)
      {// update the er interference power
        iter->edgeInterferenceW = item.edgeInterferenceW;
        iter->interferenceNowDbm = item.interferenceNowDbm;
        iter->interferencePreviousDbm = item.interferencePreviousDbm;
        iter->lastComputedDeltaInterferenceDb = item.lastComputedDeltaInterferenceDb;
        iter->muBWatt = item.muBWatt;
        return;
      }
    }
    m_linkMetaData.push_back (item);
  }

  double MacLow::GetMaxErEdgeInterference () const
  {
    double maxErEdgeInterference = IMPOSSIBLE_ER;
    for(std::vector<LinkMetaData>::const_iterator iter = m_linkMetaData.begin (); iter != m_linkMetaData.end (); ++ iter)
    {
      if ( iter->edgeInterferenceW < maxErEdgeInterference)
      {
        maxErEdgeInterference = iter->edgeInterferenceW;
      }
    }
    return maxErEdgeInterference;
  }
  /* Get record according to the link. If there is no record for this link, create a new item and return it.
  */
  LinkMetaData MacLow::GetLinkMetaDataByLink (Mac48Address receiver, Mac48Address sender) const
  {
    for(std::vector<LinkMetaData>::const_iterator iter = m_linkMetaData.begin (); iter != m_linkMetaData.end (); ++ iter)
    {
      if ( iter->addr1 == receiver && iter->addr2 == sender)
      {
        return *iter;
      }
    }
    // if there is no record regarding this link, we create a new record and return;
    LinkMetaData linkMetaData;
    linkMetaData.addr1 = receiver; // address of the sender (no matter the ack sender or data sender)
    linkMetaData.addr2 = sender; // address of the receiver (including ack receiver and data receiver)
    linkMetaData.edgeInterferenceW = m_initialErEdgeInterferenceW; // the ER edge interference for this transmission from receiver to sender
    linkMetaData.interferenceNowDbm = 0.0;
    linkMetaData.interferencePreviousDbm = 0.0; // the real \DeltaI = m_interferenceNow - m_interferencePrevious;
    linkMetaData.lastComputedDeltaInterferenceDb = 0.0;
    linkMetaData.muBWatt = 0.0; // the average value of the interference outside of the ER
    return linkMetaData;
  }


  template <typename T>
    std::string MacLow::ToString (T const &val)
    {
      std::stringstream sstr;
      sstr << val;
      return sstr.str();
    }
  Payload MacLow::ParseControlPacketPayload (uint8_t* buffer, WifiMacHeader hdr)
  {

    Payload  payload;
    uint8_t size = 0;
    if (m_phy->GetObject<WifiImacPhy> ()->GetChannelNumber () == CONTROL_CHANNEL )
    {
      size = MAX_INFO_ITEM_SIZE;
    }
    else if ( m_phy->GetObject<WifiImacPhy> ()->GetChannelNumber () == DATA_CHANNEL)
    {
      size = MAX_INFO_ITEM_SIZE_IN_DATA_PACKET;
    }
    uint32_t itemSize = ER_INFO_ITEM_SIZE;

    //std::cout<<m_self<<" printing out receiving control signal! "<< std::endl;
    for (uint8_t i = 0; i < size; ++ i)
    {
      uint16_t sender = 0;
      ErInfoItem item;
      // sender
      sender = buffer[i * itemSize + 1] & 0x01;
      sender <<= 8;
      sender |= buffer[i * itemSize + 0];
      item.sender = sender;
      if ( item.sender > NETWORK_SIZE || item.sender < 1) // network size begins with 1
      {
        break;
      }

      // receiver
      TdmaLink linkInfo = Simulator::m_nodeLinkDetails[item.sender].selfInitiatedLink;
      item.receiver = Mac48Address (linkInfo.receiverAddr.c_str ()).GetNodeId ();

      // version number
      item.updateSeqNo = 0;
      item.updateSeqNo = (buffer[i * itemSize + 1]>> 1);

      // item priority
      uint16_t priority = 0;
      priority = (buffer [i * itemSize + 2] & 0x07);
      item.itemPriority = priority;


      // edge_interference
      uint16_t edgeInterferenceDbm = 0;
      edgeInterferenceDbm = ((buffer[i * itemSize + 2] >>3) & 0x1f);
      edgeInterferenceDbm <<= 5;
      edgeInterferenceDbm |= (buffer[i * itemSize + 3] & 0x7f);
      item.edgeInterferenceW = m_phy->GetObject<WifiImacPhy> ()->DbmToW ((double)edgeInterferenceDbm/AMPLIFY_TIMES * -1);

      payload.vec.push_back (item);
    }
    uint16_t maxErEdgeDbm = 0;
    maxErEdgeDbm = buffer [size * itemSize + 1] & 0x0f;
    maxErEdgeDbm <<= 8;
    maxErEdgeDbm |= buffer [size * itemSize];
    payload.maxErEdgeInterferenceW = m_phy->GetObject<WifiImacPhy> ()->DbmToW ( (double) maxErEdgeDbm/AMPLIFY_TIMES * -1);
    uint16_t ctrlChannelInterferenceDbm = 0;
    ctrlChannelInterferenceDbm = buffer [size * itemSize + 2];
    ctrlChannelInterferenceDbm <<= 4;
    ctrlChannelInterferenceDbm |= ((buffer [size * itemSize + 1] >> 4 ) & 0x0f);
    payload.controlChannelInterferenceW = m_phy->GetObject<WifiImacPhy> ()->DbmToW ( (double) (ctrlChannelInterferenceDbm/AMPLIFY_TIMES * (-1)));
    payload.nextRxTimeslot = buffer[size * itemSize + 4];
    payload.nextRxTimeslot <<= 8;
    payload.nextRxTimeslot |= buffer[size * itemSize + 3];
    payload.nextRxTimeslot += m_currentTimeslot;
    return payload;
  }

  uint32_t MacLow::ComputeDelayBetweenTwoTimeslots (uint32_t nowTimeslot, uint32_t targetTimeslot)
  {
    uint32_t temp = 0;
    uint32_t difference = 0;
    if ( nowTimeslot % m_maxTimeslotInPayload >= targetTimeslot)
    {
      difference = nowTimeslot % m_maxTimeslotInPayload - targetTimeslot;
    }
    else if ( nowTimeslot % m_maxTimeslotInPayload < targetTimeslot)
    {
      difference = targetTimeslot - nowTimeslot % m_maxTimeslotInPayload;
    }
    if (( nowTimeslot % m_maxTimeslotInPayload) > targetTimeslot &&
        difference < m_impossibleD0Value)
    {
      temp = (nowTimeslot % m_maxTimeslotInPayload) - targetTimeslot; //the first item must be the item of its own;
    }
    else if (difference >= m_impossibleD0Value) //overflow
    {
      temp = m_maxTimeslotInPayload + (nowTimeslot % m_maxTimeslotInPayload) - targetTimeslot; //the first item must be the item of its own;
    }
    return temp;
  }


  void MacLow::UpdateReceivedErInfoItem (ErInfoItem erItem,  Mac48Address controlPacketSender)
  {
    uint16_t originalSender = 0;
    originalSender = erItem.receiver;
    if ( originalSender == m_self.GetNodeId ())
    {
      return; // if the current node is the original sender of the er information item, we don't have to do anything.
    }
    for (std::vector<ErInfoItem>::iterator it = m_othersControlInformation.begin (); it != m_othersControlInformation.end (); ++ it)
    {
      //Locate the item in m_othersControlInformation
      if ( it->sender == erItem.sender && it->receiver == erItem.receiver)
      {      
        uint32_t difference = 0;
        if ( it->updateSeqNo >= erItem.updateSeqNo)
        {
          difference = it->updateSeqNo - erItem.updateSeqNo;
        }
        else if ( it->updateSeqNo < erItem.updateSeqNo)
        {
          difference = erItem.updateSeqNo - it->updateSeqNo;
        }

        //NEW ER estimation, judge according to verison number
        if (it->updateSeqNo < erItem.updateSeqNo || difference > IMPOSSIBLE_VER_NO_DIFFERENCE)
        {

          //--------------------------------------------------------------------------------
          //              UPDATE INFO ITEM FIELDS
          //--------------------------------------------------------------------------------
          it->edgeInterferenceW = erItem.edgeInterferenceW;
          it->updateSeqNo = erItem.updateSeqNo;
          if (controlPacketSender.GetNodeId () == erItem.sender || controlPacketSender.GetNodeId () == erItem.receiver) // is sent by the original sender, is not relayed. Re-set priority?
          {
            it->itemPriority= erItem.itemPriority == 0 ? erItem.itemPriority : erItem.itemPriority - 1;
          }
          else if (controlPacketSender.GetNodeId () != erItem.sender && controlPacketSender.GetNodeId () != erItem.receiver )// relayed information
          {
            if (it->itemPriority <= erItem.itemPriority)
            {
              it->itemPriority = it->itemPriority == 0 ? it->itemPriority : it->itemPriority - 1;
            }
            else if ( it->itemPriority > erItem.itemPriority)
            {
              it->itemPriority = erItem.itemPriority == 0 ? erItem.itemPriority : erItem.itemPriority - 1;
            }
          }


          ErInfoItem tempItem;
          CopyErInfoItem (it, &tempItem);
          m_othersControlInformation.erase (it);
          if ( m_othersControlInformation.size () == 0)
          {
            m_othersControlInformation.push_back (tempItem);
          }
          else if ( (m_othersControlInformation.end () -1)->itemPriority > tempItem.itemPriority )
          {
            m_othersControlInformation.push_back (tempItem);
          }
          else
          {
            for (std::vector<ErInfoItem>::iterator sub_it = m_othersControlInformation.begin (); sub_it != m_othersControlInformation.end (); ++ sub_it)
            {
              if (sub_it->itemPriority <= tempItem.itemPriority)
              {
                m_othersControlInformation.insert (sub_it, tempItem);
                break;
              }
            }
          }
        }
        return;
      }
    }

    erItem.itemPriority =erItem.itemPriority == 0 ? erItem.itemPriority : erItem.itemPriority - 1;
    if ( m_othersControlInformation.size () == 0)
    {
      m_othersControlInformation.push_back (erItem);
    }
    else if ( (m_othersControlInformation.end () - 1)->itemPriority > erItem.itemPriority )
    {
      m_othersControlInformation.push_back (erItem);
    }
    else
    {
      for (std::vector<ErInfoItem>::iterator sub_it = m_othersControlInformation.begin (); sub_it != m_othersControlInformation.end (); ++ sub_it)
      {
        if ( sub_it->itemPriority <= erItem.itemPriority)
        {
          m_othersControlInformation.insert (sub_it, erItem);
          break;
        }
      }
    }
  }


  std::vector<ErInfoItem> MacLow::SelectErInfoItemsToTransmit (uint32_t totalSize )
  {
    int32_t selfPriority = 0, othersPriority = 0;
    uint32_t itemSentCountInOthers = 0;
    m_informingRange = IMPOSSIBLE_ER;
    std::vector<ErInfoItem>::iterator selfIter = m_controlInformation.begin ();
    std::vector<ErInfoItem>::iterator othersIter = m_othersControlInformation.begin ();
    std::vector<ErInfoItem> itemsSentInOthers;
    if (selfIter != m_controlInformation.end ())
    {
      selfPriority = selfIter->itemPriority;
    }
    else
    {
      selfPriority = -1;
    }
    if (othersIter != m_othersControlInformation.end ())
    {
      othersPriority = othersIter->itemPriority;
    }
    else 
    {
      othersPriority = -1;
    }
    std::vector<ErInfoItem> retVector;
    for (uint32_t i=0; i < totalSize; ++ i)
    {
      if ( retVector.size () == totalSize || (selfIter == m_controlInformation.end () && othersIter == m_othersControlInformation.end ()) )
      {
        break;
      }
      if ( selfPriority >= othersPriority && selfPriority > 0 && selfIter != m_controlInformation.end ())
      {

        if ( selfIter->edgeInterferenceW < m_informingRange)
        {
          //for power control
          m_informingRange = selfIter->edgeInterferenceW;
        }
        retVector.push_back ( *selfIter );
        if (selfIter->itemPriority != 0)
        {
          selfIter->itemPriority -= 1;
        }
        if (selfIter != m_controlInformation.end ())
        {
          selfIter ++;
        }
        if (selfIter == m_controlInformation.end ())
        {
          selfPriority = -1;
        }
        else
        {
          selfPriority = selfIter->itemPriority;
        }
      }
      else if ( selfPriority < othersPriority && othersIter != m_othersControlInformation.end ())
      {
        std::vector<TdmaLink> relatedLinks = Simulator::m_nodeLinkDetails[m_self.GetNodeId ()].relatedLinks;
        std::vector<std::string> relatedNodes;
        for (std::vector<TdmaLink>::iterator it = relatedLinks.begin (); it != relatedLinks.end (); ++ it)
        {
          if ( it->senderAddr == m_self.ToString ())
          {
            relatedNodes.push_back (it->receiverAddr);
          }
          else if (it->receiverAddr == m_self.ToString ())
          {
            relatedNodes.push_back (it->senderAddr);
          }
        }
        relatedNodes.insert (relatedNodes.begin (), m_self.ToString ());

        Mac48Address receiver = Mac48Address ( IntToMacAddress (othersIter->receiver).c_str ());

        std::vector<std::string> nodesInEr;
        nodesInEr = Simulator::ListNodesInEr (receiver.ToString (), othersIter->edgeInterferenceW);

        for (std::vector<std::string>::iterator nodesIt = relatedNodes.begin (); nodesIt != relatedNodes.end (); ++ nodesIt)
        {
          if (find (nodesInEr.begin (), nodesInEr.end (), *nodesIt) != nodesInEr.end ())// if the current node is in the link's ER
          {
            retVector.push_back ( *othersIter );
            ErInfoItem temp;
            CopyErInfoItem (othersIter, &temp);
            itemsSentInOthers.push_back (temp); // record the item that will be sent
            if (temp.itemPriority != 0)
            {
              temp.itemPriority  -= 1;
            }
            ErInfoItem tempItem = *othersIter;
            *othersIter = m_othersControlInformation[itemSentCountInOthers];
            m_othersControlInformation[itemSentCountInOthers] = tempItem; //move the sent item to the front of the vector, later, we calculate their position in the vector to realize the round-robin mechanism
            itemSentCountInOthers += 1;
            break;
          }
        }
        //retVector.push_back ( *othersIter );
        if (othersIter != m_othersControlInformation.end ())
        {
          othersIter ++;
        }
        if (othersIter == m_othersControlInformation.end ())
        {
          othersPriority = -1;
        }
        else
        {
          othersPriority = othersIter->itemPriority;
        }

      }
    }
    // only when send out a control message, sort the vector
    //sort (m_othersControlInformation.begin (), m_othersControlInformation.end (), ErInfoItemCompare);
    for (std::vector<ErInfoItem>::iterator it = itemsSentInOthers.begin (); it != itemsSentInOthers.end (); ++ it)
    {
      // find the sent item one by one, delete them from the original vector,
      for (std::vector<ErInfoItem>::iterator sub_it = m_othersControlInformation.begin (); sub_it != m_othersControlInformation.end (); ++ sub_it)
      {
        if ( sub_it->itemId == it->itemId)
        {
          m_othersControlInformation.erase (sub_it);
          break;
        }
      }
      // insert the sent items in the right place
      if ( m_othersControlInformation.size () == 0 || (m_othersControlInformation.end () -1)->itemPriority >= it->itemPriority)
      {
        m_othersControlInformation.push_back (*it);
      }
      else
      {
        for (std::vector<ErInfoItem>::iterator sub_it = m_othersControlInformation.begin (); sub_it != m_othersControlInformation.end (); ++ sub_it)
        {
          if ( sub_it->itemPriority < it->itemPriority)
          {
            m_othersControlInformation.insert (sub_it, *it);
            break;
          }
        }
      }
    }
    sort (m_controlInformation.begin (), m_controlInformation.end (), ErInfoItemCompare);
    return retVector;
  }

  uint8_t* MacLow::GenerateControlPayload (uint32_t max_size, uint32_t item_size, uint8_t payload[])
  {
    uint32_t size_count = 0;
    int32_t maxPriorityInControlPacketPayload = 0;

    std::vector<ErInfoItem> itemsToSend;
    if (Simulator::Now () > Simulator::LearningTimeDuration )
    {
      itemsToSend = SelectErInfoItemsToTransmit (max_size);
    }
    else if ( Simulator::Now () < Simulator::LearningTimeDuration )
    {
      itemsToSend = m_controlInformation;
    }
    uint8_t *ptr = NULL;
    m_setLisenterCallback ();
    m_controlPacketPayload.clear ();
    for (uint32_t i = 0; i < CONTROL_PAYLOAD_LENGTH; ++ i) // intialization
    {
      payload[i] = 0;
    }
    ptr = payload;
    for ( std::vector<ErInfoItem>::iterator it = itemsToSend.begin (); it != itemsToSend.end (); ++ it)
    {
      size_count ++;
      if ( size_count > max_size)
      {
        break;
      }
      //Sender Id
      *(ptr++) = (it->sender & 0xff); //the lower 8-bit of the sender. 1 byte finishes
      *(ptr) = ((it->sender >> 8) & 0x01); // the higher 1-bit of the sender
      // Version number
      *(ptr++) |= ((it->updateSeqNo) << 1); // 7 bits
      //Item priority
      if (it->itemPriority > DEFAULT_INFO_ITEM_PRIORITY)
      {
        it->itemPriority = DEFAULT_INFO_ITEM_PRIORITY;
      }
      if ( it->itemPriority > maxPriorityInControlPacketPayload)
      {
        maxPriorityInControlPacketPayload = it->itemPriority;
        m_controlMessagePriority = maxPriorityInControlPacketPayload;
      }
      *(ptr) = (it->itemPriority & 0x07);// 3 bits
      //ER edge
      uint16_t edgeInterferenceDbm = (uint16_t)( -AMPLIFY_TIMES * m_phy->GetObject<WifiImacPhy> ()->WToDbm (it->edgeInterferenceW));
      *(ptr++) |= ((edgeInterferenceDbm & 0x1f) << 3); // lower 5 bits
      *(ptr++) = ((edgeInterferenceDbm >> 5) & 0x7f); //higher 5 bits

    }
    if ( size_count <= max_size )
    {
      uint32_t invalid_sender = INVALID_SENDER;
      *(ptr++) = (invalid_sender & 0xff); //the lower 8-bit of the sender
      *(ptr) = ((invalid_sender >> 8) & 0x01); // the higher 1-bit of the sender
    }

    double controlChannelInterferenceW = m_phy->GetObject<WifiImacPhy> () ->ControlChannel_ComputeInterferenceWhenReceivingData ();
    double maxErEdgeInterferenceW = GetMaxErEdgeInterference ();

    if ( controlChannelInterferenceW == 0)
    {
      controlChannelInterferenceW = DEFAULT_INTERFERENCE_W; // a negligible value
    }

    uint16_t controlChannelInterferenceDbm = (uint16_t)( -AMPLIFY_TIMES * m_phy->GetObject<WifiImacPhy> ()->WToDbm (controlChannelInterferenceW));
    uint16_t maxErEdgeInterferenceDbm = (uint16_t)( -AMPLIFY_TIMES * m_phy->GetObject<WifiImacPhy> ()->WToDbm (maxErEdgeInterferenceW));
    payload [max_size * item_size] = maxErEdgeInterferenceDbm & 0xff;
    payload [max_size * item_size + 1] = ((maxErEdgeInterferenceDbm >> 8) & 0x0f);
    payload [max_size * item_size + 1] |=  (controlChannelInterferenceDbm & 0x0f) << 4;
    payload [max_size * item_size + 2] = ((controlChannelInterferenceDbm >> 4) & 0xff);
    payload [max_size * item_size + 3] = (m_nextSendingSlot - m_currentTimeslot) & 0xff;
    payload [max_size * item_size + 4] = ((m_nextSendingSlot - m_currentTimeslot) >> 8) & 0xff;
    ptr = NULL;
    return payload;
  }

  /* When the link does not have the maximum link priority, both the sender and the receiver of the link keep staying in 
   * the control channel, and sense the channel, if the channel is idle, broadcast control packet.
   */
  void MacLow::TrySendControlPacket ()
  {
    if ( m_phy->IsStateIdle ())
    {
      uint8_t payload[CONTROL_PAYLOAD_LENGTH];
      GenerateControlPayload (MAX_INFO_ITEM_SIZE, ER_INFO_ITEM_SIZE, payload);

      Ptr<Packet> pkt = Create<Packet> (payload, CONTROL_PAYLOAD_LENGTH);
      WifiMacHeader hdr;
      hdr.SetAddr2 (m_self);
      hdr.SetDsNotTo ();
      hdr.SetDsNotFrom ();
      hdr.SetFragmentNumber (0);
      hdr.SetNoRetry ();
      hdr.SetAddr1 (m_self);
      hdr.SetTypeData ();
      MacLowTransmissionParameters params;
      params.DisableAck ();
      params.DisableRts ();
      params.DisableOverrideDurationId ();
      params.DisableNextData ();
      m_currentPacket = pkt;
      m_currentHdr = hdr;
      m_setPacketCallback (hdr, pkt);
      m_txParams = params;
      NS_ASSERT (m_phy->GetObject<WifiImacPhy> () ->GetChannelNumber () == CONTROL_CHANNEL);

      //For Control signal reliability
      std::vector<std::string> nodesInEr = Simulator::ListNodesInEr (m_self.ToString (), m_informingRange);
      Simulator::SenderRegisterControlReliability (m_self.ToString (), nodesInEr);

      SendDataPacket ();
      if ( m_controlMessagePriority != 0)
      {
        m_controlMessagePriority -- ;
      }
    }
    return;
  }
  void MacLow::GenerateDataPacketAndSend ()
  {  
    NS_ASSERT (m_phy->GetObject<WifiImacPhy> ()->GetChannelNumber () == DATA_CHANNEL);
    std::cout<<Simulator::Now () <<" "<<m_self<<" queue_size: "<< m_packetQueue.size () << std::endl;
    if ( m_packetQueue.size () > 0 ) // if there are no packets to send, just return;
    {
      m_packetQueue.pop_back ();
    }
    else
    {
      return;
    }
    m_setLisenterCallback ();

    uint8_t payload[DATA_PACKET_PAYLOAD_LENGTH];
    GenerateControlPayload (MAX_INFO_ITEM_SIZE_IN_DATA_PACKET, ER_INFO_ITEM_SIZE, payload);

    Ptr<Packet> pkt = Create<Packet> (payload, DATA_PACKET_PAYLOAD_LENGTH);
    //Ptr<Packet> pkt = Create<Packet> (DATA_PACKET_PAYLOAD_LENGTH); // packet size get from scratch/imac.cc
    WifiMacHeader hdr;
    hdr.SetAddr2 (m_self);
    NS_ASSERT (!m_dataReceiverAddr.IsGroup ());
    hdr.SetAddr1 (m_dataReceiverAddr);
    hdr.SetDsNotTo ();
    hdr.SetDsNotFrom ();
    hdr.SetFragmentNumber (0);
    hdr.SetNoRetry ();

    hdr.SetTypeData ();
    MacLowTransmissionParameters params;
    params.EnableAck ();
    params.DisableRts ();
    params.DisableOverrideDurationId ();
    params.DisableNextData ();
    m_currentPacket = pkt;
    m_currentHdr = hdr;
    m_setPacketCallback (hdr, pkt);
    m_txParams = params;
    m_sendingCount ++;
    SendDataPacket ();
  }
  void MacLow::SetMacLowTransmissionListener (MacLowTransmissionListener *listener )
  {
    m_listener = listener;
  }
  void MacLow::SetListenerCallback (VoidCallback callback)
  {
    m_setLisenterCallback = callback;
  }
  void MacLow::SetDcaTxopPacketCallback (SetPacketCallback callback)
  {
    m_setPacketCallback = callback;
  }

  void MacLow::SenseChannelLater ()
  {
    if ( m_phy->IsStateIdle ())
    {
      Simulator::Schedule (NanoSeconds (MAX_PROPAGATION_DELAY), &MacLow::TrySendControlPacket, this); // 190ns is the signal propagation delay from one side of the network to the other side of the network
    }
  }


  void MacLow::ScheduleControlSignalTransmission ()
  {
    int64_t seed = m_currentTimeslot * Simulator::NodesCountUpperBound + m_self.GetNodeId ();
    srand (seed ); // set rand seed;
    int singlePriorityTime = NanoSeconds (MAX_BACKOFF_TIME).GetNanoSeconds () / (m_defaultPriority + 1);
    int backoffTime = rand () % singlePriorityTime; 
    backoffTime = backoffTime + (m_defaultPriority - m_controlMessagePriority) * singlePriorityTime;
    Simulator::Schedule (NanoSeconds (abs (backoffTime)), &MacLow::SenseChannelLater, this);
  }



  ErInfoItem MacLow::GetErInfoItem (uint16_t sender, uint16_t receiver, bool isDataEr, bool isSelf)
  {
    ErInfoItem item;
    item.sender = 0; 
    item.receiver = 0;
    if (isSelf == true )
    {
      for (std::vector<ErInfoItem>::iterator it = m_controlInformation.begin (); it != m_controlInformation.end (); ++ it)
      {
        if (it->sender == sender && it->receiver == receiver)
        {
          item = *it;
          break;
        }
      }
    }
    else
    {
      for (std::vector<ErInfoItem>::iterator it = m_othersControlInformation.begin (); it != m_othersControlInformation.end (); ++ it)
      {
        if (it->sender == sender && it->receiver == receiver)
        {
          item = *it;
          break;
        }
      }
    }
    if ( item.sender == item.receiver && item.sender== 0) // in case there is no such item
    {
      item = GenerateDefaultErInfoItem (sender, receiver, true);
    }
    return item;
  }

  void MacLow::UpdateControlInformation (ErInfoItem payload)
  {

    for (std::vector<ErInfoItem>::iterator it = m_controlInformation.begin (); it != m_controlInformation.end (); ++ it)
    {
      if (payload.sender == it->sender && payload.receiver == it->receiver)
      {
        ErInfoItem temp;
        CopyErInfoItem (&payload, &temp);
        m_controlInformation.erase (it);
        m_controlInformation.insert (m_controlInformation.begin (), temp);
        return;
      }
    }
    m_controlInformation.insert (m_controlInformation.begin (), payload);
    sort (m_controlInformation.begin (), m_controlInformation.end (), ErInfoItemCompare); // sort according to priority, 
  }

  std::string MacLow::IntToMacAddress (uint16_t nodeId)
  {
    string str = "00:00:00:00:00:00";
    if ( nodeId < 16)
    {
      if (nodeId < 10)
      {
        str[16] = '0'+nodeId;
      }
      else if (nodeId >=10 && nodeId <16)
      {
        str[16] = 'a'+nodeId - 10;
      }
      return str;
    }
    else if (nodeId < 16*16)
    {
      uint16_t second = nodeId / 16;
      if (second < 10)
      {
        str[15] = '0'+second;
      }
      else if ( second >= 10 && second <16)
      {
        str[15] = 'a'+second - 10;
      }
      uint16_t first = nodeId % 16;
      if (first < 10)
      {
        str[16] = '0'+first;
      }
      else if (first >= 10 && second <16)
      {
        str[16] = 'a'+first - 10;
      }
      return str;
    }
    else if (nodeId < 16*16*16)
    {
      uint16_t third = nodeId / (16*16);
      if (third < 10)
      {
        str[13] = '0'+ third;
      }
      else if ( third >= 10 && third <16)
      {
        str[13] = 'a'+ third - 10;
      }
      uint16_t second = (nodeId % (16*16))/ 16;
      if (second < 10)
      {
        str[15] = '0'+ second;
      }
      else if ( second >= 10 && second <16)
      {
        str[15] = 'a'+ second - 10;
      }
      uint16_t first = nodeId % 16;
      if (first < 10)
      {
        str[16] = '0'+ first;
      }
      else if (first >= 10 && second <16)
      {
        str[16] = 'a'+ first - 10;
      }
      return str;
    }
    return str;
  }
  void MacLow::UpdateMaxEr (NodeMaxErItem item)
  {
    bool found = false;
    for (std::vector<NodeMaxErItem>::iterator it = m_maxErs.begin (); it != m_maxErs.end (); ++ it)
    {
      if ( it->address  == item.address)
      {
        it->erEdgeInterferenceW = item.erEdgeInterferenceW;
        found = true;
        break;
      }
    }
    if ( found == false )
    {
      m_maxErs.push_back (item);
    }

  }


  double MacLow::FindMaxErEdge (Mac48Address address) const
  {
    for (std::vector<NodeMaxErItem>::const_iterator it = m_maxErs.begin (); it != m_maxErs.end (); ++ it)
    {
      if ( it->address == address)
      {
        return it->erEdgeInterferenceW;
      }
    }
    return m_initialErEdgeInterferenceW;
  }

  bool MacLow::CheckSenderInTxEr(Mac48Address address)
  {
    double nodeMaxErW = FindMaxErEdge (address);
    std::vector<std::string> nodesInEr = Simulator::ListNodesInEr (address.ToString (), nodeMaxErW);
    return find (nodesInEr.begin (), nodesInEr.end (), m_self.ToString ()) != nodesInEr.end ();
  }

  double MacLow::FindQuantileValue (double quantile, std::vector<double> &vec)
  {
    if (vec.size () == 0)
    {
      return 0;
    }
    std::vector<double> tempVec  = vec; //copy
    uint32_t index = (uint32_t) (quantile * vec.size ());
    index = index == 0 ? index : index - 1; // index starts from 0 to n-1, minus 1
    QuickSort quick(&tempVec);
    //sort (tempVec.begin (), tempVec.end ());//ascending  sort
    return tempVec[index];
  }

  bool MacLow::CompareErInfoItem ( ErInfoItem a, ErInfoItem b)
  {
    return a.itemId <= b.itemId;
  }

  void MacLow::quick_sort(std::vector<ErInfoItem> *sortArray)
  {
    //std::cout<<m_self<<" invoking sorting algorithm! "<< std::endl;
    int startIndex = 0;
    int endIndex = 0;

    endIndex = (int)(*sortArray).size() - 1;

    quick_sort(sortArray, startIndex, endIndex);
  }

  void MacLow::quick_sort(std::vector<ErInfoItem> *sortArray, int startIndex, int endIndex)
  {
    /*
       std::cout<<" sorting result: "<< std::endl;
       for (std::vector<ErInfoItem>::iterator it = sortArray->begin (); it != sortArray->end (); ++ it)
       {
       std::cout<<" id: "<< it->itemId<< std::endl;
       }
       std::cout<<" startIndex "<<startIndex << " endIndex "<< endIndex << std::endl;
       */
    if(startIndex >= endIndex)
    {
      //std::cout<<" ends!"<< std::endl;
      return;
    }

    // *initialize* the array
    sortingArray = sortArray;

    // For instance: 1..n
    int left = startIndex;
    int right = endIndex;

    if(left < right)
    {
      // get pivot
      int pivot = Partition(sortingArray, left, right);
      //std::cout<<" pivot "<< pivot << std::endl;

      // sort left side
      quick_sort(sortingArray, left, (pivot-1) );

      // sort right side
      quick_sort(sortingArray, (pivot+1), right);
    }
  }


  int MacLow::Partition(std::vector<ErInfoItem> *sortArray, int startIndex, int endIndex)
  {       
    // initially this start - 1 when startIndex is 0.
    int l = startIndex - 1;
    int k = endIndex;

    for(int i = startIndex; i <= k - 1; i++)
    {
      // Go until you find a value smaller than the last value.
      //if((*sortArray)[i] <= (*sortArray)[k])
      if ( (*sortArray)[i].itemId <= (*sortArray)[k].itemId )
      {
        // increment l
        l++;

        // swap i and j
        // NOTE: this is supposed to swap j with itself the first time.
        Swap(l, i);       
      }
    }   

    // when loop is finished, swap
    Swap(l + 1, k);

    return l + 1;
  }

  void MacLow::Swap(int l, int k)
  {
    // create temp variable
    ErInfoItem tmp;

    // store first element in temp
    //tmp = (*sortingArray)[l];
    CopyErInfoItem (&((*sortingArray)[l]), &tmp);
    // swap second element to first element
    //(*sortingArray)[l] = (*sortingArray)[k];

    //(*sortingArray)[l] = (*sortingArray)[k];
    CopyErInfoItem (&((*sortingArray)[k]), &((*sortingArray)[l]));
    // put temp variable in second element
    //(*sortingArray)[k] = tmp;
    CopyErInfoItem (&tmp, &((*sortingArray)[k]));
  }



  uint32_t MacLow::BinarySearch (std::vector<uint32_t> &vec, uint32_t key)
  {
    int low = 0;
    int high = vec.size () - 1;
    int mid = 0;
    while ( low <= high)
    {
      mid = low + (high-low)/2; 
      if ( vec[mid] == key )
      {
        return vec[mid];
      }
      else if ( vec[mid] < key)
      {
        low = mid + 1;
      }
      else  if ( vec[mid] > key)
      {
        high = mid - 1;
      }
    }
    return 0;
  }


  // copy first to second;
  void MacLow::CopyErInfoItem (ErInfoItem *b, ErInfoItem *a)
  {
    a->itemId = b->itemId;
    a->sender = b->sender;
    a->receiver = b->receiver;
    a->edgeInterferenceW = b->edgeInterferenceW;
    a->updateSeqNo = b->updateSeqNo;
    a->itemPriority = b->itemPriority;
  }

  // copy first to second;
  void MacLow::CopyErInfoItem (std::vector<ErInfoItem>::iterator b, ErInfoItem *a)
  {
    a->itemId = b->itemId;
    a->sender = b->sender;
    a->receiver = b->receiver;
    a->edgeInterferenceW = b->edgeInterferenceW;
    a->updateSeqNo = b->updateSeqNo;
    a->itemPriority = b->itemPriority;
  }

  double MacLow::FindDeliverTime (uint16_t sender, uint16_t receiver, std::vector<LinkD0> *vecPtr, uint32_t category)
  {
    for (std::vector<LinkD0>::iterator it = vecPtr->begin (); it != vecPtr->end (); ++ it)
    {
      if (it->sender == sender && it->receiver == receiver)
      {
        if ( category == ER_INFO_ITEM_CATEGORY_ONE )
        {
          //std::cout<<" found category one: "<< it->d0CategoryOne << std::endl;
          return it->d0CategoryOne;
        }
        else if (category == ER_INFO_ITEM_CATEGORY_TWO )
        {
          //std::cout<<" found category two: "<< it->d0CategoryOne << std::endl;
          return it->d0CategoryTwo;
        }
      }
    }
    return 0; // cannot use negative value, will be converted into unsigned integers
  }

  void MacLow::SetInitialEr (uint16_t sender, uint16_t receiver, double initialErW)
  {
    for (std::vector<ErInfoItem>::iterator it = m_controlInformation.begin (); it != m_controlInformation.end (); ++ it)
    {
      if (it->sender == sender && it->receiver == receiver)
      {
        it->edgeInterferenceW = initialErW;
        return;
      }
    }
    ErInfoItem item = GenerateDefaultErInfoItem (sender, receiver, true);
    item.edgeInterferenceW = initialErW;
    m_controlInformation.push_back (item);

  }

  std::vector<LinkD0>::iterator MacLow::FindLinkD0Item ( uint16_t sender, uint16_t receiver, std::vector<LinkD0> &vec)
  {
    for (std::vector<LinkD0>::iterator it = vec.begin (); it != vec.end (); ++ it)
    {
      if (it->sender == sender && it->receiver == receiver )
      {
        return it;
      }
    }
    return vec.end ();
  }

  void MacLow::UpdateNodeTxProbability ( NodesTxProbability item )
  {
    for ( std::vector<NodesTxProbability>::iterator it = m_nodesTxProbability.begin (); it != m_nodesTxProbability.end (); ++ it)
    {
      if ( it->nodeId == item.nodeId)
      {
#ifdef ESTIMATED_MAX 
        if (it->txProbability == DEFAULT_TX_PROBABILITY && item.txProbability != 0)
        {
          it->txProbability = item.txProbability;
        }
        else if ( item.txProbability > it->txProbability)
        {
          it->txProbability = item.txProbability;
        }
#endif
#ifdef TX_DURATION_PROBABILITY
        it->txProbability = item.txProbability;
#endif
#ifdef ESTIMATED_3_STD
        it->difference = item.txProbability - it->mean;
        it->increment = m_ewmaCoefficient * it->difference;
        it->mean = it->mean + it->difference;
        it->variance = (1 - m_ewmaCoefficient)* (it->mean + it->difference * it->increment);
        it->txProbability = it->mean + 3 * sqrt (it->variance);
#endif
        return;
      }
    }
    m_nodesTxProbability.push_back (item);
  }

  double MacLow::GetNodeTxProbability ( uint16_t nodeId ) const
  {
    for (std::vector<NodesTxProbability>::const_iterator it = m_nodesTxProbability.begin (); it != m_nodesTxProbability.end (); ++ it)
    {
      if ( it->nodeId == nodeId )
      {
        //std::cout<<Simulator::Now () <<" "<<nodeId<<" tx_probability: "<< it->txProbability << std::endl;
        return it->txProbability;
      }
    }
    //std::cout<<Simulator::Now () <<" "<<nodeId<<" tx_probability: "<< DEFAULT_TX_PROBABILITY << std::endl;
    return DEFAULT_TX_PROBABILITY;
  }

  double MacLow::InsertD0Sample (uint16_t sender, uint16_t receiver, double sample, uint32_t category)
  {
    // if find, insert and get a quantile, return
    for (std::vector<LinkD0Samples>::iterator it = m_linkD0SamplesVec.begin (); it != m_linkD0SamplesVec.end (); ++ it)
    {
      if ( it->sender == sender && it->receiver == receiver)
      {
        if (category == ER_INFO_ITEM_CATEGORY_ONE)
        {
          if ( it->d0SampleCategoryOne.size () == MAX_D0_SAMPLE_SIZE)
          {
            it->d0SampleCategoryOne.erase (it->d0SampleCategoryOne.begin ());
          }
          it->d0SampleCategoryOne.push_back (sample);
          return FindQuantileValue (D_0_QUANTILE, it->d0SampleCategoryOne);
        }
        else if ( category == ER_INFO_ITEM_CATEGORY_TWO)
        {
          if (it->d0SampleCategoryTwo.size () == MAX_D0_SAMPLE_SIZE)
          {
            it->d0SampleCategoryTwo.erase (it->d0SampleCategoryTwo.begin ());
          }
          it->d0SampleCategoryTwo.push_back (sample);
          return FindQuantileValue (D_0_QUANTILE, it->d0SampleCategoryTwo);
        }
      }
    }
    // if cannot find, create a new item and return the sampled value;
    LinkD0Samples  linkD0Sample;
    linkD0Sample.sender = sender;
    linkD0Sample.receiver = receiver;
    if (category == ER_INFO_ITEM_CATEGORY_ONE )
    {
      linkD0Sample.d0SampleCategoryOne.push_back (sample);
    }
    else if ( category == ER_INFO_ITEM_CATEGORY_TWO)
    {
      linkD0Sample.d0SampleCategoryTwo.push_back (sample);
    }
    m_linkD0SamplesVec.push_back (linkD0Sample);
    return sample;
  }


  void MacLow::GeneratePacket ()
  {
    Time generationInterval = MilliSeconds (PACKET_GENERATION_INTERVAL); // every 50 ms
    //if (m_uniform.GetValue () <= m_packetGenreationProbability && m_packetQueue.size () <= QUEUE_SIZE)
    if (m_packetQueue.size () <= QUEUE_SIZE)
    {
      m_packetQueue.push_back (1);
    }
    if (Simulator::Now () > Simulator::SimulationStopTime)
    {
      return;
    }
    Simulator::Schedule (generationInterval, &MacLow::GeneratePacket, this);

  }

} // namespace ns3




