/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005, 2006 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
 *
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
#ifndef MAC_LOW_H
#define MAC_LOW_H

#include <vector>
#include <stdint.h>
#include <ostream>
#include <map>
#include "controller.h"
#include "wifi-mac-header.h"
#include "wifi-mode.h"
#include "wifi-preamble.h"
#include "wifi-remote-station-manager.h"
#include "ctrl-headers.h"
#include "mgt-headers.h"
#include "block-ack-agreement.h"
#include "ns3/mac48-address.h"
#include "ns3/callback.h"
#include "math-helper.h"
#include "ns3/event-id.h"
#include "ns3/packet.h"
#include "ns3/nstime.h"
#include "qos-utils.h"
#include "block-ack-cache.h"
#include "wifi-imac-phy.h"
#include "settings.h"

namespace ns3 {
  class WifiPhy;
  class WifiMac;
  class EdcaTxopN;

  /**
   * \ingroup wifi
   * \brief listen to events coming from ns3::MacLow.
   */
  // added by Chuan
  typedef struct NextTxSlotInfo //This is for the case when @m_self is the receiver
  {
    int64_t linkId;
    int64_t nextSlotFromSelf; //Calculated as a receiver;
    //std::vector<int64_t> nextSlots;
    int64_t nextSlotFromSender;
    bool beConservative;
  } NextTxSlotInfo;
  typedef struct NextRxSlotInfo
  {
    int64_t linkId;
    int64_t nextSlot;
  } NextRxSlotInfo;
  typedef struct NewErRxStatus
  {
    int64_t linkId;
    bool allowPdrEstimation;
    int64_t slotCount; // -1: Do not count.  0: start counting
  } NewErRxStatus;
  typedef struct LinkEstimatorItem
  {
    uint16_t DataSequenceNo; // the data sequence number from sender S to receiver R
    uint16_t AckSequenceNo; // the ack sequence number from receiver R to sender S
    double PreviousDataPdr;
    double PreviousAckPdr;
    double DataPdr; // the estimated data pdr at the receiver side;
    double AckPdr; // the estimated ack pdr at the sender side;
    Mac48Address Sender; // the sender of the data link
    Mac48Address Receiver;// the receiver of the data link
    std::vector<uint16_t> ReceivedDataPacketNumbers;
    std::vector<uint16_t> ReceivedAckPacketNumbers;
    uint16_t LastDataSequenceNo;
    uint16_t LastAckSequenceNo;
    double previousDataErW;
    double previousAckErW;
    double LastDataErEdgeInterferenceW;
    double LastAckErEdgeInterferenceW;
    double DataInterferenceW;
    double AckInterferenceW;
  }LinkEstimatorItem;



  typedef struct ErInfoItem 
  {
    uint32_t itemId;
    uint16_t sender;
    uint16_t receiver;
    float edgeInterferenceW;
    uint8_t updateSeqNo;
    int32_t itemPriority;
    double risingAchieved;
  } ErInfoItem;


  typedef struct Payload 
  {
    std::vector<ErInfoItem> vec;
    double maxErEdgeInterferenceW;
    double controlChannelInterferenceW;
    int64_t nextRxTimeslot;
    bool ErRxFromSender;
    double txProbability;
    std::vector<NextRxSlotInfo> rxVec;
  } Payload;

  typedef struct NodesTxProbability
  {
    uint16_t nodeId;
    double txProbability;
    double difference;
    double increment;
    double mean;
    double variance;
  } NodesTxProbability;

  typedef struct LinkPriority 
  {
    int64_t linkId; 
    int64_t priority;
    std::string senderAddr;
    std::string receiverAddr;
  } LinkPriority;
  typedef struct LinkMetaData
  {
    Mac48Address addr1; // address of the sender (no matter the ack sender or data sender)
    Mac48Address addr2; // address of the receiver (including ack receiver and data receiver)
    double edgeInterferenceW; // the ER edge interference for this transmission from addr1 to addr2
    double interferenceNowDbm;
    double interferencePreviousDbm; // the real \DeltaI = m_interferenceNow - m_interferencePrevious;
    double lastComputedDeltaInterferenceDb;
    double muBWatt; // the average value of the interference outside of the ER
  } LinkMetaData;

  typedef struct NodeMaxErItem
  {
    Mac48Address address;
    double erEdgeInterferenceW;
  } NodeMaxErItem;



  class MacLowTransmissionListener
  {
    public:
      MacLowTransmissionListener ();
      virtual ~MacLowTransmissionListener ();

      /**
       * \param snr the snr of the cts
       * \param txMode the txMode of the cts
       *
       * ns3::MacLow received an expected CTS within
       * CtsTimeout.
       */
      virtual void GotCts (double snr, WifiMode txMode) = 0;
      /**
       * ns3::MacLow did not receive an expected CTS
       * within CtsTimeout.
       */
      virtual void MissedCts (void) = 0;
      /**
       * \param snr the snr of the ack
       * \param txMode the transmission mode of the ack
       *
       * ns3::MacLow received an expected ACL within
       * AckTimeout. The <i>snr</i> and <i>txMode</i>
       * arguments are not valid when SUPER_FAST_ACK is
       * used.
       */
      virtual void GotAck (double snr, WifiMode txMode) = 0;
      /**
       * ns3::MacLow did not receive an expected ACK within
       * AckTimeout.
       */
      virtual void MissedAck (void) = 0;
      /**
       * \param blockAck Block ack response header
       * \param source Address of block ack sender
       *
       * Invoked when ns3::MacLow receives a block ack frame.
       * Block ack frame is received after a block ack request
       * and contains information about the correct reception
       * of a set of packet for which a normal ack wasn't send.
       * Default implementation for this method is empty. Every
       * queue that intends to be notified by MacLow of reception
       * of a block ack must redefine this function.
       */
      virtual void GotBlockAck (const CtrlBAckResponseHeader *blockAck, Mac48Address source);
      /**
       * ns3::MacLow did not receive an expected BLOCK_ACK within
       * BlockAckTimeout. This method is used only for immediate
       * block ack variant. With delayed block ack, the MissedAck method will be
       * called instead: upon receipt of a block ack request, the rx station will
       * reply with a normal ack frame. Later, when the rx station gets a txop, it
       * will send the block ack back to the tx station which will reply with a
       * normal ack to the rx station.
       */
      virtual void MissedBlockAck (void);
      /**
       * Invoked when ns3::MacLow wants to start a new transmission
       * as configured by MacLowTransmissionParameters::EnableNextData.
       * The listener is expected to call again MacLow::StartTransmission
       * with the "next" data to send.
       */
      virtual void StartNext (void) = 0;

      /**
       * Invoked if this transmission was canceled
       * one way or another. When this method is invoked,
       * you can assume that the packet has not been passed
       * down the stack to the PHY.
       */
      virtual void Cancel (void) = 0;
  };


  /**
   * \brief listen to NAV events
   * \ingroup wifi
   *
   * This class is typically connected to an instance of ns3::Dcf
   * and calls to its methods are forwards to the corresponding
   * ns3::Dcf methods.
   */
  class MacLowDcfListener
  {
    public:
      MacLowDcfListener ();
      virtual ~MacLowDcfListener ();
      /**
       * \param duration duration of NAV timer
       */
      virtual void NavStart (Time duration) = 0;
      /**
       * \param duration duration of NAV timer
       */
      virtual void NavReset (Time duration) = 0;
      virtual void AckTimeoutStart (Time duration) = 0;
      virtual void AckTimeoutReset () = 0;
      virtual void CtsTimeoutStart (Time duration) = 0;
      virtual void CtsTimeoutReset () = 0;
  };

  /**
   * \ingroup wifi
   * \brief listen for block ack events.
   */
  class MacLowBlockAckEventListener
  {
    public:
      MacLowBlockAckEventListener ();
      virtual ~MacLowBlockAckEventListener ();
      /**
       * Typically is called in order to notify EdcaTxopN that a block ack inactivity
       * timeout occurs for the block ack agreement identified by the pair <i>originator</i>, <i>tid</i>.
       *
       * Rx station maintains an inactivity timer for each block ack
       * agreement. Timer is reset when a frame with ack policy block ack
       * or a block ack request are received. When this timer reaches zero
       * this method is called and a delba frame is scheduled for transmission.
       */
      virtual void BlockAckInactivityTimeout (Mac48Address originator, uint8_t tid) = 0;
  };

  /**
   * \brief control how a packet is transmitted.
   * \ingroup wifi
   *
   * The ns3::MacLow::StartTransmission method expects
   * an instance of this class to describe how the packet
   * should be transmitted.
   */
  class MacLowTransmissionParameters
  {
    public:
      MacLowTransmissionParameters ();

      /**
       * Wait ACKTimeout for an ACK. If we get an ACK
       * on time, call MacLowTransmissionListener::GotAck.
       * Call MacLowTransmissionListener::MissedAck otherwise.
       */
      void EnableAck (void);
      /**
       *   - wait PIFS after end-of-tx. If idle, call
       *     MacLowTransmissionListener::MissedAck.
       *   - if busy at end-of-tx+PIFS, wait end-of-rx
       *   - if Ack ok at end-of-rx, call
       *     MacLowTransmissionListener::GotAck.
       *   - if Ack not ok at end-of-rx, report call
       *     MacLowTransmissionListener::MissedAck
       *     at end-of-rx+SIFS.
       *
       * This is really complicated but it is needed for
       * proper HCCA support.
       */
      void EnableFastAck (void);
      /**
       *  - if busy at end-of-tx+PIFS, call
       *    MacLowTransmissionListener::GotAck
       *  - if idle at end-of-tx+PIFS, call
       *    MacLowTransmissionListener::MissedAck
       */
      void EnableSuperFastAck (void);
      /**
       * Wait BASICBLOCKACKTimeout for a Basic Block Ack Response frame.
       */
      void EnableBasicBlockAck (void);
      /**
       * Wait COMPRESSEDBLOCKACKTimeout for a Compressed Block Ack Response frame.
       */
      void EnableCompressedBlockAck (void);
      /**
       * NOT IMPLEMENTED FOR NOW
       */
      void EnableMultiTidBlockAck (void);
      /**
       * Send a RTS, and wait CTSTimeout for a CTS. If we get a
       * CTS on time, call MacLowTransmissionListener::GotCts
       * and send data. Otherwise, call MacLowTransmissionListener::MissedCts
       * and do not send data.
       */
      void EnableRts (void);
      /**
       * \param size size of next data to send after current packet is
       *        sent.
       *
       * Add the transmission duration of the next data to the
       * durationId of the outgoing packet and call
       * MacLowTransmissionListener::StartNext at the end of
       * the current transmission + SIFS.
       */
      void EnableNextData (uint32_t size);

      /**
       * \param durationId the value to set in the duration/Id field of
       *        the outgoing packet.
       *
       * Ignore all other durationId calculation and simply force the
       * packet's durationId field to this value.
       */
      void EnableOverrideDurationId (Time durationId);

      /**
       * Do not wait for Ack after data transmission. Typically
       * used for Broadcast and multicast frames.
       */
      void DisableAck (void);
      /**
       * Do not send rts and wait for cts before sending data.
       */
      void DisableRts (void);
      /**
       * Do not attempt to send data burst after current transmission
       */
      void DisableNextData (void);
      /**
       * Do not force the duration/id field of the packet: its
       * value is automatically calculated by the MacLow before
       * calling WifiPhy::Send.
       */
      void DisableOverrideDurationId (void);

      /**
       * \returns true if must wait for ACK after data transmission,
       *          false otherwise.
       *
       * This methods returns true when any of MustWaitNormalAck,
       * MustWaitFastAck, or MustWaitSuperFastAck return true.
       */
      bool MustWaitAck (void) const;
      /**
       * \returns true if normal ACK protocol should be used, false
       *          otherwise.
       *
       * \sa EnableAck
       */
      bool MustWaitNormalAck (void) const;
      /**
       * \returns true if fast ack protocol should be used, false
       *          otherwise.
       *
       * \sa EnableFastAck
       */
      bool MustWaitFastAck (void) const;
      /**
       * \returns true if super fast ack protocol should be used, false
       *          otherwise.
       *
       * \sa EnableSuperFastAck
       */
      bool MustWaitSuperFastAck (void) const;
      /**
       * \returns true if block ack mechanism is used, false otherwise.
       *
       * \sa EnableBlockAck
       */
      bool MustWaitBasicBlockAck (void) const;
      /**
       * \returns true if compressed block ack mechanism is used, false otherwise.
       *
       * \sa EnableCompressedBlockAck
       */
      bool MustWaitCompressedBlockAck (void) const;
      /**
       * \returns true if multi-tid block ack mechanism is used, false otherwise.
       *
       * \sa EnableMultiTidBlockAck
       */
      bool MustWaitMultiTidBlockAck (void) const;
      /**
       * \returns true if RTS should be sent and CTS waited for before
       *          sending data, false otherwise.
       */
      bool MustSendRts (void) const;
      /**
       * \returns true if a duration/id was forced with
       *         EnableOverrideDurationId, false otherwise.
       */
      bool HasDurationId (void) const;
      /**
       * \returns the duration/id forced by EnableOverrideDurationId
       */
      Time GetDurationId (void) const;
      /**
       * \returns true if EnableNextData was called, false otherwise.
       */
      bool HasNextPacket (void) const;
      /**
       * \returns the size specified by EnableNextData.
       */
      uint32_t GetNextPacketSize (void) const;

    private:
      friend std::ostream &operator << (std::ostream &os, const MacLowTransmissionParameters &params);
      uint32_t m_nextSize;
      enum
      {
        ACK_NONE,
        ACK_NORMAL,
        ACK_FAST,
        ACK_SUPER_FAST,
        BLOCK_ACK_BASIC,
        BLOCK_ACK_COMPRESSED,
        BLOCK_ACK_MULTI_TID
      } m_waitAck;
      bool m_sendRts;
      Time m_overrideDurationId;
  };

  std::ostream &operator << (std::ostream &os, const MacLowTransmissionParameters &params);


  /**
   * \ingroup wifi
   * \brief handle RTS/CTS/DATA/ACK transactions.
   */
  class MacLow : public Object
  {
    public:
      typedef Callback<void, Ptr<Packet>, const WifiMacHeader*> MacLowRxCallback;
      typedef Callback<void> StartTxCallback;
      typedef Callback<bool> BooleanCallback;
      typedef Callback<void> VoidCallback;
      typedef Callback<void, WifiMacHeader, Ptr<const Packet> > SetPacketCallback;

      MacLow ();
      virtual ~MacLow ();

      void SetPhy (Ptr<WifiPhy> phy);
      void SetWifiRemoteStationManager (Ptr<WifiRemoteStationManager> manager);

      void SetAddress (Mac48Address ad);
      void SetAckTimeout (Time ackTimeout);
      void SetBasicBlockAckTimeout (Time blockAckTimeout);
      void SetCompressedBlockAckTimeout (Time blockAckTimeout);
      void SetCtsTimeout (Time ctsTimeout);
      void SetSifs (Time sifs);
      void SetSlotTime (Time slotTime);
      void SetPifs (Time pifs);
      void SetBssid (Mac48Address ad);
      void SetPromisc (void);
      Mac48Address GetAddress (void) const;
      Time GetAckTimeout (void) const;
      Time GetBasicBlockAckTimeout () const;
      Time GetCompressedBlockAckTimeout () const;
      Time GetCtsTimeout (void) const;
      Time GetSifs (void) const;
      Time GetSlotTime (void) const;
      Time GetPifs (void) const;
      Mac48Address GetBssid (void) const;

      /**
       * \param callback the callback which receives every incoming packet.
       *
       * This callback typically forwards incoming packets to
       * an instance of ns3::MacRxMiddle.
       */
      void SetRxCallback (Callback<void,Ptr<Packet>,const WifiMacHeader *> callback);

      /* After priority computation, if the node has the maximum priority, use this callback to start a tx
      */
      void SetStartTxCallback (StartTxCallback callback);
      void SetQueueEmptyCallback (BooleanCallback callback);
      void SetListenerCallback (VoidCallback callback);
      void SetDcaTxopPacketCallback (SetPacketCallback callback);
      /**
       * \param listener listen to NAV events for every incoming
       *        and outgoing packet.
       */
      void RegisterDcfListener (MacLowDcfListener *listener);

      /**
       * \param packet to send (does not include the 802.11 MAC header and checksum)
       * \param hdr header associated to the packet to send.
       * \param parameters transmission parameters of packet.
       *
       * This transmission time includes the time required for
       * the next packet transmission if one was selected.
       */
      Time CalculateTransmissionTime (Ptr<const Packet> packet,
          const WifiMacHeader* hdr,
          const MacLowTransmissionParameters& parameters) const;

      /**
       * \param packet packet to send
       * \param hdr 802.11 header for packet to send
       * \param parameters the transmission parameters to use for this packet.
       * \param listener listen to transmission events.
       *
       * Start the transmission of the input packet and notify the listener
       * of transmission events.
       */
      void StartTransmission (Ptr<const Packet> packet,
          const WifiMacHeader* hdr,
          MacLowTransmissionParameters parameters,
          MacLowTransmissionListener *listener);
      void SetMacLowTransmissionListener (MacLowTransmissionListener *listener );

      /**
       * \param packet packet received
       * \param rxSnr snr of packet received
       * \param txMode transmission mode of packet received
       * \param preamble type of preamble used for the packet received
       *
       * This method is typically invoked by the lower PHY layer to notify
       * the MAC layer that a packet was successfully received.
       */
      void ReceiveOk (Ptr<Packet> packet, double rxSnr, WifiMode txMode, WifiPreamble preamble);
      /**
       * \param packet packet received.
       * \param rxSnr snr of packet received.
       *
       * This method is typically invoked by the lower PHY layer to notify
       * the MAC layer that a packet was unsuccessfully received.
       */
      void ReceiveError (Ptr<const Packet> packet, double rxSnr);
      /**
       * \param duration switching delay duration.
       *
       * This method is typically invoked by the PhyMacLowListener to notify
       * the MAC layer that a channel switching occured. When a channel switching
       * occurs, pending MAC transmissions (RTS, CTS, DATA and ACK) are cancelled.
       */
      void NotifySwitchingStartNow (Time duration);
      /**
       * \param respHdr Add block ack response from originator (action
       * frame).
       * \param originator Address of peer station involved in block ack
       * mechanism.
       * \param startingSeq Sequence number of the first MPDU of all
       * packets for which block ack was negotiated.
       *
       * This function is typically invoked only by ns3::RegularWifiMac
       * when the STA (which may be non-AP in ESS, or in an IBSS) has
       * received an ADDBA Request frame and is transmitting an ADDBA
       * Response frame. At this point MacLow must allocate buffers to
       * collect all correctly received packets belonging to the category
       * for which Block Ack was negotiated.
       */
      void CreateBlockAckAgreement (const MgtAddBaResponseHeader *respHdr,
          Mac48Address originator,
          uint16_t startingSeq);
      /**
       * \param originator Address of peer participating in Block Ack mechanism.
       * \param tid TID for which Block Ack was created.
       *
       * Checks if exists an established block ack agreement with <i>originator</i>
       * for tid <i>tid</i>. If the agreement exists, tears down it. This function is typically
       * invoked when a DELBA frame is received from <i>originator</i>.
       */
      void DestroyBlockAckAgreement (Mac48Address originator, uint8_t tid);
      /**
       * \param ac Access class managed by the queue.
       * \param listener The listener for the queue.
       *
       * The lifetime of the registered listener is typically equal to the lifetime of the queue
       * associated to this AC.
       */
      void RegisterBlockAckListenerForAc (enum AcIndex ac, MacLowBlockAckEventListener *listener);

      /* The @sender and @receiver is from the perspective of DATA link. We update the received sequence number and check 
       * if it is time to update er.edge.interferenceW
       */
      void UpdateReceivedDataPacketNumbers (Mac48Address sender, Mac48Address receiver, uint16_t seqNo);

      /* The @sender and @receiver is from the perspective of DATA link.
      */
      void UpdateReceivedAckPacketNumbers (Mac48Address sender, Mac48Address receiver, uint16_t seqNo);
      void AddOrUpdateErRegion (LinkMetaData item);

      /* Identify the link according to the sender and receiver (Notice that this sender and receiver is not from the perspective
       * of a DATA link, therefore, the sender of the ACK link is actually the receiver of the DATA link).
       * The purpose of this method is to record the computed interferenceDb and the N+I at both the current estimation time and the
       * last estimation time. The minimum variance controller need this value to compute the \Delta Interference
       */
      LinkMetaData GetLinkMetaDataByLink (Mac48Address receiver, Mac48Address sender) const;

      /* This method is invoked at the very beginning of each timeslot after the learning process. This method calculates the priority of 
       * each link at the current timeslot. According to the calculation, if one of the links that is related to the current node has the 
       * maximum priority value, the node status is set to be true (active), and the channel number is set to be 1 (data channel), and after
       * 2 microseconds (switching delay), the node will send or receive DATA packet.
       * However, if the node failed to have the change to be set as active, the node stays in channel 2(control channel). The node checks if 
       * the payload for control packet is empty or not, if not, this means this node needs to send control packet to inform others about its
       * status change regarding er.edge.interferenceW. After a random time of backoff, we sense the channel, and send the packet if the channel
       * is idle.
       */
      void CalcPriority (); 
      int64_t DoCalculatePriority (int64_t linkId, int64_t timeslot);
      /* When succeeded in priority calculation, the node status is set to be true, meaning active. An active has the previllege of sending
       * DATA or receiving DATA.
       */
      void SetNodeActiveFalse ();
      bool GetNodeActiveStatus () const;

      // By calculating the priority, the sender could decide if the link initialed by the sender could succeed in the priority calculation
      // If the link wins, that means the link can transmit in this timeslot, we should set the node status (active  or not) according to the
      // value returned by this method.
      bool SenderComputeThePriority (std::string addr);

      template <typename T>
        static std::string ToString (T const &val);

      /* This method parse the paylod of the control packet. Basically, the control packet is also of data type. however, we have put some
       * meaningful data in the payload:
       * sender address (sender of the DATA link)
       * followed by a ' ' (space)
       * receiver address (receiver of the DATA link)
       * followed by a ' '
       * flag identifying whether the er.edge.interferenceW is for ACK link or DATA link.
       * er.edge.interferenceW. 
       * (Since we view sender and receiver from the perspective of a DATA link, if the flag indicates the er.edge.interferenceW
       *  is a DATA link, that means, we should update the receiver's DATA er.edge.interferenceW; on the other hand, if the flag indicates the er.edge.
       *  interferenceW is an ACK link, we should update the sender's ACK er.edge.interferenceW.)
       */
      Payload ParseControlPacketPayload (uint8_t* buffer, WifiMacHeader hdr);

      /* This method is invoked when a node finds out his ER edge interferenceW regarding one of the links that is related to this node
       * has changed. We create the the wifi-mac-header, read the payload and do carrier sensing, if the channel is idle, send the packet
       */
      void TrySendControlPacket ();

      void ScheduleControlSignalTransmission ();

      void SenseChannelLater ();

      void GenerateDataPacketAndSend ();

      ErInfoItem GetErInfoItem (uint16_t sender, uint16_t receiver, bool isDataEr, bool isSelf);

      void UpdateControlInformation (ErInfoItem payload);

      std::string IntToMacAddress (uint16_t nodeId);

      void UpdateMaxEr (NodeMaxErItem item);

      double GetMaxErEdgeInterference () const;


      uint16_t FindD0Quantile (Mac48Address address);

      uint16_t FindMaxD0AsD0Prime (Mac48Address sender, Mac48Address receiver);
      uint16_t FindD0ByNeighbor (Mac48Address sender, Mac48Address receiver, Mac48Address estimatingNode, bool isDataEr);

      std::vector<ErInfoItem> SelectErInfoItemsToTransmit (uint32_t totalSize ) ;
      ErInfoItem GenerateDefaultErInfoItem (uint16_t sender, uint16_t receiver, bool isDataEr);
      LinkEstimatorItem GenerateLinkEstimatorItem (Mac48Address sender, Mac48Address receiver);
      uint32_t FindInfoItemCategory (Mac48Address orginalSender, Mac48Address currentSender, Mac48Address currentReceiver, double edgeW);

      uint8_t* GenerateControlPayload (uint32_t maxSize, uint32_t itemSize, uint8_t payload[]);
      void ProcessControlPayload (uint8_t buffer[], WifiMacHeader hdr);

      double FindMaxErEdge (Mac48Address address) const;
      bool CheckSenderInTxEr(Mac48Address address);

      double FindQuantileValue (double quantile, std::vector<double> &vec);

      bool CompareErInfoItem ( ErInfoItem a, ErInfoItem b);

      //----------------------------QUICK SORT --------------------------------------

      void quick_sort(std::vector<ErInfoItem> *sortArray);
      void quick_sort(std::vector<ErInfoItem> *sortArray, int startIndex, int endIndex);
      int Partition(std::vector<ErInfoItem> *sortArray, int startIndex, int endIndex);
      void Swap( int l, int k);
      uint32_t BinarySearch (std::vector<uint32_t> &vec, uint32_t key);

      void CopyErInfoItem (ErInfoItem *a, ErInfoItem *b);
      void CopyErInfoItem (std::vector<ErInfoItem>::iterator it, ErInfoItem *b);
      ErInfoItem BinarySearchErInfoItem (uint32_t itemId);
      void SetInitialEr (uint16_t sender, uint16_t receiver,  double initialErW);
      void UpdateNodeTxProbability ( NodesTxProbability item );
      double GetNodeTxProbability ( uint16_t nodeId ) const;
      void GeneratePacket ();
      void CollectConfilictingLinks ( std::vector<int64_t> &vec, Mac48Address sender);
      void InitiateNextTxSlotInfo  ();
      NextTxSlotInfo GetNextTxSlot (int64_t linkId);
      void UpdateNextRxSlot (int64_t linkId, int64_t nextRxSlot, bool fromSender);
      void UpdateConservativeStatus (int64_t linkId, bool beConservative);
      void ClearNextRxSlot (int64_t linkId);
      void InitiateErRxStatus  ();
      bool GetErRxStatus (int64_t linkId);
      void UpdateErRxStatus (int64_t linkId, bool receptionStatus);
      void IncreaseSlotCount ();
      void ResetPriorityForErItem (int64_t linkId);
      std::vector<NextRxSlotInfo> CalcNextRxSlotAsReceiver ();
      int64_t ComputeNextRxSlot (Mac48Address sender);
      void CsmaSchedule ();
#if defined(SCREAM)
      void CalcScreamSchedule ();
      void InitiateRemainNodes ();
      void TrySendProbePacket ();
      void CheckAndSendProbeMsg ();
      void SwithChannelForScream ();
      void ScreamNormalDataTransmission ();
      uint16_t GetFromPosition (uint16_t pos);
#endif
      void SetPdrRequirement70 ();
      void SetPdrRequirement80 ();
      void SetPdrRequirement90 ();
      void SetPdrRequirement95 ();
      void IncrementReTransmissionTimes ();
      bool IsDuplicatePacket (Ptr<Packet> s);

      /****************************************** PRIVATE *********************************************/
    private:
      void CancelAllEvents (void);
      uint32_t GetAckSize (void) const;
      uint32_t GetBlockAckSize (enum BlockAckType type) const;
      uint32_t GetRtsSize (void) const;
      uint32_t GetCtsSize (void) const;
      uint32_t GetSize (Ptr<const Packet> packet, const WifiMacHeader *hdr) const;
      Time NowUs (void) const;
      void ForwardDown (Ptr<const Packet> packet, const WifiMacHeader *hdr,
          WifiMode txMode);
      Time CalculateOverallTxTime (Ptr<const Packet> packet,
          const WifiMacHeader* hdr,
          const MacLowTransmissionParameters &params) const;
      WifiMode GetRtsTxMode (Ptr<const Packet> packet, const WifiMacHeader *hdr) const;
      WifiMode GetDataTxMode (Ptr<const Packet> packet, const WifiMacHeader *hdr) const;
      WifiMode GetCtsTxModeForRts (Mac48Address to, WifiMode rtsTxMode) const;
      WifiMode GetAckTxModeForData (Mac48Address to, WifiMode dataTxMode) const;

      Time GetCtsDuration (Mac48Address to, WifiMode rtsTxMode) const;
      Time GetAckDuration (Mac48Address to, WifiMode dataTxMode) const;
      Time GetBlockAckDuration (Mac48Address to, WifiMode blockAckReqTxMode, enum BlockAckType type) const;
      void NotifyNav (const WifiMacHeader &hdr, WifiMode txMode, WifiPreamble preamble);
      void DoNavResetNow (Time duration);
      bool DoNavStartNow (Time duration);
      bool IsNavZero (void) const;
      void NotifyAckTimeoutStartNow (Time duration);
      void NotifyAckTimeoutResetNow ();
      void NotifyCtsTimeoutStartNow (Time duration);
      void NotifyCtsTimeoutResetNow ();
      void MaybeCancelPrevious (void);

      void NavCounterResetCtsMissed (Time rtsEndRxTime);
      void NormalAckTimeout (void);
      void FastAckTimeout (void);
      void SuperFastAckTimeout (void);
      void FastAckFailedTimeout (void);
      void BlockAckTimeout (void);
      void CtsTimeout (void);
      void SendCtsAfterRts (Mac48Address source, Time duration, WifiMode txMode, double rtsSnr);
      void SendAckAfterData (Mac48Address source, Time duration, WifiMode txMode, double rtsSnr);
      void SendDataAfterCts (Mac48Address source, Time duration, WifiMode txMode);
      void WaitSifsAfterEndTx (void);

      void SendRtsForPacket (void);
      void SendDataPacket (void);
      void SendCurrentTxPacket (void);
      void StartDataTxTimers (void);
      virtual void DoDispose (void);

      /* The @sender and @receiver is from the perspective of a DATA link
      */
      LinkEstimatorItem GetEstimatorTableItemByNeighborAddr (Mac48Address sender, Mac48Address receiver );


      std::vector<double> GetErInforItemForLink (Mac48Address sender, Mac48Address receiver);

      /* param estimatorItem The item in the estimator table that need to update. We identify the item according to the
       * neighbor address in this item.
       */
      void UpdateEstimatorTableItem (LinkEstimatorItem estimatorItem);


      void UpdateReceivedErInfoItem (ErInfoItem erItem,  Mac48Address controlPacketSender);

      /* after received a control packet, update the er.edge.interferneceW according to the received control packet
      */
      void UpdateEstimatorTableItem (ErInfoItem payload, bool isFirstPayloadItem, Mac48Address controlPacketSender);
      /**
       * \param originator Address of peer participating in Block Ack mechanism.
       * \param tid TID for which Block Ack was created.
       * \param seq Starting sequence
       *
       * This function forward up all completed "old" packets with sequence number
       * smaller than <i>seq</i>. All comparison are performed circularly mod 4096.
       */
      void RxCompleteBufferedPacketsWithSmallerSequence (uint16_t seq, Mac48Address originator, uint8_t tid);
      /**
       * \param originator Address of peer participating in Block Ack mechanism.
       * \param tid TID for which Block Ack was created.
       *
       * This method is typically invoked when a MPDU with ack policy
       * subfield set to Normal Ack is received and a block ack agreement
       * for that packet exists.
       * This happens when the originator of block ack has only few MPDUs to send.
       * All completed MSDUs starting with starting sequence number of block ack
       * agreement are forward up to WifiMac until there is an incomplete or missing MSDU.
       * See section 9.10.4 in IEEE802.11 standard for more details.
       */
      void RxCompleteBufferedPacketsUntilFirstLost (Mac48Address originator, uint8_t tid);
      /*
       * This method checks if exists a valid established block ack agreement.
       * If there is, store the packet without pass it up to WifiMac. The packet is buffered
       * in order of increasing sequence control field. All comparison are performed
       * circularly modulo 2^12.
       */
      bool StoreMpduIfNeeded (Ptr<Packet> packet, WifiMacHeader hdr);
      /*
       * Invoked after that a block ack request has been received. Looks for corresponding
       * block ack agreement and creates block ack bitmap on a received packets basis.
       */
      void SendBlockAckAfterBlockAckRequest (const CtrlBAckRequestHeader reqHdr, Mac48Address originator,
          Time duration, WifiMode blockAckReqTxMode);
      /*
       * This method creates block ack frame with header equals to <i>blockAck</i> and start its transmission.
       */
      void SendBlockAckResponse (const CtrlBAckResponseHeader* blockAck, Mac48Address originator, bool immediate,
          Time duration, WifiMode blockAckReqTxMode);
      /*
       * Every time that a block ack request or a packet with ack policy equals to <i>block ack</i>
       * are received, if a relative block ack agreement exists and the value of inactivity timeout
       * is not 0, the timer is reset.
       * see section 11.5.3 in IEEE802.11e for more details.
       */
      void ResetBlockAckInactivityTimerIfNeeded (BlockAckAgreement &agreement);

      void SetupPhyMacLowListener (Ptr<WifiPhy> phy);


      double m_ewmaCoefficient;
      Ptr<WifiPhy> m_phy;
      Ptr<WifiRemoteStationManager> m_stationManager;
      MacLowRxCallback m_rxCallback;
      StartTxCallback m_startTxCallback;
      BooleanCallback m_queueEmptyCallback;
      VoidCallback m_setLisenterCallback;
      SetPacketCallback m_setPacketCallback;
      typedef std::vector<MacLowDcfListener *>::const_iterator DcfListenersCI;
      typedef std::vector<MacLowDcfListener *> DcfListeners;
      DcfListeners m_dcfListeners;

      EventId m_normalAckTimeoutEvent;
      EventId m_fastAckTimeoutEvent;
      EventId m_superFastAckTimeoutEvent;
      EventId m_fastAckFailedTimeoutEvent;
      EventId m_blockAckTimeoutEvent;
      EventId m_ctsTimeoutEvent;
      EventId m_sendCtsEvent;
      EventId m_sendAckEvent;
      EventId m_sendDataEvent;
      EventId m_waitSifsEvent;
      EventId m_navCounterResetCtsMissed;
      EventId m_ctsLateEvent;
      EventId m_dataLateEvent;
      EventId m_ackLateEvent;
      EventId m_dataCorrectlyReceivedEvent;

      Ptr<Packet> m_currentPacket;
      WifiMacHeader m_currentHdr;
      MacLowTransmissionParameters m_txParams;
      MacLowTransmissionListener *m_listener;
      Mac48Address m_self;
      Mac48Address m_bssid;
      Time m_ackTimeout;
      Time m_basicBlockAckTimeout;
      Time m_compressedBlockAckTimeout;
      Time m_ctsTimeout;
      Time m_sifs;
      Time m_slotTime;
      Time m_pifs;
      std::vector<LinkMetaData> m_linkMetaData;

      int64_t m_nodesCountUpperBound;

      MathHelper m_mathHelper;
      Time m_lastNavStart;
      Time m_lastNavDuration;
      uint16_t m_estimatorWindow;
      std::vector<LinkEstimatorItem> m_linkEstimatorTable;
      //the controller from which we get the delta interference power.
      PControllerWithDesiredPdr m_pControllerWithDesiredPdr;
      PControllerWithReferenceInterference m_pControllerWithReferenceInterference;
      MinimumVarianceController m_minVarController;
      //the desired Pdr we want. // data pdr
      double m_desiredDataPdr;
      double m_ackPdr; // intuitively, the ack pdr should be the higher the better.
      bool m_promisc;
      // Listerner needed to monitor when a channel switching occurs.
      class PhyMacLowListener * m_phyMacLowListener;
      //uint32_t m_sequenceVectorCapacity;
#if defined(SCREAM)
      int16_t m_consideredNodeId;
      std::vector<uint16_t> m_remainNodes;
#endif
#ifdef CSMA_PRKS_HYBRID
      std::vector<uint16_t> m_csmaNodes;
      bool m_runPRKS;
#endif
      //----------------------------TDMA TDMA TDMA ---------------------------------------------------
      Time m_timeslot;
      bool m_nodeActive;
      std::string m_controlPacketPayload;
      uint32_t m_currentTimeslot;
      uint32_t m_erInfoUpdatedTimeSlot;
      Mac48Address m_dataReceiverAddr; // generate packet at the MAC layer to avoid wasting slots
      double m_initialErEdgeInterferenceW;
      int32_t m_controlMessagePriority;
      int32_t m_defaultPriority;
      std::vector<ErInfoItem> m_controlInformation; // this is for the node itself
      std::vector<ErInfoItem> m_othersControlInformation; // this is for the node itself
      ErInfoItem m_othersControlInformationCopy[INFO_ITEM_SUM]; // this is for the node itself
      std::vector<ErInfoItem> *sortingArray;
      uint32_t m_sendProbLastComputedTimeSlot;
      uint32_t m_sendingCount;
      double m_selfSendingProbability;
      std::vector<NodesTxProbability> m_nodesTxProbability;
      uint32_t m_conflictingSetSize;
      //Ptr<WifiMacQueue> m_packetQueue;
      std::vector<Ptr<Packet> > m_packetQueue;
      UniformVariable m_uniform;
      double m_packetGenreationProbability;
      //========================NEW DESIGN================================================
      std::vector<int64_t> m_conflictingSet;
      int64_t m_nextSendingSlot;
      std::vector<NextTxSlotInfo> m_nextTxSlotInfo;
      std::vector<NewErRxStatus> m_newErRxStatus;
      uint32_t m_newErEdgeReceivedFromReceiver; //0: not received; 1: received ER from receiver; 2: send info back to RX; 3: ACK received again, denoting RX received the Info.
      uint32_t m_retransmissionTimes;
#if defined(CONVERGECAST)
      EventId m_incrementReTxTimesEvent;
      std::vector<int16_t> m_linkParent;
      std::vector<double> m_linkRequirement;
#endif


      /************************POWER CONTROL in TDMA **************************************************/
      uint32_t m_maxBiDirectionalErChangeInformTimes; 
      uint32_t m_defaultInformTimes;
      double m_informingRange;
      double m_previousSendingPower;
      std::vector<NodeMaxErItem> m_maxErs;
      /************************************************************************************************/

      /*
       * BlockAck data structures.
       */
      typedef std::pair<Ptr<Packet>, WifiMacHeader> BufferedPacket;
      typedef std::list<BufferedPacket>::iterator BufferedPacketI;

      typedef std::pair<Mac48Address, uint8_t> AgreementKey;
      typedef std::pair<BlockAckAgreement, std::list<BufferedPacket> > AgreementValue;

      typedef std::map<AgreementKey, AgreementValue> Agreements;
      typedef std::map<AgreementKey, AgreementValue>::iterator AgreementsI;

      typedef std::map<AgreementKey, BlockAckCache> BlockAckCaches;
      typedef std::map<AgreementKey, BlockAckCache>::iterator BlockAckCachesI;

      Agreements m_bAckAgreements;
      BlockAckCaches m_bAckCaches;

      typedef std::map<AcIndex, MacLowBlockAckEventListener*> QueueListeners;
      QueueListeners m_edcaListeners;
      struct LinkMetaDataCompare {
        bool operator () (LinkMetaData a, LinkMetaData b) { return a.edgeInterferenceW < b.edgeInterferenceW; }
      } LinkMetaDataCompare;
      struct MaxErsCompare {
        bool operator () (NodeMaxErItem a, NodeMaxErItem b) { return a.erEdgeInterferenceW < b.erEdgeInterferenceW; }
      } MaxErsCompare;


      struct ErInfoItemCompare {
        bool operator () (ErInfoItem a, ErInfoItem b) 
        {
          return a.itemPriority > b.itemPriority;
        }
      } ErInfoItemCompare;

  };

  class SnrTag : public Tag
  {
    public:
      static TypeId GetTypeId (void);
      virtual TypeId GetInstanceTypeId (void) const;

      virtual uint32_t GetSerializedSize (void) const;
      virtual void Serialize (TagBuffer i) const;
      virtual void Deserialize (TagBuffer i);
      virtual void Print (std::ostream &os) const;

      void Set (double snr);
      double Get (void) const;
    private:
      double m_snr;
  };

} // namespace ns3

#endif /* MAC_LOW_H */
