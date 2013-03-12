/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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
 */
#include "ns3/core-config.h"
#include "simulator.h"
#include "simulator-impl.h"
#include "scheduler.h"
#include "map-scheduler.h"
#include "event-impl.h"

#include "ptr.h"
#include "string.h"
#include "object-factory.h"
#include "global-value.h"
#include "assert.h"
#include "log.h"

#include <math.h>
#include <fstream>
#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>

NS_LOG_COMPONENT_DEFINE ("Simulator");

namespace ns3 {

  GlobalValue g_simTypeImpl = GlobalValue ("SimulatorImplementationType", 
      "The object class to use as the simulator implementation",
      StringValue ("ns3::DefaultSimulatorImpl"),
      MakeStringChecker ());

  GlobalValue g_schedTypeImpl = GlobalValue ("SchedulerType", 
      "The object class to use as the scheduler implementation",
      TypeIdValue (MapScheduler::GetTypeId ()),
      MakeTypeIdChecker ());


  static void
    TimePrinter (std::ostream &os)
    {
      os << Simulator::Now ().GetSeconds () << "s";
    }

  static void
    NodePrinter (std::ostream &os)
    {
      if (Simulator::GetContext () == 0xffffffff)
      {
        os << "-1";
      }
      else
      {
        os << Simulator::GetContext ();
      }
    }

  static SimulatorImpl **PeekImpl (void)
  {
    static SimulatorImpl *impl = 0;
    return &impl;
  }

  static SimulatorImpl * GetImpl (void)
  {
    SimulatorImpl **pimpl = PeekImpl ();
    /* Please, don't include any calls to logging macros in this function
     * or pay the price, that is, stack explosions.
     */
    if (*pimpl == 0)
    {
      {
        ObjectFactory factory;
        StringValue s;

        g_simTypeImpl.GetValue (s);
        factory.SetTypeId (s.Get ());
        *pimpl = GetPointer (factory.Create<SimulatorImpl> ());
      }
      {
        ObjectFactory factory;
        StringValue s;
        g_schedTypeImpl.GetValue (s);
        factory.SetTypeId (s.Get ());
        (*pimpl)->SetScheduler (factory);
      }

      //
      // Note: we call LogSetTimePrinter _after_ creating the implementation
      // object because the act of creation can trigger calls to the logging 
      // framework which would call the TimePrinter function which would call 
      // Simulator::Now which would call Simulator::GetImpl, and, thus, get us 
      // in an infinite recursion until the stack explodes.
      //
      LogSetTimePrinter (&TimePrinter);
      LogSetNodePrinter (&NodePrinter);
    }
    return *pimpl;
  }

  Time Simulator::LearningTimeDuration = Seconds (LEARNING_PROCESS_DURATION);
  uint32_t Simulator::MinInformRange = MIN_INFORM_RANGE;
  Time Simulator::SimulationStopTime = Seconds (SIMULATION_STOP_TIME);
  int16_t Simulator::NodesCountUpperBound = NODE_COUNT_UPPER_BOUND;
  std::vector<TdmaLink> Simulator::m_tdmaLinks;
  std::vector<NodeSignalMap> Simulator::m_signalMaps;
  std::vector<std::string> Simulator::NodesWillSend;
  std::vector<ControlReliability> Simulator::m_controlReliabilityCollec;
  std::vector<std::string> Simulator::m_nodesInDataChannel;
  bool Simulator::m_linksClassified = false;
  NodeLinkDetails Simulator::m_nodeLinkDetails[NODE_COUNT_UPPER_BOUND];

  Time Simulator::SlotBeginningTime = Seconds (0);



  void Simulator::ClearSendingNodes ()
  {
    NodesWillSend.clear ();
  }


  void Simulator::SenderRegisterControlReliability (std::string sender, std::vector<std::string> idealReceivers) // actual receiver count: 0
  {
    ControlReliability controlReliability;
    controlReliability.sender = sender;
    controlReliability.idealReceivers = idealReceivers;
    controlReliability.actualReceiverCount = 0;
    m_controlReliabilityCollec.push_back (controlReliability);
  }
  void Simulator::ReceiverRegisterControlReliability (std::string sender, std::string receiver) // count ++
  {
    for (std::vector<ControlReliability>::iterator it = m_controlReliabilityCollec.begin (); it != m_controlReliabilityCollec.end (); ++ it)
    {
      if ( it->sender == sender)
      {
        if (find (it->idealReceivers.begin (), it->idealReceivers.end (), receiver) != it->idealReceivers.end ())
        {
          it->actualReceiverCount += 1;
        }
      }
    }
  }
  void Simulator::CountFinalControlReliability ()
  {
    uint32_t idealCount = 0, actualCount = 0;
    for (std::vector<ControlReliability>::iterator it = m_controlReliabilityCollec.begin (); it != m_controlReliabilityCollec.end (); ++ it)
    {
      for (std::vector<std::string>::iterator ii = it->idealReceivers.begin (); ii != it->idealReceivers.end (); ++ ii)
      {
        if ( find (m_nodesInDataChannel.begin (), m_nodesInDataChannel.end (), *ii) == m_nodesInDataChannel.end ())
        {
          idealCount ++;
        }
      }
      //idealCount += it->idealReceivers.size ();
      actualCount += it->actualReceiverCount;
    }
    std::cout<<" idealReceiverCount: "<< idealCount <<" actualReceiverCount: "<< actualCount << std::endl;
    m_controlReliabilityCollec.clear ();
  }

  void Simulator::ResigerSendingNode (std::string sender, double senderEdgeInterferenceW, std::string receiver, double receiverEdgeInterferenceW)
  {
    if ( find (NodesWillSend.begin (), NodesWillSend.end (), sender) == NodesWillSend.end ())
    {
      NodesWillSend.push_back (sender);
    }
  }

  void Simulator::AddTdmaLink (TdmaLink link)
  {
    for (std::vector<TdmaLink>::iterator _it = m_tdmaLinks.begin (); _it != m_tdmaLinks.end (); ++ _it)
    {
      // if this link has already been registered in the vector, we stop doing this again.
      if ( _it->senderAddr == link.senderAddr && _it->receiverAddr == link.receiverAddr )
      {
        return;
      }
    }
    // if the link is not registered in this vector, register it 
    m_tdmaLinks.push_back (link);
    //Simulator::PrintLinks ();
  }

  /* Find the link information (senderAddr, receiverAddr, linkId) from the variable m_tdmaLinks and return it
  */
  TdmaLink Simulator::FindLinkBySender (std::string senderAddr )
  {
    for (std::vector<TdmaLink>::iterator _it = m_tdmaLinks.begin (); _it != m_tdmaLinks.end (); ++ _it)
    {
      if (_it->senderAddr == senderAddr )
      {
        return *_it;
      }
    }
    TdmaLink linkInfo;
    linkInfo.senderAddr = "";
    linkInfo.receiverAddr = "";
    linkInfo.linkId = 0;
    return linkInfo;
  }

  /* Find all the links that are related to the node (@addr), no matter this node is acting a a sender or receiver
  */
  std::vector<TdmaLink> Simulator::FindRelatedLinks (std::string addr)
  {
    std::vector<TdmaLink> relatedLinks;
    for (std::vector<TdmaLink>::iterator _it = m_tdmaLinks.begin (); _it != m_tdmaLinks.end (); ++ _it)
    {
      if (_it->senderAddr == addr || _it->receiverAddr == addr)
      {
        relatedLinks.push_back (*_it);
      }
    }
    return relatedLinks;
  }

  std::vector<std::string> Simulator::ListNodesInEr (std::string nodeAddr, double nodeErEdgeInterferenceW)
  {
    std::vector<std::string> neighbors;
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if (it->selfAddress == nodeAddr )
      {
        for (std::vector<TdmaSignalMap>::iterator _it = it->signalMap.begin (); _it != it->signalMap.end (); ++ _it)
        {
          if (_it->supposedInterferenceW >= nodeErEdgeInterferenceW)
          {
            neighbors.push_back (_it->from);
          }
        }
      }
    }

    if ( find (neighbors.begin (), neighbors.end (), nodeAddr) == neighbors.end ())
    {
      neighbors.push_back (nodeAddr);
    }
    return neighbors;
  }

  std::vector<std::string> Simulator::ListNodesInEr (std::string nodeAddr)
  {
    std::vector<std::string> neighbors;
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if (it->selfAddress == nodeAddr )
      {
        for (std::vector<TdmaSignalMap>::iterator _it = it->signalMap.begin (); _it != it->signalMap.end (); ++ _it)
        {
          neighbors.push_back (_it->from);
          if (neighbors.size () == MinInformRange)
          {
            break;
          }
        }
      }
    }

    if ( find (neighbors.begin (), neighbors.end (), nodeAddr) == neighbors.end ())
    {
      neighbors.push_back (nodeAddr);
    }
    return neighbors;
  }

  std::vector<std::string> Simulator::ListNodesInEr (std::string senderAddr, double senderErEdgeInterferenceW, std::string receiverAddr, double receiverErEdgeInterferenceW)
  {
    std::vector<std::string> neighbors;
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if ( it->selfAddress == senderAddr )
      {
        for (std::vector<TdmaSignalMap>::iterator _it = it->signalMap.begin (); _it != it->signalMap.end (); ++ _it)
        {
          if (_it->supposedInterferenceW >= senderErEdgeInterferenceW )
          {
            neighbors.push_back (_it->from);
          }
        }
        break;
      }
    }
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if ( it->selfAddress == receiverAddr )
      {
        for (std::vector<TdmaSignalMap>::iterator _it = it->signalMap.begin (); _it != it->signalMap.end (); ++ _it)
        {
          if (_it->supposedInterferenceW >= receiverErEdgeInterferenceW && find (neighbors.begin (), neighbors.end (), _it->from) == neighbors.end ())
          {
            neighbors.push_back (_it->from);
          }
        }
        break;
      }
    }
    if ( find (neighbors.begin (), neighbors.end (), senderAddr) == neighbors.end ())
    {
      neighbors.push_back (senderAddr);
    }
    if (find (neighbors.begin (), neighbors.end (), receiverAddr ) == neighbors.end ())
    {
      neighbors.push_back (receiverAddr);
    }
    return neighbors;
  }

  void Simulator::AddSignalMapItem (std::string selfAddr, std::string neighborAddr, double outBoundAttenuation, double inBoundAttenuation, double outSinr, double inSinr, double noisePlusInterferenceW, double supposedInterferenceW)
  {
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if ( it->selfAddress == selfAddr )
      {
        //std::cout<<" signalmapsize: "<< it->signalMap.size () << std::endl;
        for (std::vector<TdmaSignalMap>::iterator _it = it->signalMap.begin (); _it != it->signalMap.end (); ++ _it)
        {
          //if there is a record regarding this neighbor, just update it and exit the method.
          if ( _it->from == neighborAddr )
          {
            _it->outBoundAttenuation = outBoundAttenuation;
            _it->inBoundAttenuation = inBoundAttenuation;
            _it->outSinr = outSinr;
            _it->inSinr = inSinr;
            _it->noisePlusInterferenceW = noisePlusInterferenceW;
            _it->supposedInterferenceW = supposedInterferenceW;
            return;
          }
          // if there is no record regarding this neighbor, insert the beighbor's information into the vector
        }
        TdmaSignalMap item;
        item.from = neighborAddr;
        item.outBoundAttenuation = outBoundAttenuation;
        item.inBoundAttenuation = inBoundAttenuation;
        item.inSinr = inSinr;
        item.outSinr = outSinr;
        item.noisePlusInterferenceW = noisePlusInterferenceW;
        item.supposedInterferenceW = supposedInterferenceW;
        it->signalMap.push_back (item);
        return;
      }
    }

    // * if there is even no record regarding this node (@selfAddress), create a completely new entry for this node and 
    // store its signalMap information to the vector
    NodeSignalMap nodeSignalMap;
    nodeSignalMap.selfAddress = selfAddr;
    std::vector<TdmaSignalMap> signalMap;
    nodeSignalMap.signalMap = signalMap;

    TdmaSignalMap item;
    item.from = neighborAddr;
    item.outBoundAttenuation = outBoundAttenuation;
    item.inBoundAttenuation = inBoundAttenuation;
    item.inSinr = inSinr;
    item.outSinr = outSinr;
    item.noisePlusInterferenceW = noisePlusInterferenceW;
    item.supposedInterferenceW = supposedInterferenceW;
    nodeSignalMap.signalMap.push_back (item);
    m_signalMaps.push_back (nodeSignalMap);
  }

  std::vector<TdmaLink> Simulator::ListAllLinks ()
  {
    return m_tdmaLinks;
  }

  void Simulator::PrintLinks ()
  {
    std::cout<<std::endl<<" print links (count): "<< m_tdmaLinks.size ()<< std::endl;
    for (std::vector<TdmaLink>::iterator _it = m_tdmaLinks.begin (); _it != m_tdmaLinks.end (); ++ _it)
    {
      std::cout<<" sender.addr: "<< _it->senderAddr <<" receiver.addr: "<< _it->receiverAddr <<" link.id: "<< _it->linkId << std::endl;
    }
  }

  std::vector<TdmaSignalMap> Simulator::GetNodeSignalMap (std::string addr)
  {
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if ( it->selfAddress == addr )
      {
        return it->signalMap;
      }
    }
    std::vector<TdmaSignalMap> vec;
    return vec;
  }

  void Simulator::PrintSignalMap (std::string addr)
  {
    std::cout<<" signalmaps.size: "<<  m_signalMaps.size () << std::endl;
    for (std::vector<NodeSignalMap>::iterator it = m_signalMaps.begin (); it != m_signalMaps.end (); ++ it)
    {
      if ( it->selfAddress == addr )
      {
        std::cout<<std::endl<<" signalMap for address: "<< it->selfAddress << std::endl;
        for (std::vector<TdmaSignalMap>::iterator _it = it->signalMap.begin (); _it != it->signalMap.end (); ++ _it)
        {
          std::cout<<" from: "<<_it->from <<" outbound: "<< _it->outBoundAttenuation <<" inbound: "<<_it->inBoundAttenuation<<" outSinr: "
            <<_it->outSinr <<" inSinr: "<<_it->inSinr <<" supposedInterferenceW: " <<_it->supposedInterferenceW <<std::endl;
        }
      }
    }
  }
  void
    Simulator::Destroy (void)
    {
      NS_LOG_FUNCTION_NOARGS ();

      SimulatorImpl **pimpl = PeekImpl (); 
      if (*pimpl == 0)
      {
        return;
      }
      /* Note: we have to call LogSetTimePrinter (0) below because if we do not do
       * this, and restart a simulation after this call to Destroy, (which is 
       * legal), Simulator::GetImpl will trigger again an infinite recursion until
       * the stack explodes.
       */
      LogSetTimePrinter (0);
      LogSetNodePrinter (0);
      (*pimpl)->Destroy ();
      (*pimpl)->Unref ();
      *pimpl = 0;
    }

  void
    Simulator::SetScheduler (ObjectFactory schedulerFactory)
    {
      NS_LOG_FUNCTION (schedulerFactory);
      GetImpl ()->SetScheduler (schedulerFactory);
    }

  bool 
    Simulator::IsFinished (void)
    {
      NS_LOG_FUNCTION_NOARGS ();
      return GetImpl ()->IsFinished ();
    }

  Time
    Simulator::Next (void)
    {
      NS_LOG_FUNCTION_NOARGS ();
      return GetImpl ()->Next ();
    }

  void 
    Simulator::Run (void)
    {
      NS_LOG_FUNCTION_NOARGS ();
      GetImpl ()->Run ();
    }

  void 
    Simulator::RunOneEvent (void)
    {
      NS_LOG_FUNCTION_NOARGS ();
      GetImpl ()->RunOneEvent ();
    }

  void 
    Simulator::Stop (void)
    {
      NS_LOG_LOGIC ("stop");
      GetImpl ()->Stop ();
    }

  void 
    Simulator::Stop (Time const &time)
    {
      NS_LOG_FUNCTION (time);
      GetImpl ()->Stop (time);
    }

  Time
    Simulator::Now (void)
    {
      /* Please, don't include any calls to logging macros in this function
       * or pay the price, that is, stack explosions.
       */
      return GetImpl ()->Now ();
    }

  Time
    Simulator::GetDelayLeft (const EventId &id)
    {
      NS_LOG_FUNCTION (&id);
      return GetImpl ()->GetDelayLeft (id);
    }

  EventId
    Simulator::Schedule (Time const &time, const Ptr<EventImpl> &ev)
    {
      NS_LOG_FUNCTION (time << ev);
      return DoSchedule (time, GetPointer (ev));
    }

  EventId
    Simulator::ScheduleNow (const Ptr<EventImpl> &ev)
    {
      NS_LOG_FUNCTION (ev);
      return DoScheduleNow (GetPointer (ev));
    }
  void
    Simulator::ScheduleWithContext (uint32_t context, const Time &time, EventImpl *impl)
    {
      return GetImpl ()->ScheduleWithContext (context, time, impl);
    }
  EventId
    Simulator::ScheduleDestroy (const Ptr<EventImpl> &ev)
    {
      NS_LOG_FUNCTION (ev);
      return DoScheduleDestroy (GetPointer (ev));
    }
  EventId 
    Simulator::DoSchedule (Time const &time, EventImpl *impl)
    {
      return GetImpl ()->Schedule (time, impl);
    }
  EventId 
    Simulator::DoScheduleNow (EventImpl *impl)
    {
      return GetImpl ()->ScheduleNow (impl);
    }
  EventId 
    Simulator::DoScheduleDestroy (EventImpl *impl)
    {
      return GetImpl ()->ScheduleDestroy (impl);
    }


  EventId
    Simulator::Schedule (Time const &time, void (*f)(void))
    {
      NS_LOG_FUNCTION (time << f);
      return DoSchedule (time, MakeEvent (f));
    }

  void
    Simulator::ScheduleWithContext (uint32_t context, Time const &time, void (*f)(void))
    {
      NS_LOG_FUNCTION (time << context << f);
      return ScheduleWithContext (context, time, MakeEvent (f));
    }

  EventId
    Simulator::ScheduleNow (void (*f)(void))
    {
      NS_LOG_FUNCTION (f);
      return DoScheduleNow (MakeEvent (f));
    }

  EventId
    Simulator::ScheduleDestroy (void (*f)(void))
    {
      NS_LOG_FUNCTION (f);
      return DoScheduleDestroy (MakeEvent (f));
    }

  void
    Simulator::Remove (const EventId &ev)
    {
      NS_LOG_FUNCTION (&ev);
      if (*PeekImpl () == 0)
      {
        return;
      }
      return GetImpl ()->Remove (ev);
    }

  void
    Simulator::Cancel (const EventId &ev)
    {
      NS_LOG_FUNCTION (&ev);
      if (*PeekImpl () == 0)
      {
        return;
      }
      return GetImpl ()->Cancel (ev);
    }

  bool 
    Simulator::IsExpired (const EventId &id)
    {
      NS_LOG_FUNCTION (&id);
      if (*PeekImpl () == 0)
      {
        return true;
      }
      return GetImpl ()->IsExpired (id);
    }

  Time Now (void)
  {
    NS_LOG_FUNCTION_NOARGS ();
    return Time (Simulator::Now ());
  }

  Time 
    Simulator::GetMaximumSimulationTime (void)
    {
      NS_LOG_FUNCTION_NOARGS ();
      return GetImpl ()->GetMaximumSimulationTime ();
    }

  uint32_t
    Simulator::GetContext (void)
    {
      return GetImpl ()->GetContext ();
    }

  uint32_t
    Simulator::GetSystemId (void)
    {
      NS_LOG_FUNCTION_NOARGS ();

      if (*PeekImpl () != 0)
      {
        return GetImpl ()->GetSystemId ();
      }
      else
      {
        return 0;
      }
    }

  void
    Simulator::SetImplementation (Ptr<SimulatorImpl> impl)
    {
      if (*PeekImpl () != 0)
      {
        NS_FATAL_ERROR ("It is not possible to set the implementation after calling any Simulator:: function. Call Simulator::SetImplementation earlier or after Simulator::Destroy.");
      }
      *PeekImpl () = GetPointer (impl);
      // Set the default scheduler
      ObjectFactory factory;
      StringValue s;
      g_schedTypeImpl.GetValue (s);
      factory.SetTypeId (s.Get ());
      impl->SetScheduler (factory);
      //
      // Note: we call LogSetTimePrinter _after_ creating the implementation
      // object because the act of creation can trigger calls to the logging 
      // framework which would call the TimePrinter function which would call 
      // Simulator::Now which would call Simulator::GetImpl, and, thus, get us 
      // in an infinite recursion until the stack explodes.
      //
      LogSetTimePrinter (&TimePrinter);
      LogSetNodePrinter (&NodePrinter);
    }
  Ptr<SimulatorImpl>
    Simulator::GetImplementation (void)
    {
      return GetImpl ();
    }


} // namespace ns3

