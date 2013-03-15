/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005 INRIA
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

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "event-id.h"
#include "event-impl.h"
#include "make-event.h"
#include "nstime.h"

#include "deprecated.h"
#include "object-factory.h"

#include <stdint.h>
#include <string>
#include <vector>

namespace ns3 {

const double PATH_LOSS_EXPONENT = 2.6;
const uint32_t LEARNING_PROCESS_DURATION = 200;
const uint32_t SIMULATION_STOP_TIME = 3000;
const uint32_t NODE_COUNT_UPPER_BOUND = 1000;
const uint32_t MIN_INFORM_RANGE = 20;
const double REFERENCE_DISTANCE = 1.0;
const double REFERENCE_LOSS = 46.6777;
class SimulatorImpl;
class Scheduler;

typedef struct FeasibleSchedule
{
  int64_t controlLink;
  std::vector<int64_t> feasibleLinks;
}FeasibleSchedule;

typedef struct TdmaLink
{
  std::string senderAddr;
  std::string receiverAddr;
  int32_t linkId;
}
TdmaLink;

typedef struct NodeLinkDetails
{
  std::string addr;
  TdmaLink selfInitiatedLink;
  std::vector<TdmaLink> relatedLinks;
} NodeLinkDetails;

typedef struct TdmaSignalMap
{
  std::string from;
  double outBoundAttenuation;
  double inBoundAttenuation;
  double outSinr;
  double inSinr;

  double noisePlusInterferenceW;// when want to guarantee the control signal can be received by a certain PDR, we need to use this
  double supposedInterferenceW;
}
TdmaSignalMap;

typedef struct NodeSignalMap
{
  std::string selfAddress;
  std::vector<TdmaSignalMap> signalMap;
}
NodeSignalMap;

typedef struct ControlReliability
{
  std::string sender;
  std::vector<std::string> idealReceivers;
  uint32_t actualReceiverCount;
} 
ControlReliability;

/**
 * \ingroup core
 *
 * \brief Control the scheduling of simulation events. 
 *
 * The internal simulation clock is maintained
 * as a 64-bit integer in a unit specified by the user
 * through the TimeStepPrecision::Set function. This means that it is
 * not possible to specify event expiration times with anything better
 * than this user-specified accuracy. Events whose expiration time is
 * the same modulo this accuracy are scheduled in FIFO order: the 
 * first event inserted in the scheduling queue is scheduled to 
 * expire first.
 * 
 * A simple example of how to use the Simulator class to schedule events
 * is shown below:
 * \include src/core/examples/sample-simulator.cc
 */
class Simulator 
{
public:
  /**
   * \param impl a new simulator implementation
   *
   * The simulator provides a mechanism to swap out different implementations.
   * For example, the default implementation is a single-threaded simulator
   * that performs no realtime synchronization.  By calling this method, you
   * can substitute in a new simulator implementation that might be multi-
   * threaded and synchronize events to a realtime clock.
   *
   * The simulator implementation can be set when the simulator is not 
   * running.
   */
  static void SetImplementation (Ptr<SimulatorImpl> impl);

  static Ptr<SimulatorImpl> GetImplementation (void);

  /**
   * \param schedulerFactory a new event scheduler factory
   *
   * The event scheduler can be set at any time: the events scheduled
   * in the previous scheduler will be transfered to the new scheduler
   * before we start to use it.
   */
  static void SetScheduler (ObjectFactory schedulerFactory);

  /**
   * Every event scheduled by the Simulator::insertAtDestroy method is
   * invoked. Then, we ensure that any memory allocated by the 
   * Simulator is freed.
   * This method is typically invoked at the end of a simulation
   * to avoid false-positive reports by a leak checker.
   * After this method has been invoked, it is actually possible
   * to restart a new simulation with a set of calls to Simulator::run
   * and Simulator::insert_*.
   */
  static void Destroy (void);

  /**
   * If there are no more events lefts to be scheduled, or if simulation
   * time has already reached the "stop time" (see Simulator::Stop()),
   * return true. Return false otherwise.
   */
  static bool IsFinished (void);
  /**
   * If Simulator::IsFinished returns true, the behavior of this
   * method is undefined. Otherwise, it returns the microsecond-based
   * time of the next event expected to be scheduled.
   */
  static Time Next (void) NS_DEPRECATED;

  /**
   * Run the simulation until one of:
   *   - no events are present anymore
   *   - the user called Simulator::stop
   *   - the user called Simulator::stopAtUs and the
   *     expiration time of the next event to be processed
   *     is greater than or equal to the stop time.
   */
  static void Run (void);

  static int16_t NodesCountUpperBound;
  static Time SimulationStopTime;
  static Time LearningTimeDuration;
  static uint32_t MinInformRange;
  static std::vector<std::string> NodesWillSend;
  // if the @Simulator::Now () is different from this value, that means, we are entering a new slot, therefore, we need to clear
  static Time SlotBeginningTime; 

  /**
   * Process only the next simulation event, then return immediately.
   */
  static void RunOneEvent (void) NS_DEPRECATED;

  /**
   * If an event invokes this method, it will be the last
   * event scheduled by the Simulator::run method before
   * returning to the caller.
   */
  static void Stop (void);

  /**
   * Force the Simulator::run method to return to the caller when the
   * expiration time of the next event to be processed is greater than
   * or equal to the stop time.  The stop time is relative to the
   * current simulation time.
   * @param time the stop time, relative to the current time.
   */
  static void Stop (Time const &time);

  /**
   * Schedule an event to expire at the relative time "time"
   * is reached.  This can be thought of as scheduling an event
   * for the current simulation time plus the Time passed as a
   * parameter
   *
   * When the event expires (when it becomes due to be run), the 
   * input method will be invoked on the input object.
   *
   * @param time the relative expiration time of the event.
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @returns an id for the scheduled event.
   */
  template <typename MEM, typename OBJ>
  static EventId Schedule (Time const &time, MEM mem_ptr, OBJ obj);

  /**
   * @param time the relative expiration time of the event.
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @returns an id for the scheduled event.
   */
  template <typename MEM, typename OBJ, typename T1>
  static EventId Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1);

  /**
   * @param time the relative expiration time of the event.
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @returns an id for the scheduled event.
   */
  template <typename MEM, typename OBJ, typename T1, typename T2>
  static EventId Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

  /**
   * @param time the relative expiration time of the event.
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @returns an id for the scheduled event.
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3>
  static EventId Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);

  /**
   * @param time the relative expiration time of the event.
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   * @returns an id for the scheduled event.
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4>
  static EventId Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3, T4 a4);

  /**
   * @param time the relative expiration time of the event.
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   * @param a5 the fifth argument to pass to the invoked method
   * @returns an id for the scheduled event.
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static EventId Schedule (Time const &time, MEM mem_ptr, OBJ obj, 
                           T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);
  /**
   * @param time the relative expiration time of the event.
   * @param f the function to invoke
   * @returns an id for the scheduled event.
   */
  static EventId Schedule (Time const &time, void (*f)(void));

  /**
   * @param time the relative expiration time of the event.
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @returns an id for the scheduled event.
   */
  template <typename U1, typename T1>
  static EventId Schedule (Time const &time, void (*f)(U1), T1 a1);

  /**
   * @param time the relative expiration time of the event.
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @returns an id for the scheduled event.
   */
  template <typename U1, typename U2, typename T1, typename T2>
  static EventId Schedule (Time const &time, void (*f)(U1,U2), T1 a1, T2 a2);

  /**
   * @param time the relative expiration time of the event.
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @returns an id for the scheduled event.
   */
  template <typename U1, typename U2, typename U3, typename T1, typename T2, typename T3>
  static EventId Schedule (Time const &time, void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3);

  /**
   * @param time the relative expiration time of the event.
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   * @returns an id for the scheduled event.
   */
  template <typename U1, typename U2, typename U3, typename U4, 
            typename T1, typename T2, typename T3, typename T4>
  static EventId Schedule (Time const &time, void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4);

  /**
   * @param time the relative expiration time of the event.
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   * @param a5 the fifth argument to pass to the function to invoke
   * @returns an id for the scheduled event.
   */
  template <typename U1, typename U2, typename U3, typename U4, typename U5,
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static EventId Schedule (Time const &time, void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);

  /**
   * Schedule an event with the given context.
   * A context of 0xffffffff means no context is specified.
   *
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   */
  template <typename MEM, typename OBJ>
  static void ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, typename T1>
  static void ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, typename T1, typename T2>
  static void ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3>
  static void ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4>
  static void ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3, T4 a4);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   * @param a5 the fifth argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static void ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, 
                                   T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);
  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param f the function to invoke
   */
  static void ScheduleWithContext (uint32_t context, Time const &time, void (*f)(void));

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   */
  template <typename U1, typename T1>
  static void ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1), T1 a1);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename T1, typename T2>
  static void ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2), T1 a1, T2 a2);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename T1, typename T2, typename T3>
  static void ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename U4, 
            typename T1, typename T2, typename T3, typename T4>
  static void ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4);

  /**
   * @param time the relative expiration time of the event.
   * @param context user-specified context parameter
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   * @param a5 the fifth argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename U4, typename U5,
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static void ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);

  /**
   * Schedule an event to expire Now. All events scheduled to
   * to expire "Now" are scheduled FIFO, after all normal events
   * have expired. 
   *
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   */
  template <typename MEM, typename OBJ>
  static EventId ScheduleNow (MEM mem_ptr, OBJ obj);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1>
  static EventId ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2>
  static EventId ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3>
  static EventId ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4>
  static EventId ScheduleNow (MEM mem_ptr, OBJ obj, 
                              T1 a1, T2 a2, T3 a3, T4 a4);
  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   * @param a5 the fifth argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static EventId ScheduleNow (MEM mem_ptr, OBJ obj, 
                              T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);
  /**
   * @param f the function to invoke
   */
  static EventId ScheduleNow (void (*f)(void));

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   */
  template <typename U1,
            typename T1>
  static EventId ScheduleNow (void (*f)(U1), T1 a1);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   */
  template <typename U1, typename U2,
            typename T1, typename T2>
  static EventId ScheduleNow (void (*f)(U1,U2), T1 a1, T2 a2);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3,
            typename T1, typename T2, typename T3>
  static EventId ScheduleNow (void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename U4,
            typename T1, typename T2, typename T3, typename T4>
  static EventId ScheduleNow (void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   * @param a5 the fifth argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename U4, typename U5,
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static EventId ScheduleNow (void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);

  /**
   * Schedule an event to expire at Destroy time. All events 
   * scheduled to expire at "Destroy" time are scheduled FIFO, 
   * after all normal events have expired and only when 
   * Simulator::Destroy is invoked.
   *
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   */
  template <typename MEM, typename OBJ>
  static EventId ScheduleDestroy (MEM mem_ptr, OBJ obj);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1>
  static EventId ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ,
            typename T1, typename T2>
  static EventId ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1, T2 a2);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3>
  static EventId ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3);

  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4>
  static EventId ScheduleDestroy (MEM mem_ptr, OBJ obj, 
                                  T1 a1, T2 a2, T3 a3, T4 a4);
  /**
   * @param mem_ptr member method pointer to invoke
   * @param obj the object on which to invoke the member method
   * @param a1 the first argument to pass to the invoked method
   * @param a2 the second argument to pass to the invoked method
   * @param a3 the third argument to pass to the invoked method
   * @param a4 the fourth argument to pass to the invoked method
   * @param a5 the fifth argument to pass to the invoked method
   */
  template <typename MEM, typename OBJ, 
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static EventId ScheduleDestroy (MEM mem_ptr, OBJ obj, 
                                  T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);
  /**
   * @param f the function to invoke
   */
  static EventId ScheduleDestroy (void (*f)(void));

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   */
  template <typename U1,
            typename T1>
  static EventId ScheduleDestroy (void (*f)(U1), T1 a1);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   */
  template <typename U1, typename U2,
            typename T1, typename T2>
  static EventId ScheduleDestroy (void (*f)(U1,U2), T1 a1, T2 a2);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3,
            typename T1, typename T2, typename T3>
  static EventId ScheduleDestroy (void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename U4,
            typename T1, typename T2, typename T3, typename T4>
  static EventId ScheduleDestroy (void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4);

  /**
   * @param f the function to invoke
   * @param a1 the first argument to pass to the function to invoke
   * @param a2 the second argument to pass to the function to invoke
   * @param a3 the third argument to pass to the function to invoke
   * @param a4 the fourth argument to pass to the function to invoke
   * @param a5 the fifth argument to pass to the function to invoke
   */
  template <typename U1, typename U2, typename U3, typename U4, typename U5,
            typename T1, typename T2, typename T3, typename T4, typename T5>
  static EventId ScheduleDestroy (void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5);

  /**
   * Remove an event from the event list. 
   * This method has the same visible effect as the 
   * ns3::EventId::Cancel method
   * but its algorithmic complexity is much higher: it has often 
   * O(log(n)) complexity, sometimes O(n), sometimes worse.
   * Note that it is not possible to remove events which were scheduled
   * for the "destroy" time. Doing so will result in a program error (crash).
   *
   * @param id the event to remove from the list of scheduled events.
   */
  static void Remove (const EventId &id);

  /**
   * Set the cancel bit on this event: the event's associated function
   * will not be invoked when it expires. 
   * This method has the same visible effect as the 
   * ns3::Simulator::remove method but its algorithmic complexity is 
   * much lower: it has O(1) complexity.
   * This method has the exact same semantics as ns3::EventId::cancel.
   * Note that it is not possible to cancel events which were scheduled
   * for the "destroy" time. Doing so will result in a program error (crash).
   * 
   * @param id the event to cancel
   */
  static void Cancel (const EventId &id);

  /**
   * This method has O(1) complexity.
   * Note that it is not possible to test for the expiration of
   * events which were scheduled for the "destroy" time. Doing so
   * will result in a program error (crash).
   * An event is said to "expire" when it starts being scheduled
   * which means that if the code executed by the event calls
   * this function, it will get true.
   *
   * @param id the event to test for expiration
   * @returns true if the event has expired, false otherwise.
   */
  static bool IsExpired (const EventId &id);

  /**
   * Return the "current simulation time".
   */
  static Time Now (void);

  /**
   * \param id the event id to analyse
   * \returns the delay left until the input event id expires.
   *          if the event is not running, this method returns
   *          zero.
   */
  static Time GetDelayLeft (const EventId &id);

  /**
   * \returns the maximum simulation time at which an event 
   *          can be scheduled.
   *
   * The returned value will always be bigger than or equal to Simulator::Now.
   */
  static Time GetMaximumSimulationTime (void);

  /**
   * \returns the current simulation context
   */
  static uint32_t GetContext (void);

  /**
   * \param time delay until the event expires
   * \param event the event to schedule
   * \returns a unique identifier for the newly-scheduled event.
   *
   * This method will be typically used by language bindings
   * to delegate events to their own subclass of the EventImpl base class.
   */
  static EventId Schedule (Time const &time, const Ptr<EventImpl> &event);

  /**
   * \param time delay until the event expires
   * \param context event context
   * \param event the event to schedule
   * \returns a unique identifier for the newly-scheduled event.
   *
   * This method will be typically used by language bindings
   * to delegate events to their own subclass of the EventImpl base class.
   */
  static void ScheduleWithContext (uint32_t context, const Time &time, EventImpl *event);

  /**
   * \param event the event to schedule
   * \returns a unique identifier for the newly-scheduled event.
   *
   * This method will be typically used by language bindings
   * to delegate events to their own subclass of the EventImpl base class.
   */
  static EventId ScheduleDestroy (const Ptr<EventImpl> &event);

  /**
   * \param event the event to schedule
   * \returns a unique identifier for the newly-scheduled event.
   *
   * This method will be typically used by language bindings
   * to delegate events to their own subclass of the EventImpl base class.
   */
  static EventId ScheduleNow (const Ptr<EventImpl> &event);
  static void AddTdmaLink (TdmaLink link);
  static void AddSignalMapItem (std::string selfAddr, std::string neighborAddr, double outBoundAttenuation, double inBoundAttenuation, double outSinr, double inSinr, double noisePlusInterferenceW, double supposedInterferenceW);
  static void PrintSignalMap (std::string addr);
  static void PrintLinks ();
  static std::vector<TdmaLink> ListAllLinks ();
  static void ResigerSendingNode (std::string sender, double senderEdgeInterferenceW, std::string receiver, double receiverEdgeInterferenceW);
  static void ClearSendingNodes ();

  /**
   * \returns the system id for this simulator; used for 
   *          MPI or other distributed simulations
   */
  static uint32_t GetSystemId (void);
  static TdmaLink FindLinkBySender (std::string senderAddr );
  static std::vector<TdmaLink> FindRelatedLinks (std::string addr);
  static std::vector<std::string> ListNodesInEr (std::string senderAddr, double senderErEdgeInterferenceW, std::string receiverAddr, double receiverErEdgeInterferenceW);
  static std::vector<std::string> ListNodesInEr (std::string nodeAddr, double nodeErEdgeInterferenceW);
  static std::vector<std::string> ListNodesInEr (std::string nodeAddr);
  static void SenderRegisterControlReliability (std::string sender, std::vector<std::string> idealReceivers); // actual receiver count: 0
  static void ReceiverRegisterControlReliability (std::string sender, std::string receiver); // count ++
  static void CountFinalControlReliability ();

  static std::vector<std::string> m_nodesInDataChannel;
  static bool m_linksClassified;
  static NodeLinkDetails m_nodeLinkDetails[NODE_COUNT_UPPER_BOUND];
  static std::vector<TdmaSignalMap> GetNodeSignalMap (std::string addr);
  static std::vector<int64_t> m_sendingLinks;
  static int16_t m_controlNodeId;
  static int64_t m_controlLink;

  //SCREAM implementation
  /* @linkId is the control link id. If found, return true, otherwise false.
   */
  static bool CheckLinkScheduledAsControlLink (int64_t linkId);
  /* Find the feasible schedule for @m_controlLink first, then add @linkId into its feasibleLinks vector
   * @feasibleLinks vector keeps records of feasible links for the control link @m_controlLink
   */
  static void RegisterFeasibleLink (int64_t linkId);
  /* Find the feasible schedule for the control link @linkId
   * This operation should only be executed after the SCREAM scheduling work has been done.
   */
  static FeasibleSchedule GetScheduleByControlLink (int64_t linkId);
  /* Register a new feasible schedule in @m_screamSchedules. the control link id is
   * provided in @feasibleSchedule.controlLink
   */
  static void RegisterNewFeasibleSchedule (FeasibleSchedule feasibleSchedule);
  /* Register scream premitive, if true, newly considered link should be removed
   * if false, newly considered link can be added into @feasibleLinks vector
   */
  static void RegisterScreamPremitive(bool val);
  /* Return the scream premitive value
   */
  static bool CheckScreamPremitive();

  static void PrintScreamSchedules ();
  //--------------------------------------------------------------------PRIVATE-------------------------------------------------------
private:
  Simulator ();
  ~Simulator ();
  static EventId DoSchedule (Time const &time, EventImpl *event);
  static EventId DoScheduleNow (EventImpl *event);
  static EventId DoScheduleDestroy (EventImpl *event);
  static std::vector<TdmaLink> m_tdmaLinks;
  static std::vector<NodeSignalMap> m_signalMaps;
  static std::vector<ControlReliability> m_controlReliabilityCollec;
  static std::vector<FeasibleSchedule> m_screamSchedules;
  static bool m_screamPremitive;
};

/**
 * \brief create an ns3::Time instance which contains the
 *        current simulation time.
 *
 * This is really a shortcut for the ns3::Simulator::Now method.
 * It is typically used as shown below to schedule an event
 * which expires at the absolute time "2 seconds":
 * \code
 * Simulator::Schedule (Seconds (2.0) - Now (), &my_function);
 * \endcode
 */
Time Now (void);

} // namespace ns3

namespace ns3 {

template <typename MEM, typename OBJ>
EventId Simulator::Schedule (Time const &time, MEM mem_ptr, OBJ obj) 
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj));
}


template <typename MEM, typename OBJ,
          typename T1>
EventId Simulator::Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1) 
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj, a1));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2>
EventId Simulator::Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2)
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj, a1, a2));
}

template <typename MEM, typename OBJ,
          typename T1, typename T2, typename T3>
EventId Simulator::Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3) 
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj, a1, a2, a3));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3, typename T4>
EventId Simulator::Schedule (Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3, T4 a4) 
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj, a1, a2, a3, a4));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3, typename T4, typename T5>
EventId Simulator::Schedule (Time const &time, MEM mem_ptr, OBJ obj, 
                             T1 a1, T2 a2, T3 a3, T4 a4, T5 a5) 
{
  return DoSchedule (time, MakeEvent (mem_ptr, obj, a1, a2, a3, a4, a5));
}

template <typename U1, typename T1>
EventId Simulator::Schedule (Time const &time, void (*f)(U1), T1 a1)
{
  return DoSchedule (time, MakeEvent (f, a1));
}

template <typename U1, typename U2, 
          typename T1, typename T2>
EventId Simulator::Schedule (Time const &time, void (*f)(U1,U2), T1 a1, T2 a2)
{
  return DoSchedule (time, MakeEvent (f, a1, a2));
}

template <typename U1, typename U2, typename U3,
          typename T1, typename T2, typename T3>
EventId Simulator::Schedule (Time const &time, void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3)
{
  return DoSchedule (time, MakeEvent (f, a1, a2, a3));
}

template <typename U1, typename U2, typename U3, typename U4,
          typename T1, typename T2, typename T3, typename T4>
EventId Simulator::Schedule (Time const &time, void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4)
{
  return DoSchedule (time, MakeEvent (f, a1, a2, a3, a4));
}

template <typename U1, typename U2, typename U3, typename U4, typename U5,
          typename T1, typename T2, typename T3, typename T4, typename T5>
EventId Simulator::Schedule (Time const &time, void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5)
{
  return DoSchedule (time, MakeEvent (f, a1, a2, a3, a4, a5));
}




template <typename MEM, typename OBJ>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj)
{
  ScheduleWithContext (context, time, MakeEvent (mem_ptr, obj));
}


template <typename MEM, typename OBJ,
          typename T1>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1)
{
  return ScheduleWithContext (context, time, MakeEvent (mem_ptr, obj, a1));
}

template <typename MEM, typename OBJ,
          typename T1, typename T2>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2)
{
  return ScheduleWithContext (context, time, MakeEvent (mem_ptr, obj, a1, a2));
}

template <typename MEM, typename OBJ,
          typename T1, typename T2, typename T3>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3)
{
  return ScheduleWithContext (context, time, MakeEvent (mem_ptr, obj, a1, a2, a3));
}

template <typename MEM, typename OBJ,
          typename T1, typename T2, typename T3, typename T4>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3, T4 a4)
{
  return ScheduleWithContext (context, time, MakeEvent (mem_ptr, obj, a1, a2, a3, a4));
}

template <typename MEM, typename OBJ,
          typename T1, typename T2, typename T3, typename T4, typename T5>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, MEM mem_ptr, OBJ obj,
                                     T1 a1, T2 a2, T3 a3, T4 a4, T5 a5)
{
  return ScheduleWithContext (context, time, MakeEvent (mem_ptr, obj, a1, a2, a3, a4, a5));
}

template <typename U1, typename T1>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1), T1 a1)
{
  return ScheduleWithContext (context, time, MakeEvent (f, a1));
}

template <typename U1, typename U2,
          typename T1, typename T2>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2), T1 a1, T2 a2)
{
  return ScheduleWithContext (context, time, MakeEvent (f, a1, a2));
}

template <typename U1, typename U2, typename U3,
          typename T1, typename T2, typename T3>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3)
{
  return ScheduleWithContext (context, time, MakeEvent (f, a1, a2, a3));
}

template <typename U1, typename U2, typename U3, typename U4,
          typename T1, typename T2, typename T3, typename T4>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4)
{
  return ScheduleWithContext (context, time, MakeEvent (f, a1, a2, a3, a4));
}

template <typename U1, typename U2, typename U3, typename U4, typename U5,
          typename T1, typename T2, typename T3, typename T4, typename T5>
void Simulator::ScheduleWithContext (uint32_t context, Time const &time, void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5)
{
  return ScheduleWithContext (context, time, MakeEvent (f, a1, a2, a3, a4, a5));
}




template <typename MEM, typename OBJ>
EventId
Simulator::ScheduleNow (MEM mem_ptr, OBJ obj) 
{
  return DoScheduleNow (MakeEvent (mem_ptr, obj));
}


template <typename MEM, typename OBJ, 
          typename T1>
EventId
Simulator::ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1) 
{
  return DoScheduleNow (MakeEvent (mem_ptr, obj, a1));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2>
EventId
Simulator::ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1, T2 a2) 
{
  return DoScheduleNow (MakeEvent (mem_ptr, obj, a1, a2));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3>
EventId
Simulator::ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3) 
{
  return DoScheduleNow (MakeEvent (mem_ptr, obj, a1, a2, a3));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3, typename T4>
EventId
Simulator::ScheduleNow (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3, T4 a4) 
{
  return DoScheduleNow (MakeEvent (mem_ptr, obj, a1, a2, a3, a4));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3, typename T4, typename T5>
EventId
Simulator::ScheduleNow (MEM mem_ptr, OBJ obj, 
                        T1 a1, T2 a2, T3 a3, T4 a4, T5 a5) 
{
  return DoScheduleNow (MakeEvent (mem_ptr, obj, a1, a2, a3, a4, a5));
}

template <typename U1,
          typename T1>
EventId
Simulator::ScheduleNow (void (*f)(U1), T1 a1)
{
  return DoScheduleNow (MakeEvent (f, a1));
}

template <typename U1, typename U2,
          typename T1, typename T2>
EventId
Simulator::ScheduleNow (void (*f)(U1,U2), T1 a1, T2 a2)
{
  return DoScheduleNow (MakeEvent (f, a1, a2));
}

template <typename U1, typename U2, typename U3,
          typename T1, typename T2, typename T3>
EventId
Simulator::ScheduleNow (void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3)
{
  return DoScheduleNow (MakeEvent (f, a1, a2, a3));
}

template <typename U1, typename U2, typename U3, typename U4,
          typename T1, typename T2, typename T3, typename T4>
EventId
Simulator::ScheduleNow (void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4)
{
  return DoScheduleNow (MakeEvent (f, a1, a2, a3, a4));
}

template <typename U1, typename U2, typename U3, typename U4, typename U5,
          typename T1, typename T2, typename T3, typename T4, typename T5>
EventId
Simulator::ScheduleNow (void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5)
{
  return DoScheduleNow (MakeEvent (f, a1, a2, a3, a4, a5));
}



template <typename MEM, typename OBJ>
EventId
Simulator::ScheduleDestroy (MEM mem_ptr, OBJ obj) 
{
  return DoScheduleDestroy (MakeEvent (mem_ptr, obj));
}


template <typename MEM, typename OBJ, 
          typename T1>
EventId
Simulator::ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1) 
{
  return DoScheduleDestroy (MakeEvent (mem_ptr, obj, a1));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2>
EventId
Simulator::ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1, T2 a2) 
{
  return DoScheduleDestroy (MakeEvent (mem_ptr, obj, a1, a2));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3>
EventId
Simulator::ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3) 
{
  return DoScheduleDestroy (MakeEvent (mem_ptr, obj, a1, a2, a3));
}

template <typename MEM, typename OBJ,
          typename T1, typename T2, typename T3, typename T4>
EventId
Simulator::ScheduleDestroy (MEM mem_ptr, OBJ obj, T1 a1, T2 a2, T3 a3, T4 a4) 
{
  return DoScheduleDestroy (MakeEvent (mem_ptr, obj, a1, a2, a3, a4));
}

template <typename MEM, typename OBJ, 
          typename T1, typename T2, typename T3, typename T4, typename T5>
EventId
Simulator::ScheduleDestroy (MEM mem_ptr, OBJ obj, 
                            T1 a1, T2 a2, T3 a3, T4 a4, T5 a5) 
{
  return DoScheduleDestroy (MakeEvent (mem_ptr, obj, a1, a2, a3, a4, a5));
}

template <typename U1,
          typename T1>
EventId
Simulator::ScheduleDestroy (void (*f)(U1), T1 a1)
{
  return DoScheduleDestroy (MakeEvent (f, a1));
}

template <typename U1, typename U2,
          typename T1, typename T2>
EventId
Simulator::ScheduleDestroy (void (*f)(U1,U2), T1 a1, T2 a2)
{
  return DoScheduleDestroy (MakeEvent (f, a1, a2));
}

template <typename U1, typename U2, typename U3,
          typename T1, typename T2, typename T3>
EventId
Simulator::ScheduleDestroy (void (*f)(U1,U2,U3), T1 a1, T2 a2, T3 a3)
{
  return DoScheduleDestroy (MakeEvent (f, a1, a2, a3));
}

template <typename U1, typename U2, typename U3, typename U4,
          typename T1, typename T2, typename T3, typename T4>
EventId
Simulator::ScheduleDestroy (void (*f)(U1,U2,U3,U4), T1 a1, T2 a2, T3 a3, T4 a4)
{
  return DoScheduleDestroy (MakeEvent (f, a1, a2, a3, a4));
}

template <typename U1, typename U2, typename U3, typename U4, typename U5,
          typename T1, typename T2, typename T3, typename T4, typename T5>
EventId
Simulator::ScheduleDestroy (void (*f)(U1,U2,U3,U4,U5), T1 a1, T2 a2, T3 a3, T4 a4, T5 a5)
{
  return DoScheduleDestroy (MakeEvent (f, a1, a2, a3, a4, a5));
}

} // namespace ns3

#endif /* SIMULATOR_H */
