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
#include "interference-helper.h"
#include "wifi-phy.h"
#include "error-rate-model.h"
#include "ns3/simulator.h"
#include "ns3/mac48-address.h"
#include "ns3/log.h"
#include <algorithm>
#include <cmath>
#include "math-helper.h"
#include <string>
#include <sstream>
#include "quick-sort.h"
#include "settings.h"
NS_LOG_COMPONENT_DEFINE ("InterferenceHelper");

namespace ns3 {

/****************************************************************
 *       Phy event class
 ****************************************************************/

InterferenceHelper::Event::Event (uint32_t size, WifiMode payloadMode,
                                  enum WifiPreamble preamble,
                                  Time duration, double rxPower)
  : m_size (size),
    m_payloadMode (payloadMode),
    m_preamble (preamble),
    m_startTime (Simulator::Now ()),
    m_endTime (m_startTime + duration),
    m_rxPowerW (rxPower)
{
}
InterferenceHelper::Event::~Event ()
{
}

Time
InterferenceHelper::Event::GetDuration (void) const
{
  return m_endTime - m_startTime;
}
Time
InterferenceHelper::Event::GetStartTime (void) const
{
  return m_startTime;
}
Time
InterferenceHelper::Event::GetEndTime (void) const
{
  return m_endTime;
}
double
InterferenceHelper::Event::GetRxPowerW (void) const
{
  return m_rxPowerW;
}
uint32_t
InterferenceHelper::Event::GetSize (void) const
{
  return m_size;
}
WifiMode
InterferenceHelper::Event::GetPayloadMode (void) const
{
  return m_payloadMode;
}
enum WifiPreamble
InterferenceHelper::Event::GetPreambleType (void) const
{
  return m_preamble;
}

/****************************************************************
 *       Class which records SNIR change events for a
 *       short period of time.
 ****************************************************************/

InterferenceHelper::NiChange::NiChange (Time time, double delta)
  : m_time (time),
    m_delta (delta)
{
}
Time
InterferenceHelper::NiChange::GetTime (void) const
{
  return m_time;
}
double
InterferenceHelper::NiChange::GetDelta (void) const
{
  return m_delta;
}
bool
InterferenceHelper::NiChange::operator < (const InterferenceHelper::NiChange& o) const
{
  return (m_time < o.m_time);
}

/****************************************************************
 *       The actual InterferenceHelper
 ****************************************************************/

InterferenceHelper::InterferenceHelper ()
  : m_errorRateModel (0),
    m_firstPower (0.0),
    m_rxing (false)
{
  m_noiseW = NOISE_POWER_W; // in the unit of Watt
  m_interferenceW = DEFAULT_INTERFERENCE_W; // in the unit of Watt
  m_ewmaCoefficient = EWMA_COEFFICIENT;
  m_sampleInterval = MicroSeconds (NORMAL_SAMPLE_INTERVAL);
  m_dataInterferenceSampleInterval = MicroSeconds (DATA_INTERFERENCE_SAMPLE_INTERVAL);
  m_ackInterferenceSampleInterval = MicroSeconds (ACK_INTERFERENCE_SAMPLE_INTERVAL);
  m_controlInterferenceSampleInterval = MicroSeconds (CONTROL_INTERFERENCE_SAMPLE_INTERVAL);
  //Simulator::Schedule (Simulator::LearningTimeDuration , &InterferenceHelper::DoSampleInterference, this);
  m_simulationEndTime = Simulator::SimulationStopTime;
  m_niDataReceivingSampling = false;
  m_niDataReceivingSampling_ControlChannel = false;
  m_niAckReceivingSampling = false;
  m_concurrency = 0;
  m_receivedPowerW = 0;

  m_meanControlInterferenceW = 0;
  m_varianceControlInterferenceW = 0;
  m_meanDataInterferenceW = 0;
  m_varianceDataInterferenceW = 0;
  m_meanAckInterferenceW = 0;
  m_varianceAckInterferenceW = 0;
}
InterferenceHelper::~InterferenceHelper ()
{
  EraseEvents ();
  m_errorRateModel = 0;
}

Ptr<InterferenceHelper::Event>
InterferenceHelper::Add (uint32_t size, WifiMode payloadMode,
                         enum WifiPreamble preamble,
                         Time duration, double rxPowerW)
{
  Ptr<InterferenceHelper::Event> event;

  event = Create<InterferenceHelper::Event> (size,
                                             payloadMode,
                                             preamble,
                                             duration,
                                             rxPowerW);
  AppendEvent (event);
  return event;
}


void
InterferenceHelper::SetNoiseFigure (double value)
{
  m_noiseFigure = value;
}

double
InterferenceHelper::GetNoiseFigure (void) const
{
  return m_noiseFigure;
}

void
InterferenceHelper::SetErrorRateModel (Ptr<ErrorRateModel> rate)
{
  m_errorRateModel = rate;
}

Ptr<ErrorRateModel>
InterferenceHelper::GetErrorRateModel (void) const
{
  return m_errorRateModel;
}

/* Get the duration (start from Simulator::Now ()) in which the background energy is greater than or equal to the parameter @engeryW.
 */
Time
InterferenceHelper::GetEnergyDuration (double energyW)
{
  Time now = Simulator::Now ();
  double noiseInterferenceW = 0.0;
  Time end = now;
  noiseInterferenceW = m_firstPower;
  for (NiChanges::const_iterator i = m_niChanges.begin (); i != m_niChanges.end (); i++)
    {
      noiseInterferenceW += i->GetDelta ();
      end = i->GetTime ();
      if (end < now)
        {
          continue;
        }
      if (noiseInterferenceW < energyW)
        {
          break;
        }
    }
  return end > now ? end - now : MicroSeconds (0);
}

/* Add transmissions into a log vector. This vector is quite important in terms of interference calculation
 * Meanwhile,  we maintain a variable @m_concurrency to keep track of the number of concurrent transmitters.
 * Note that when we switch the channel, the will invoke the @EraseEvent method, and we need to reset the 
 * variable @m_concurrency to zero. 
 */
void
InterferenceHelper::AppendEvent (Ptr<InterferenceHelper::Event> event)
{
  Time now = Simulator::Now ();

  if (!m_rxing)
  {
    NiChanges::iterator nowIterator = GetPosition (now);
    for (NiChanges::iterator i = m_niChanges.begin (); i != nowIterator; i++)
    {
      m_firstPower += i->GetDelta ();
      if ( i->GetDelta () < 0 )
      {
        m_concurrency --;
      }
      if (m_firstPower < 0 || m_niChanges.size () == 0)
      {
        m_firstPower = 0;
      }
    }
    m_niChanges.erase (m_niChanges.begin (), nowIterator);
    if ( m_niChanges.size () == 0 )
    {
      m_firstPower = 0;
    }
    m_niChanges.insert (m_niChanges.begin (), NiChange (event->GetStartTime (), event->GetRxPowerW ()));
    m_concurrency ++;
  }
  else
  {
    AddNiChangeEvent (NiChange (event->GetStartTime (), event->GetRxPowerW ()));
    m_concurrency ++;
  }
  AddNiChangeEvent (NiChange (event->GetEndTime (), -event->GetRxPowerW ()));

}


double
InterferenceHelper::CalculateSnr (double signal, double noiseInterference, WifiMode mode)
{
  // thermal noise at 290K in J/s = W
  static const double BOLTZMANN = 1.3803e-23;
  // Nt is the power of thermal noise in W
  double Nt = BOLTZMANN * 290.0 * mode.GetBandwidth ();
  
  // receiver noise Floor (W) which accounts for thermal noise and non-idealities of the receiver
  double noiseFloorW = m_noiseFigure * Nt;
  noiseFloorW = NOISE_POWER_W; // similar to NetEye
  m_noiseW = noiseFloorW; // m_noiseW is in the unit of Watt

  double noise = noiseFloorW + noiseInterference;
  double snr = signal / noise;
  //std::cout<<" in calculate_snr, snr is: "<< snr << std::endl;
  return snr;
}

void InterferenceHelper::PrintDetails ()
{
  std::cout<<m_oss.str ();
  m_oss.clear ();
  m_oss.seekp (0);
}

/* 
 * Get the current background noise power in the unit of Watt
 */
double InterferenceHelper::GetNoise ()
{
  if ( m_noiseW < 0)
  {
    m_noiseW = NOISE_POWER_W;
  }
  return m_noiseW;
}

/*
 * Get the current interference power in the unit of Watt
 */
double InterferenceHelper::GetInterference ()
{
  if (  m_interferenceW <= 0)
  {
    //std::cout<<" interference power becomes negative, error happend! "<<std::endl;
    m_interferenceW = 1.000e-23; // never be negative
  }
  return m_interferenceW;
}

/* According to the time duration specified in the parameter @event, this method creates a vector and copy all the events
 * that happend within the time duration to the variable @ni and return the interference value in Watt
 */
double
InterferenceHelper::CalculateNoiseInterferenceW (Ptr<InterferenceHelper::Event> event, NiChanges *ni)
{
  double noiseInterference = m_firstPower;
  NS_ASSERT (m_rxing);
  for (NiChanges::const_iterator i = m_niChanges.begin () + 1; m_niChanges.size () > 0 && i != m_niChanges.end (); i++)
  {
    if ((event->GetEndTime () == i->GetTime ()) && event->GetRxPowerW () == -i->GetDelta ())
      {
        break;
      }
    ni->push_back (*i);
  }
  ni->insert (ni->begin (), NiChange (event->GetStartTime (), noiseInterference));
  ni->push_back (NiChange (event->GetEndTime (), 0));
  return noiseInterference;
}

/* Return the number of concurrent transmitters.
 */
uint32_t InterferenceHelper::GetConcurrentTxNo () const
{
  return m_concurrency;
}

/* In piece-wise interference modeling, packets are divided into chunks and the simulator calculate the chunk success rate for every chunk
 * . That is what this method will do.
 */
double
InterferenceHelper::CalculateChunkSuccessRate (double snir, Time duration, WifiMode mode) const
{
  if (duration == NanoSeconds (0))
    {
      return 1.0;
    }
  uint32_t rate = mode.GetPhyRate ();
  uint64_t nbits = (uint64_t)(rate * duration.GetSeconds ());
  double csr = m_errorRateModel->GetChunkSuccessRate (mode, snir, (uint32_t)nbits);
  return csr;
}

/* Return the factorial value of an integer number
 */
double InterferenceHelper::Factorial ( int n)
{
  double res = 1;
  for( int i = 1; i <= n; ++ i)
  {
    res = res*i;
  }
  return res;
}
double InterferenceHelper::GetFirstPower () const
{
  return m_firstPower;
}

/* Calculate the Packet Error Rate (PER) based on N+I for different packet chunks
 */
double
InterferenceHelper::CalculatePer (Ptr<const InterferenceHelper::Event> event, NiChanges *ni)
{
  // according to the PRK paper
  
  m_perDetails.clear ();
  double noiseInterferenceW = 0.0;
  double snr = 0.0; 
  double pdr = 1.0;
  NiChanges::iterator it = ni->begin ();
  Time previous = (*it).GetTime ();
  WifiMode payloadMode = event->GetPayloadMode ();
  WifiPreamble preamble = event->GetPreambleType ();
  WifiMode headerMode = WifiPhy::GetPlcpHeaderMode (payloadMode, preamble);
  Time plcpHeaderStart = (*it).GetTime () + MicroSeconds (WifiPhy::GetPlcpPreambleDurationMicroSeconds (payloadMode, preamble));
  Time plcpPayloadStart = plcpHeaderStart + MicroSeconds (WifiPhy::GetPlcpHeaderDurationMicroSeconds (payloadMode, preamble));
  noiseInterferenceW = (*it).GetDelta (); //according to CalculateNoiseInterferenceW, the delta value of the first item in the vector ni is equal to m_firstPower.
  if (Simulator::Now () > Simulator::LearningTimeDuration )
  {
    //std::cout<<" payload phy rate: "<<payloadMode.GetPhyRate ()<<" header phy rate: "<< headerMode.GetPhyRate () << std::endl;
  }
  m_perDetails.append (" GetDelta (): ");
  m_perDetails.append (ToString (it->GetDelta ()));
  it ++;
  double powerW = event->GetRxPowerW ();
  for (; it != ni->end (); ++ it)
  {
    if ( noiseInterferenceW < 0)
    {
      noiseInterferenceW = 0; // the noise interference in Watt should never be negative.  The negative value of @noiseInterferenceW is caused by the inaccuracy float point number calculation.
    }
    Time current = it->GetTime (); 
    Time duration;
    uint32_t rate;
    uint64_t nbits;
    uint32_t nBytes;
    if (previous >= plcpPayloadStart )
    { 
      rate = payloadMode.GetPhyRate ();
      duration = current - previous;
      nbits = (uint64_t)(rate * duration.GetSeconds ());
      nBytes = nbits%8!=0? 1 + nbits/8: nbits/8; 
      double tempPdr = 0.0; 

      snr = CalculateSnr (powerW, noiseInterferenceW, payloadMode);
      //snr = 10.0 * log10 (snr);
      tempPdr = ExpectedPdr(snr, nBytes);
      pdr *= tempPdr>1?1:tempPdr;
      m_perDetails.append (" &&_N+I: ");
      m_perDetails.append (ToString(noiseInterferenceW));
      m_perDetails.append (" csr: ");
      m_perDetails.append (ToString (tempPdr));
      m_perDetails.append (" snr: ");
      m_perDetails.append (ToString (10*log10(snr)));
      m_perDetails.append (" nBytes: ");
      m_perDetails.append (ToString (nBytes));
      m_perDetails.append (" current_pdr: ");
      m_perDetails.append (ToString (pdr));
    }
    else if ( previous >= plcpHeaderStart)
    {
      if ( current >= plcpPayloadStart )
      {
        rate = headerMode.GetPhyRate ();
        duration = plcpPayloadStart - previous;
        nbits = (uint64_t)(rate * duration.GetSeconds ());
        nBytes = nbits%8!=0? 1 + nbits/8: nbits/8;
        double tempPdr = 0.0; 
         
        snr = CalculateSnr (powerW, noiseInterferenceW, headerMode);
        //snr = 10.0 * log10 (snr);
        tempPdr = ExpectedPdr(snr,  nBytes);
        pdr *= tempPdr>1?1:tempPdr;
        m_perDetails.append (" &&_N+I: ");
        m_perDetails.append (ToString(noiseInterferenceW));
        m_perDetails.append (" csr: ");
        m_perDetails.append (ToString (tempPdr));
        m_perDetails.append (" snr: ");
        m_perDetails.append (ToString (10*log10(snr)));
        m_perDetails.append (" nBytes: ");
        m_perDetails.append (ToString (nBytes));
        m_perDetails.append (" current_pdr: ");
        m_perDetails.append (ToString (pdr));

        rate = payloadMode.GetPhyRate ();
        duration = current - plcpPayloadStart;
        nbits = (uint64_t)(rate*duration.GetSeconds ());
        nBytes = nbits%8!=0? 1 + nbits/8: nbits/8;

        snr = CalculateSnr (powerW, noiseInterferenceW, payloadMode);
        //snr = 10.0 * log10 (snr);
        tempPdr = ExpectedPdr(snr, nBytes);
        pdr *= tempPdr>1?1:tempPdr;
        m_perDetails.append (" &&_N+I: ");
        m_perDetails.append (ToString(noiseInterferenceW));
        m_perDetails.append (" csr: ");
        m_perDetails.append (ToString (tempPdr));
        m_perDetails.append (" snr: ");
        m_perDetails.append (ToString (10*log10(snr)));
        m_perDetails.append (" nBytes: ");
        m_perDetails.append (ToString (nBytes));
        m_perDetails.append (" current_pdr: ");
        m_perDetails.append (ToString (pdr));
      }
      else if (current >= plcpHeaderStart)
      {
        rate = headerMode.GetPhyRate ();
        duration = plcpPayloadStart - previous;
        nbits = (uint64_t)(rate * duration.GetSeconds ());
        nBytes = nbits%8!=0? 1 + nbits/8: nbits/8; // get the n-bytes value
        double tempPdr = 0.0; 
        
        snr = CalculateSnr (powerW, noiseInterferenceW, headerMode);
        //snr = 10.0 * log10 (snr);
        tempPdr = ExpectedPdr(snr, nBytes);
        pdr *= tempPdr>1?1:tempPdr;
        m_perDetails.append (" &&_N+I: ");
        m_perDetails.append (ToString(noiseInterferenceW));
        m_perDetails.append (" csr: ");
        m_perDetails.append (ToString (tempPdr));
        m_perDetails.append (" snr: ");
        m_perDetails.append (ToString (10*log10(snr)));
        m_perDetails.append (" nBytes: ");
        m_perDetails.append (ToString (nBytes));
        m_perDetails.append (" current_pdr: ");
        m_perDetails.append (ToString (pdr));
      }
    }
    else 
    {
      if (current >= plcpPayloadStart)
      {
        rate = headerMode.GetPhyRate ();
        duration = plcpPayloadStart - previous;
        nbits = (uint64_t)(rate * duration.GetSeconds ());
        nBytes = nbits%8!=0? 1 + nbits/8: nbits/8; // get the n-bytes value
        double tempPdr = 0.0; 
        
        snr = CalculateSnr (powerW, noiseInterferenceW, headerMode);
        //snr = 10.0 * log10 (snr);
        tempPdr = ExpectedPdr(snr, nBytes);
        pdr *= tempPdr>1?1:tempPdr;
        m_perDetails.append (" &&_N+I: ");
        m_perDetails.append (ToString(noiseInterferenceW));
        m_perDetails.append (" csr: ");
        m_perDetails.append (ToString (tempPdr));
        m_perDetails.append (" snr: ");
        m_perDetails.append (ToString (10*log10(snr)));
        m_perDetails.append (" nBytes: ");
        m_perDetails.append (ToString (nBytes));
        m_perDetails.append (" current_pdr: ");
        m_perDetails.append (ToString (pdr));

        rate = payloadMode.GetPhyRate ();
        duration = current - plcpPayloadStart;
        nbits = (uint64_t)(rate*duration.GetSeconds ());
        nBytes = nbits%8!=0? 1 + nbits/8: nbits/8;
        
        snr = CalculateSnr (powerW, noiseInterferenceW, payloadMode);
        //snr = 10.0 * log10 (snr);
        tempPdr = ExpectedPdr(snr, nBytes);
        pdr *= tempPdr>1?1:tempPdr;
        m_perDetails.append (" &&_N+I: ");
        m_perDetails.append (ToString(noiseInterferenceW));
        m_perDetails.append (" csr: ");
        m_perDetails.append (ToString (tempPdr));
        m_perDetails.append (" snr: ");
        m_perDetails.append (ToString (10*log10(snr)));
        m_perDetails.append (" nBytes: ");
        m_perDetails.append (ToString (nBytes));
        m_perDetails.append (" current_pdr: ");
        m_perDetails.append (ToString (pdr));
      }
      else if (current >= plcpHeaderStart)
      {
        rate = headerMode.GetPhyRate ();
        duration = plcpPayloadStart - previous;
        nbits = (uint64_t)(rate * duration.GetSeconds ());
        nBytes = nbits%8!=0? 1 + nbits/8: nbits/8; // get the n-bytes value
        double tempPdr = 0.0; 
        
        snr = CalculateSnr (powerW, noiseInterferenceW, headerMode);
        //snr = 10.0 * log10 (snr);
        tempPdr = ExpectedPdr(snr, nBytes);
        pdr *= tempPdr>1?1:tempPdr;
        m_perDetails.append (" &&_N+I: ");
        m_perDetails.append (ToString(noiseInterferenceW));
        m_perDetails.append (" csr: ");
        m_perDetails.append (ToString (tempPdr));
        m_perDetails.append (" snr: ");
        m_perDetails.append (ToString (10*log10(snr)));
        m_perDetails.append (" nBytes: ");
        m_perDetails.append (ToString (nBytes));
        m_perDetails.append (" current_pdr: ");
        m_perDetails.append (ToString (pdr));
      }
    }
    // after calculating the current SINR duration, go to next
    noiseInterferenceW += it->GetDelta (); 
    m_perDetails.append (" GetDelta (): ");
    m_perDetails.append (ToString (it->GetDelta ()));
    previous = current;
  }
  return 1-pdr; // this is the PER
}


struct InterferenceHelper::SnrPer
InterferenceHelper::CalculateSnrPer (Ptr<InterferenceHelper::Event> event)
{
  NiChanges ni;
  double noiseInterferenceW = CalculateNoiseInterferenceW (event, &ni);
  double snr = CalculateSnr (event->GetRxPowerW (),
                             noiseInterferenceW,
                             event->GetPayloadMode ());

  /* calculate the SNIR at the start of the packet and accumulate
   * all SNIR changes in the snir vector.
   */
  double per = CalculatePer (event, &ni);

  struct SnrPer snrPer;
  snrPer.snr = snr;
  snrPer.per = per;
  /*
  if (Simulator::Now () >= Simulator::LearningTimeDuration )
  {
    std::cout<<"in calculatesnrper: " <<" snr: "<<snr<<" per: "<< per<< std::endl;
  }
  */
  return snrPer;
}

void
InterferenceHelper::EraseEvents (void)
{
  m_niChanges.clear ();
  m_rxing = false;
  m_firstPower = 0.0;
  m_concurrency  = 0; // reset the number of concurrent transmitters
}

InterferenceHelper::NiChanges::iterator
InterferenceHelper::GetPosition (Time moment)
{
  return std::upper_bound (m_niChanges.begin (), m_niChanges.end (), NiChange (moment, 0));

}
void
InterferenceHelper::AddNiChangeEvent (NiChange change)
{
  m_niChanges.insert (GetPosition (change.GetTime ()), change);
}
void
InterferenceHelper::NotifyRxStart ()
{
  m_rxing = true;
}

void InterferenceHelper::UpdateReceivedPowerW (double rxPowerW)
{
  m_receivedPowerW = rxPowerW;
}

void
InterferenceHelper::NotifyRxEnd ()
{
  m_rxing = false;
}



/*
 * param gamma_0e SNR value when PRR is 0.001
 * param gamma_1e SNR value when PRR is 0.999
 * param gamma_A  SNR value when PRR is 0.1
 * param gamma_B  SNR value when PRR is 0.9
 * param receivedPower the received power at the receiver side. In Xin's version, he passed the distance,
 * however, in NS-3, there is method calculating received power according to node mobility model.
 * Hence, here I passed the receiver power. Xin used the distance parameter to calculate the receive power.
 * param noise the background noise at the receiver side.
 * \param snr ratio between two watt-variable
 */
double InterferenceHelper::ExpectedPdr(double snr, uint32_t length)
{
  if (length == 0)
  {
    return 1.0;
  }
  /*
  double gamma_0e = m_mathHelper.PdrToGamma (0.001, length), gamma_1e = m_mathHelper.PdrToGamma (0.999, length), 
         gamma_A = m_mathHelper.PdrToGamma (0.1, length), gamma_B = m_mathHelper.PdrToGamma (0.9, length);
  double m_ge = (m_mathHelper.NormPdf (gamma_1e, snr, sigma) - m_mathHelper.NormPdf (gamma_0e, snr, sigma) )
    / (gamma_1e - gamma_0e);
  double b_ge = (m_mathHelper.NormPdf (gamma_0e, snr, sigma) * gamma_1e - m_mathHelper.NormPdf (gamma_1e,
        snr , sigma) * gamma_0e) / (gamma_1e - gamma_0e);
  double m_e = (0.9 - 0.1)/ (gamma_B - gamma_A);
  double b_e = (0.1 * gamma_B - 0.9 * gamma_A) / (gamma_B - gamma_A);
  //double Q = m_mathHelper.NormCdf ( (snr - gamma_1e) / sigma); // get the cdf according to standard normal distribution

  double pdr = ((m_e * m_ge) * pow (gamma_1e, 3) / 3 + (b_e * m_ge + b_ge * m_e) * pow (gamma_1e, 2) /
      2 + b_e * b_ge * gamma_1e) - ((m_e * m_ge) * pow (gamma_0e, 3) / 3 + (b_e * m_ge + b_ge * m_e) *
      pow (gamma_0e, 2) / 2 + b_e * b_ge * gamma_0e) + m_mathHelper.NormCdf ((snr - gamma_1e) / sigma);
  */
  double B_N = 2000;
  double R=250*8;
  double ber = m_mathHelper.gsl_cdf_ugaussian_Q ( sqrt (snr * 2 * B_N/R));
  double pdr = pow ((1-ber), length);
  return pdr > 1.0 ? 1 : pdr;
}

/* set the MAC address of the current node
 */
void InterferenceHelper::SetAddress (Mac48Address addr)
{
  m_self = addr;
}

/* get the MAC address of the current node
 */
Mac48Address InterferenceHelper::GetAddress () const
{
  return m_self;
}

/* General interference sampling: No matter the node is receiving or transmitting or neithor, we do periodic interference sampling
 * to get a rough understanding of how much the N+I is.
 */
void InterferenceHelper::DoSampleInterference ()
{
  if ( Simulator::Now () >= m_simulationEndTime )
  {
    return;
  }
  if ( Simulator::Now () >= Simulator::LearningTimeDuration )
  {
    if ((m_niChanges.size () > 0) && (m_niChanges.begin ()->GetDelta () == m_firstPower ))
    {
      double interferenceDuringPacketReception = 0;
      NiChanges::iterator nowIterator = GetPosition (Simulator::Now ());
      for (NiChanges::iterator i = m_niChanges.begin (); i != nowIterator; i++)
      {
        interferenceDuringPacketReception += i->GetDelta ();
        if (interferenceDuringPacketReception < 0)
        {
          interferenceDuringPacketReception = 0;
        }
      }
      m_interferenceSamples.push_back (interferenceDuringPacketReception);
    }
    else
    {
      m_interferenceSamples.push_back (m_firstPower);
    }

    /*
    if ( m_interferenceSamples.size () == 10000)
    {
      std::vector<double>::iterator it = m_interferenceSamples.begin ();
      double sum = 0.0;
      for ( ; it != m_interferenceSamples.end (); ++ it)
      {
        sum += *it;
      }
      // before return the value, clear the vector for future use.
      m_interferenceW =  sum / m_interferenceSamples.size ();
      m_interferenceSamples.clear ();
    }
    */

    Simulator::Schedule (m_sampleInterval, &InterferenceHelper::DoSampleInterference, this);
  }
  else
  {
    Simulator::Schedule ( m_sampleInterval, &InterferenceHelper::DoSampleInterference, this);
  }
}

/* According to the sampled interference, calculate the average value of them and treat the calculated value as the 
 * background interference plus noise (N+I) value
 */
void InterferenceHelper::ComputeSampledInterferenceW ()
{
  std::vector<double>::iterator it = m_interferenceSamples.begin ();
  double sum = 0;
  for (; it != m_interferenceSamples.end (); ++ it)
  {
    //sum = m_ewmaCoefficient * sum + (1 - m_ewmaCoefficient) * (*it);
    sum += (*it);
  }
  // before return the value, clear the vector for future use.
  sum =  sum / m_interferenceSamples.size (); //take average value
  if ( m_interferenceSamples.size () > 5000 )
  {
    m_interferenceW = sum;
    //std::cout<< "m_interferenceW: "<< m_interferenceW << " sum: "<< sum << " size: "<< m_interferenceSamples.size ()<<std::endl;
    m_interferenceSamples.clear ();
  }
}

/* When the current node is receiving data packet, start sampling N+I
 */
void InterferenceHelper::NotifyDataStartReceiving ()
{
  m_niDataReceivingSampling = true;
  SampleInterferenceWhileReceivingData ();
}

void InterferenceHelper::ControlChannel_NotifyDataStartReceiving ()
{
  m_niDataReceivingSampling_ControlChannel = true;
  ControlChannel_SampleInterferenceWhileReceivingData ();
}
/* When the current stops receiving data packet, also stop interference sampling
 */
void InterferenceHelper::NotifyDataEndReceiving ()
{
  m_niDataReceivingSampling = false;
}

void InterferenceHelper::ControlChannel_NotifyDataEndReceiving ()
{
  m_niDataReceivingSampling_ControlChannel = false;
}
void InterferenceHelper::NotifyAckStartReceiving ()
{
  m_niAckReceivingSampling = true;
  SampleInterferenceWhileReceivingAck ();
}
void InterferenceHelper::NotifyAckEndReceiving ()
{
  m_niAckReceivingSampling = false;
}

/* 
 */
void InterferenceHelper::SampleInterferenceWhileReceivingData ( )
{
  if ( Simulator::Now () >= m_simulationEndTime || m_niDataReceivingSampling == false)
  {
    return;
  }
  if ( m_niDataReceivingSampling == true && Simulator::Now () >= Simulator::LearningTimeDuration && m_niChanges.size () > 0)
  {
    double rxPowerW = m_niChanges.begin ()->GetDelta ();
    double interferenceDuringPacketReception = m_firstPower;
    for (NiChanges::iterator i = m_niChanges.begin ()+1; i != m_niChanges.end () && i->GetDelta () != -rxPowerW; i++)
    {
      if (i->GetTime () > Simulator::Now () )
      {
        break;
      }
      interferenceDuringPacketReception += i->GetDelta ();
      if (interferenceDuringPacketReception < 0)
      {
        interferenceDuringPacketReception = 0;
      }
    }

    if ( m_meanDataInterferenceW == 0)
    {
      m_meanDataInterferenceW = interferenceDuringPacketReception;
    }
    else
    {
      if ( interferenceDuringPacketReception > m_meanDataInterferenceW + NI_SAMPLE_FILTER * sqrt (m_varianceDataInterferenceW)||
          interferenceDuringPacketReception > ABSOLUTE_NI_THRESHOLD)
      {
      }
      else
      {
        double difference = interferenceDuringPacketReception - m_meanDataInterferenceW;
        double increment = m_ewmaCoefficient * difference;
        m_meanDataInterferenceW = m_meanDataInterferenceW + difference;
        m_varianceDataInterferenceW = (1 - m_ewmaCoefficient) * (m_varianceDataInterferenceW + difference * increment);
        m_dataInterferenceW = m_meanDataInterferenceW;
      }
    }
/*
    m_dataInterferenceSamples.push_back (interferenceDuringPacketReception);
    if ( m_dataInterferenceSamples.size () > MAX_INTERFERENCE_SAMPLE_SIZE )
    {
      double sum = 0;
      for (std::vector<double>::iterator it = m_dataInterferenceSamples.begin (); it != m_dataInterferenceSamples.end (); ++ it)
      {
        sum += *it;
      }
      m_dataInterferenceW = (1 - m_ewmaCoefficient) * (sum / (double)m_dataInterferenceSamples.size ()) + m_ewmaCoefficient * m_dataInterferenceW;
      m_dataInterferenceSamples.clear ();
    }
*/
  }
  Simulator::Schedule (m_dataInterferenceSampleInterval, &InterferenceHelper::SampleInterferenceWhileReceivingData, this);
}

void InterferenceHelper::ControlChannel_SampleInterferenceWhileReceivingData ( )
{
  if ( Simulator::Now () >= m_simulationEndTime || m_niDataReceivingSampling_ControlChannel == false)
  {
    return;
  }
  if ( m_niDataReceivingSampling_ControlChannel == true && Simulator::Now () >= Simulator::LearningTimeDuration && 
      m_niChanges.size () > 0)
  {
    double rxPowerW = m_niChanges.begin ()->GetDelta ();
    double interferenceDuringPacketReception = m_firstPower;
    //std::cout<<std::endl<<" printing nichanges: "<<" m_firstPower: "<<m_firstPower<< std::endl;
    for (NiChanges::iterator i = m_niChanges.begin ()+1; i != m_niChanges.end () && i->GetDelta () != -rxPowerW; i++)
    {
      if (i->GetTime () > Simulator::Now () )
      {
        break;
      }
      interferenceDuringPacketReception += i->GetDelta ();
      if (interferenceDuringPacketReception < 0)
      {
        interferenceDuringPacketReception = 0;
      }
    }
    m_dataInterferenceSamples_ControlChannel.push_back (interferenceDuringPacketReception);
    if ( m_dataInterferenceSamples_ControlChannel.size () == MAX_INTERFERENCE_SAMPLE_SIZE )
    {
      double quantileValue  = 0;
      double firstSample = *(m_dataInterferenceSamples_ControlChannel.begin ());
      #ifdef NI_QUANTILE_ESTIMATION_EWMA
      quantileValue = FindQuantileValue (NI_SAMPLE_QUANTILE, firstSample, interferenceDuringPacketReception, m_dataInterferenceSamples_ControlChannel);
      m_controlChannelInterferenceW = (1 - m_ewmaCoefficient) * quantileValue + m_ewmaCoefficient * m_controlChannelInterferenceW;
      m_dataInterferenceSamples_ControlChannel.clear ();
      #endif

      #ifdef NI_QUANTILE_ESTIMATION_SLIDING_WINDOW
      m_dataInterferenceSamples_ControlChannel.erase (m_dataInterferenceSamples_ControlChannel.begin ());
      quantileValue = FindQuantileValue (NI_SAMPLE_QUANTILE, firstSample, interferenceDuringPacketReception, m_dataInterferenceSamples_ControlChannel);
      m_controlChannelInterferenceW = quantileValue;
      #endif

      #ifdef NI_QUANTILE_ESTIMATION_SLIDING_EWMA
      m_dataInterferenceSamples_ControlChannel.erase (m_dataInterferenceSamples_ControlChannel.begin ());
      quantileValue = FindQuantileValue (NI_SAMPLE_QUANTILE, firstSample, interferenceDuringPacketReception, m_dataInterferenceSamples_ControlChannel);
      m_controlChannelInterferenceW = (1 - m_ewmaCoefficient) * quantileValue + m_ewmaCoefficient * m_controlChannelInterferenceW;
      #endif
    }  
  }
  Simulator::Schedule (m_controlInterferenceSampleInterval, &InterferenceHelper::ControlChannel_SampleInterferenceWhileReceivingData, this);
}
void InterferenceHelper::SampleInterferenceWhileReceivingAck ()
{
  if ( Simulator::Now () >= m_simulationEndTime || m_niAckReceivingSampling == false)
  {
    return;
  }
  if ( m_niAckReceivingSampling == true && Simulator::Now () >= Simulator::LearningTimeDuration && m_niChanges.size () > 0)
  {
    double rxPowerW = m_niChanges.begin ()->GetDelta ();
    double interferenceDuringPacketReception = m_firstPower;
    for (NiChanges::iterator i = m_niChanges.begin ()+1; i != m_niChanges.end () && i->GetDelta () != -rxPowerW; i++)
    {
      if (i->GetTime () > Simulator::Now () )
      {
        break;
      }
      interferenceDuringPacketReception += i->GetDelta ();
      if (interferenceDuringPacketReception < 0)
      {
        interferenceDuringPacketReception = 0;
      }
    }

    if ( m_meanAckInterferenceW == 0)
    {
      m_meanAckInterferenceW = interferenceDuringPacketReception;
    }
    else
    {
      if ( interferenceDuringPacketReception > m_meanAckInterferenceW + NI_SAMPLE_FILTER * sqrt (m_varianceAckInterferenceW) ||
          interferenceDuringPacketReception > ABSOLUTE_NI_THRESHOLD)
      {
      }
      else 
      {
        double difference = interferenceDuringPacketReception - m_meanAckInterferenceW;
        double increment = m_ewmaCoefficient * difference;
        m_meanAckInterferenceW = m_meanAckInterferenceW + difference;
        m_varianceAckInterferenceW = (1 - m_ewmaCoefficient) * (m_varianceAckInterferenceW + difference * increment);
        m_ackInterferenceW = m_meanAckInterferenceW;
      }
    }
    /* 
    m_ackInterferenceSamples.push_back (interferenceDuringPacketReception);
    if ( m_ackInterferenceSamples.size () > MAX_INTERFERENCE_SAMPLE_SIZE)
    {
      double sum = 0;
      for (std::vector<double>::iterator it = m_ackInterferenceSamples.begin (); it != m_ackInterferenceSamples.end (); ++ it)
      {
        sum += *it;
      }
      m_ackInterferenceW = (1 - m_ewmaCoefficient) * (sum / (double)m_ackInterferenceSamples.size ()) + m_ewmaCoefficient * m_ackInterferenceW;
      m_ackInterferenceSamples.clear ();
    }
    */ 
  }
  Simulator::Schedule (m_ackInterferenceSampleInterval, &InterferenceHelper::SampleInterferenceWhileReceivingAck, this);
}

/* Compute the sampled interference value when the current node is receiving data.
 * This value is important for controller behavior
 */
double InterferenceHelper::ComputeInterferenceWhenReceivingData ()
{
  return m_dataInterferenceW;
}

double InterferenceHelper::ControlChannel_ComputeInterferenceWhenReceivingData ()
{
  return m_controlChannelInterferenceW;
}
double InterferenceHelper::ComputeInterferenceWhenReceivingAck ()
{
  return m_ackInterferenceW;
}
template <typename T>
std::string InterferenceHelper::ToString (T const &val)
{
  std::stringstream sstr;
  sstr << val;
  return sstr.str();
}

/* Return the details of a packet reception activity
 */
std::string InterferenceHelper::GetPerDetails () const
{
  return m_perDetails;
}

double InterferenceHelper::FindQuantileValue (double quantile, double removeValue, double addValue, std::vector<double> vec)
{
  if (vec.size () == 0)
  {
    return 0;
  }
  #ifdef NI_QUANTILE_ESTIMATION_EWMA
  m_dataInterferenceSamples_ControlChannel_Copy = vec; //copy
  #else 
  for (std::vector<double>::iterator it = m_dataInterferenceSamples_ControlChannel_Copy.begin (); it != m_dataInterferenceSamples_ControlChannel_Copy.end (); ++ it)
  {
    if (*it == removeValue)
    {
      m_dataInterferenceSamples_ControlChannel_Copy.erase (it);
      break;
    }
  }
  m_dataInterferenceSamples_ControlChannel_Copy.push_back (addValue);
  #endif
  uint32_t index = (uint32_t) (quantile * m_dataInterferenceSamples_ControlChannel_Copy.size ());
  index = index == 0 ? index : index - 1; // index starts from 0 to n-1, minus 1
  QuickSort quick(&m_dataInterferenceSamples_ControlChannel_Copy); // asending sort
  return m_dataInterferenceSamples_ControlChannel_Copy[index];
}
} // namespace ns3
