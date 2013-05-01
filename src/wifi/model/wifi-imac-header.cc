#include "wifi-imac-header.h"
#include "wifi-mac-header.h"
#include "ns3/log.h"
#include "ns3/address-utils.h"


namespace ns3
{
NS_LOG_COMPONENT_DEFINE("WifiImacHeader");

NS_OBJECT_ENSURE_REGISTERED(WifiImacHeader);

TypeId WifiImacHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::WifiImacHeader")
    .SetParent<Header> ()
    .AddConstructor<WifiImacHeader> ()
  ;
  return tid;
}

WifiImacHeader::WifiImacHeader (WifiMacHeader hdr)
{
  m_hdr = hdr;  
}
WifiImacHeader::WifiImacHeader()
{
}

WifiMacHeader WifiImacHeader::GetInternalWifiMacHeader() const
{
  return m_hdr;
}

uint32_t WifiImacHeader::GetSerializedSize (void) const
{
  return  m_hdr.GetSerializedSize () + sizeof(uint64_t);
}

void WifiImacHeader::Serialize (Buffer::Iterator start) const
{
  m_hdr.Serialize(start);

  Buffer::Iterator it = start;
  uint8_t* tempBuffer = new uint8_t[m_hdr.GetSize()];
  it.Read(tempBuffer ,m_hdr.GetSize());

  it.WriteHtonU64 (m_attenuation);
  delete [] tempBuffer;

  //Besides the default wifi mac header, also write the attenuation number into the buffer.

}

uint32_t WifiImacHeader::Deserialize (Buffer::Iterator start)
{
  uint32_t i = m_hdr.Deserialize (start);
  Buffer::Iterator it = start;
  uint8_t* tempBuffer = new uint8_t[i];
  it.Read(tempBuffer ,i);
  m_attenuation = it.ReadNtohU64 ();
  std::cout<<"atten: "<<m_attenuation<<std::endl;
  delete [] tempBuffer;
  return it.GetDistanceFrom(start); // sizeof(double) is 8
}
void WifiImacHeader::Print (std::ostream &os) const
{
  m_hdr.Print (os);
  os << ", Sender's outBoundAttenuation="<<m_attenuation<<std::endl;
}


WifiImacHeader::~WifiImacHeader()
{
}

TypeId WifiImacHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint64_t WifiImacHeader::GetAttenuation()
{
  return m_attenuation;
}

void WifiImacHeader::SetAttenuation(uint64_t attenuation)
{
  m_attenuation = attenuation;
}

}
