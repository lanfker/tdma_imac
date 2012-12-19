#ifndef WIFI_IMAC_HEADER_H
#define WIFI_IMAC_HEADER_H

#include "ns3/header.h"
#include "ns3/mac48-address.h"
#include "ns3/nstime.h"
#include "wifi-mac-header.h"
namespace ns3 {

class WifiImacHeader: public Header
{
public:
  /*
   *if A sends packet to B, B will maintain a signal attenuation from A to B
   *However, A does not know this attenuation. Signal map needs us maintain this 
   *attenuation, therefore, we piggyback this attenuation in the mac header of the packet 
   *that will send from B to A. In this way, A would know its attenuation to B
   */
  static TypeId GetTypeId(void);
  WifiImacHeader();
  WifiImacHeader (WifiMacHeader hdr);
  ~WifiImacHeader();
  void SetAttenuation (uint64_t attenuation);
  uint64_t GetAttenuation ();
  WifiMacHeader GetInternalWifiMacHeader() const;


  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;
private:
  uint64_t m_attenuation;
  WifiMacHeader m_hdr;
};
}
#endif
