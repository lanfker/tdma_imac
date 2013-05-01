#include "receiver-address-tag.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag.h"
#include "ns3/log.h"
//#include "ns3/integer.h"

NS_LOG_COMPONENT_DEFINE ("ReceiverAddressTag");

namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (ReceiverAddressTag);

  ReceiverAddressTag::ReceiverAddressTag ()
  {
    NS_LOG_FUNCTION (this);
  }
  ReceiverAddressTag::~ReceiverAddressTag ()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  ReceiverAddressTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::ReceiverAddressTag")
      .SetParent<Tag> ()
      .AddConstructor<ReceiverAddressTag> ()
      .AddAttribute ("ReceiverAddressTag", "The address of the node which sends the cts message",
                     Mac48AddressValue (Mac48Address("ff:ff:ff:ff:ff:ff")),
                     MakeMac48AddressAccessor (&ReceiverAddressTag::Get),
                     MakeMac48AddressChecker ())
    ;
    return tid;
  }
  TypeId
  ReceiverAddressTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  ReceiverAddressTag::GetSerializedSize (void) const
  {
    return 6;
  }
  void
  ReceiverAddressTag::Serialize (TagBuffer i) const
  {
    i.Write ( (uint8_t *)&m_addr, 6);
    //i.WriteU64 (m_addr);
  }
  void
  ReceiverAddressTag::Deserialize (TagBuffer i)
  {
    i.Read ((uint8_t *)&m_addr, 6);
    m_addr = (Mac48Address)m_addr;
  }
  void
  ReceiverAddressTag::Print (std::ostream &os) const
  {
    os << "addr= " << m_addr;
  }
  void
  ReceiverAddressTag::Set (Mac48Address addr)
  {
    m_addr = addr; 
  }
  Mac48Address
  ReceiverAddressTag::Get (void) const
  {
    return m_addr;
  }

}
