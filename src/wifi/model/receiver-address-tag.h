#ifndef RECEIVER_ADDRESS_TAG_H
#define RECEIVER_ADDRESS_TAG_H 

#include "ns3/type-id.h"
#include "ns3/tag.h"
#include "ns3/mac48-address.h"

namespace ns3
{
  class ReceiverAddressTag : public Tag
  {
  public:
    ReceiverAddressTag ();
    ~ReceiverAddressTag ();
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;

    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual void Print (std::ostream &os) const;

    void Set (Mac48Address addr);
    Mac48Address Get (void) const;
  private:
    Mac48Address m_addr;
  };
}

#endif //RECEIVER_ADDRESS_TAG_H
