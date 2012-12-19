#ifndef ACK_SEQUENCE_NO_TAG_H
#define ACK_SEQUENCE_NO_TAG_H

#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"


namespace ns3
{
class AckSequenceNoTag: public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  uint16_t Get () const;
  void Set (uint16_t value); 

  void Print (std::ostream &os) const;

private:
  uint16_t m_ackNumber;

};
}

#endif // ACK_SEQUENCE_NO_TAG_H
