#include "ack-sequence-no-tag.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"
#include "ns3/integer.h"

NS_LOG_COMPONENT_DEFINE ("AckSequenceNoTag");

namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (AckSequenceNoTag);

  TypeId AckSequenceNoTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::AckSequenceNoTag")
      .SetParent<Tag> ()
      .AddConstructor<AckSequenceNoTag> ()
      .AddAttribute("AckSequenceNo",
                    "record the ack sequence number of an out going ack packet",
                    IntegerValue(0),
                    MakeIntegerAccessor (&AckSequenceNoTag::Get),
                    MakeIntegerChecker<uint16_t> ());
    return tid;
  }

  TypeId AckSequenceNoTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t AckSequenceNoTag::GetSerializedSize (void) const
  {
    return sizeof(uint16_t);
  }

  void AckSequenceNoTag::Serialize (TagBuffer i) const
  {
    i.WriteU16 (m_ackNumber);
  }

  void AckSequenceNoTag::Deserialize (TagBuffer i)
  {
    m_ackNumber = i.ReadU16 ();
  }

  void AckSequenceNoTag::Print (std::ostream &os) const
  {
    os << "ackNumber =: " << m_ackNumber;
  }

  uint16_t AckSequenceNoTag::Get (void) const
  {
    return m_ackNumber;
  }


  void AckSequenceNoTag::Set (uint16_t value)
  {
    m_ackNumber = value;
  }
}
