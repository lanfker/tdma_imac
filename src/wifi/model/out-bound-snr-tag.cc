#include "out-bound-snr-tag.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("OutBoundSnrTag");

namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (OutBoundSnrTag);

  OutBoundSnrTag::OutBoundSnrTag ()
  {
    NS_LOG_FUNCTION (this);
  }
  OutBoundSnrTag::~OutBoundSnrTag ()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  OutBoundSnrTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::OutBoundSnrTag")
      .SetParent<Tag> ()
      .AddConstructor<OutBoundSnrTag> ()
      .AddAttribute ("OutBoundSnr", "The out-bound snr of the destination node of a packet",
                     DoubleValue (0.0),
                     MakeDoubleAccessor (&OutBoundSnrTag::Get),
                     MakeDoubleChecker<double> ())
    ;
    return tid;
  }
  TypeId
  OutBoundSnrTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  OutBoundSnrTag::GetSerializedSize (void) const
  {
    return sizeof (double);
  }
  void
  OutBoundSnrTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_snr);
  }
  void
  OutBoundSnrTag::Deserialize (TagBuffer i)
  {
    m_snr = i.ReadDouble ();
  }
  void
  OutBoundSnrTag::Print (std::ostream &os) const
  {
    os << "Snr=" << m_snr;
  }
  void
  OutBoundSnrTag::Set (double snr)
  {
    m_snr = snr;
  }
  double
  OutBoundSnrTag::Get (void) const
  {
    return m_snr;
  }

}
