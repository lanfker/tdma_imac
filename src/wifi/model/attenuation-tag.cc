#include "attenuation-tag.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("AttenuationTag");

namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (AttenuationTag);

  TypeId AttenuationTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::AttenuationTag")
      .SetParent<Tag> ()
      .AddConstructor<AttenuationTag> ()
      .AddAttribute("Attenuation",
                    "record the outbound attenuation of the packet's destination node",
                    DoubleValue(0.0),
                    MakeDoubleAccessor (&AttenuationTag::GetAttenuation),
                    MakeDoubleChecker<double> ());
    return tid;
  }

  TypeId AttenuationTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t AttenuationTag::GetSerializedSize (void) const
  {
    return sizeof(double);
  }

  void AttenuationTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_attenuation);
  }

  void AttenuationTag::Deserialize (TagBuffer i)
  {
    m_attenuation = i.ReadDouble ();
  }

  void AttenuationTag::Print (std::ostream &os) const
  {
    os << "attenuation =: " << m_attenuation;
  }

  double AttenuationTag::GetAttenuation (void) const
  {
    return m_attenuation;
  }


  void AttenuationTag::SetAttenuation (double value)
  {
    m_attenuation = value;
  }
}
