#include "noise-plus-interference-tag.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("NoisePlusInterferenceTag");

/* This class represents the tag of  last noise + interference value at the packet sender side
 *
 */
namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (NoisePlusInterferenceTag);

  TypeId NoisePlusInterferenceTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::NoisePlusInterferenceTag")
      .SetParent<Tag> ()
      .AddConstructor<NoisePlusInterferenceTag> ()
      .AddAttribute("NoisePlusInterference",
                    "The maximum interference power the receiver can tolerant",
                    DoubleValue(0.0),
                    MakeDoubleAccessor (&NoisePlusInterferenceTag::Get),
                    MakeDoubleChecker<double> ());
    return tid;
  }

  TypeId NoisePlusInterferenceTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t NoisePlusInterferenceTag::GetSerializedSize (void) const
  {
    return sizeof(double);
  }

  void NoisePlusInterferenceTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_noisePlusInterference);
  }

  void NoisePlusInterferenceTag::Deserialize (TagBuffer i)
  {
    m_noisePlusInterference = i.ReadDouble ();
  }

  void NoisePlusInterferenceTag::Print (std::ostream &os) const
  {
    os << "interference =: " << m_noisePlusInterference;
  }

  double NoisePlusInterferenceTag::Get (void) const
  {
    return m_noisePlusInterference;
  }


  void NoisePlusInterferenceTag::Set (double value)
  {
    m_noisePlusInterference = value;
  }
}
