#include "interference-tag.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("InterferenceTag");

/* This class represents the tag of  maximum interference power that the receiver could 
 * tolerent.
 *
 */
namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (InterferenceTag);

  TypeId InterferenceTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::InterferenceTag")
      .SetParent<Tag> ()
      .AddConstructor<InterferenceTag> ()
      .AddAttribute("Interference",
                    "The maximum interference power the receiver can tolerant",
                    DoubleValue(0.0),
                    MakeDoubleAccessor (&InterferenceTag::Get),
                    MakeDoubleChecker<double> ());
    return tid;
  }

  TypeId InterferenceTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t InterferenceTag::GetSerializedSize (void) const
  {
    return sizeof(double);
  }

  void InterferenceTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_interference);
  }

  void InterferenceTag::Deserialize (TagBuffer i)
  {
    m_interference = i.ReadDouble ();
  }

  void InterferenceTag::Print (std::ostream &os) const
  {
    os << "interference =: " << m_interference;
  }

  double InterferenceTag::Get (void) const
  {
    return m_interference;
  }


  void InterferenceTag::Set (double value)
  {
    m_interference = value;
  }
}
