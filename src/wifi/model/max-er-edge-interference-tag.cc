#include "max-er-edge-interference-tag.h"
#include "ns3/log.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("MaxErEdgeInterferenceTag");

/* This class represents the tag of  maximum interference power that the receiver could 
 * tolerent.
 *
 */
namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (MaxErEdgeInterferenceTag);

  TypeId MaxErEdgeInterferenceTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::MaxErEdgeInterferenceTag")
      .SetParent<Tag> ()
      .AddConstructor<MaxErEdgeInterferenceTag> ()
      .AddAttribute("Interference",
                    "The maximum interference power the receiver can tolerant",
                    DoubleValue(0.0),
                    MakeDoubleAccessor (&MaxErEdgeInterferenceTag::Get),
                    MakeDoubleChecker<double> ());
    return tid;
  }

  TypeId MaxErEdgeInterferenceTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t MaxErEdgeInterferenceTag::GetSerializedSize (void) const
  {
    return sizeof(double);
  }

  void MaxErEdgeInterferenceTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_interference);
  }

  void MaxErEdgeInterferenceTag::Deserialize (TagBuffer i)
  {
    m_interference = i.ReadDouble ();
  }

  void MaxErEdgeInterferenceTag::Print (std::ostream &os) const
  {
    os << "interference =: " << m_interference;
  }

  double MaxErEdgeInterferenceTag::Get (void) const
  {
    return m_interference;
  }


  void MaxErEdgeInterferenceTag::Set (double value)
  {
    m_interference = value;
  }
}
