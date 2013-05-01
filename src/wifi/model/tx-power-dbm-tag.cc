#include "tx-power-dbm-tag.h"
#include "ns3/object.h"
#include "ns3/type-id.h"
#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/double.h"

NS_LOG_COMPONENT_DEFINE ("TxPowerDbmTag");

namespace ns3
{
  NS_OBJECT_ENSURE_REGISTERED (TxPowerDbmTag);

  TxPowerDbmTag::TxPowerDbmTag()
  {
    NS_LOG_FUNCTION (this);
  }
  TxPowerDbmTag::~TxPowerDbmTag()
  {
    NS_LOG_FUNCTION (this);
  }

  TypeId
  TxPowerDbmTag::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::TxPowerDbmTag")
      .SetParent<Tag> ()
      .AddConstructor<TxPowerDbmTag> ()
      .AddAttribute ("TxPowerDbm", "The transmission power dbm of the current packet (mainly used in RTS and CTS transmission)",
                     DoubleValue (0),
                     MakeDoubleAccessor (&TxPowerDbmTag::Get),
                     MakeDoubleChecker<double> ())
    ;
    return tid;
  }
  TypeId
  TxPowerDbmTag::GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  uint32_t
  TxPowerDbmTag::GetSerializedSize (void) const
  {
    return sizeof (double);
  }
  void
  TxPowerDbmTag::Serialize (TagBuffer i) const
  {
    i.WriteDouble (m_dbm);
    //i.Write ( (double *)&m_dbm, 1);
  }
  void
  TxPowerDbmTag::Deserialize (TagBuffer i)
  {
    m_dbm = i.ReadDouble ();
    //i.Read ( (double *)&m_dbm, 1);
  }
  void
  TxPowerDbmTag::Print (std::ostream &os) const
  {
    os << "txPowerLevel = " << m_dbm;
  }
  void
  TxPowerDbmTag::Set (double dbm)
  {
    m_dbm = dbm;
  }
  double 
  TxPowerDbmTag::Get (void) const
  {
    return m_dbm;
  }

}
