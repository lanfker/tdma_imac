#ifndef TX_POWER_DBM_TAG_H
#define TX_POWER_DBM_TAG_H 

#include "ns3/type-id.h"
#include "ns3/tag.h"


namespace ns3
{
  class TxPowerDbmTag: public Tag
  {
  public:
    TxPowerDbmTag();
    ~TxPowerDbmTag ();
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;

    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual void Print (std::ostream &os) const;

    void Set (double dbm);
    double Get (void) const;
  private:
    double m_dbm;
  };
}

#endif //TX_POWER_DBM_TAG_H
