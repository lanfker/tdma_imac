#ifndef OUT_BOUND_SNR_TAG_H
#define OUT_BOUND_SNR_TAG_H 

#include "ns3/type-id.h"
#include "ns3/tag.h"


namespace ns3
{
  class OutBoundSnrTag : public Tag
  {
  public:
    OutBoundSnrTag ();
    ~OutBoundSnrTag ();
    static TypeId GetTypeId (void);
    virtual TypeId GetInstanceTypeId (void) const;

    virtual uint32_t GetSerializedSize (void) const;
    virtual void Serialize (TagBuffer i) const;
    virtual void Deserialize (TagBuffer i);
    virtual void Print (std::ostream &os) const;

    void Set (double snr);
    double Get (void) const;
  private:
    double m_snr;
  };
}

#endif //OUT_BOUND_SNR_TAG_H
