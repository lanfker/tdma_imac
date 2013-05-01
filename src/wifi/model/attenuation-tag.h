#ifndef ATTENUATION_TAG_H
#define ATTENUATION_TAG_H 

#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"


namespace ns3
{
class AttenuationTag: public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  double GetAttenuation () const;
  void SetAttenuation (double value); 

  void Print (std::ostream &os) const;

private:
  double m_attenuation;

};
}

#endif // ATTENUATION_TAG_H
