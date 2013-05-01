#ifndef NOISEPLUSINTERFERENCE_TAG_H 
#define NOISEPLUSINTERFERENCE_TAG_H 

#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include "ns3/tag-buffer.h"


namespace ns3
{
class NoisePlusInterferenceTag: public Tag
{
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  double Get () const;
  void Set (double value); 

  void Print (std::ostream &os) const;

private:
  double m_noisePlusInterference;

};
}

#endif // NOISEPLUSINTERFERENCE_TAG_H
