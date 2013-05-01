#ifndef IMAC_RANDOM_GENERATOR_HELPER_H
#define IMAC_RANDOM_GENERATOR_HELPER_H

#include <string>
#include "ns3/attribute.h"
#include "ns3/node-container.h"
#include "ns3/application-container.h"
#include "ns3/object-factory.h"

namespace ns3
{
class ImacRandomGeneratorHelper
{
public:
  ImacRandomGeneratorHelper();
  void SetAttribute (std::string name, const AttributeValue &value);
  ApplicationContainer Install (NodeContainer c) const;

  ApplicationContainer Install (Ptr<Node> node ) const;

  ApplicationContainer Install (std::string nodeName) const;
private:
  ObjectFactory m_factory;
  Ptr<Application> InstallPriv (Ptr<Node> node) const;
};
}

#endif
