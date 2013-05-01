#include "imac-random-generator-helper.h"
#include "ns3/application-container.h"
#include "ns3/names.h"
#include "ns3/node-container.h"
#include "ns3/imac-random-traffic-generator.h"

namespace ns3
{
ImacRandomGeneratorHelper::ImacRandomGeneratorHelper()
{
  m_factory.SetTypeId("ns3::ImacRandomTrafficGenerator");
}

ApplicationContainer ImacRandomGeneratorHelper::Install(NodeContainer c) const
{
  ApplicationContainer apps;
  for( NodeContainer::Iterator i = c.Begin(); i != c.End(); ++ i)
  {
    apps.Add (InstallPriv (*i) );
  }
  return apps;
}

void ImacRandomGeneratorHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

Ptr<Application> ImacRandomGeneratorHelper::InstallPriv (Ptr<Node> node) const
{
  Ptr<Application> app = m_factory.Create<ImacRandomTrafficGenerator>();
  node->AddApplication (app);
  return app;
}

ApplicationContainer ImacRandomGeneratorHelper::Install (Ptr<Node> node ) const
{
  return ApplicationContainer (InstallPriv(node));
}

ApplicationContainer ImacRandomGeneratorHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));

}
}
