#include <time.h>
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/settings.h"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string>
//#include <gsl/gsl_cdf.h>  testing the GSL libaraly


using namespace std;
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("iMac");

const uint32_t MAX_RANDOM_SEED = 100000;
const uint32_t MAX_RUN_NUMBER = 100;
const uint32_t APP_START_TIME = 0;
const uint32_t APP_END_TIME = 120;
const double TRAFFIC_GENERATION_PROBABILITY = 1;

int main(int argc, char *argv[])
{

  srand (time (NULL));
  int seedNumber = rand () % MAX_RANDOM_SEED; // 
  SeedManager::SetSeed (seedNumber);
  srand (time (NULL));
  int runNumber = rand () % MAX_RUN_NUMBER;
  SeedManager::SetRun (runNumber);

  LogComponentEnable ("ImacRandomTrafficGenerator", LOG_LEVEL_DEBUG);
#if defined (SMALL_NETWORK)
  const char* TopologyFilePath = "scratch/data5x5x5.txt";
#elif defined (LARGE_NETWORK)
  const char* TopologyFilePath = "scratch/data5x7x7.txt";
#elif defined (CONVERGECAST)
  const char* TopologyFilePath = "scratch/convergecast.txt";
#endif


  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  uint32_t NodesCount = 0; 

  //read the coordinate file. The coordinates satisfy the poisson spatial distribution
  ifstream dataFile (TopologyFilePath);
  string line;
  istringstream lineBuffer;
  string temp;
  double x;
  double y;
  double z;
  if (dataFile.is_open())
  {
    while (getline(dataFile, line))
    {
    	lineBuffer.str(line);
    	lineBuffer >> x;
    	lineBuffer >> y;
    	//lineBuffer >> z;
      z = 0; // we are using a 2-D topology
      positionAlloc->Add(Vector (x,y,z));
      NodesCount ++; 
    }
  }
  dataFile.close();

  mobility.SetPositionAllocator (positionAlloc);// assign the coordinates to each node.
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");// for the first step of our simulation

  NodeContainer nodes;
  nodes.Create(NodesCount);

  mobility.Install(nodes);

  WifiImacChannelHelper channel = WifiImacChannelHelper::Default();

  channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel","Exponent", DoubleValue(PATH_LOSS_EXPONENT)); 
  channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  WifiImacPhyHelper phy = WifiImacPhyHelper::Default();
  phy.SetChannel(channel.Create () );

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();
  mac.SetType ("ns3::AdhocWifiMac");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
      "DataMode", StringValue ("OfdmRate250Kbps"));

  NetDeviceContainer netDevices = wifi.Install (phy, mac, nodes);

  ImacRandomGeneratorHelper randGenerator;
  randGenerator.SetAttribute("Probability",DoubleValue(TRAFFIC_GENERATION_PROBABILITY));
  
  ApplicationContainer apps = randGenerator.Install (nodes);

  apps.Start (Seconds(APP_START_TIME)); /*Suppose this time is enough*/
  apps.Stop (Seconds(APP_END_TIME));

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
