#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/core-module.h>
#include <ns3/energy-module.h>
#include <ns3/log.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/packet.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>

#include <ns3/mobility-helper.h>

#include <iostream>
#include <vector>

using namespace ns3;
using namespace ns3::lrwpan;

#define NODE_COUNT 2 // minimum 2

// Each device must be attached to the same channel
Ptr<MultiModelSpectrumChannel> m_channel;

class PanNetwork
{
    public:
    // helper classes
    LrWpanTschHelper lrWpanHelper;
    MobilityHelper mobHelper;



    // devices
    NetDeviceContainer devList;
    NodeContainer nodeList;

    PanNetwork()
    {
        Ptr<LogDistancePropagationLossModel> propModel =
            CreateObject<LogDistancePropagationLossModel>();
        Ptr<ConstantSpeedPropagationDelayModel> delayModel =
            CreateObject<ConstantSpeedPropagationDelayModel>();

        m_channel = CreateObject<MultiModelSpectrumChannel>();
        m_channel->SetPropagationDelayModel(delayModel);
        m_channel->AddPropagationLossModel(propModel);


        lrWpanHelper.SetChannel(m_channel);
        lrWpanHelper.EnableLogComponents();

        NodeContainer n;
        n.Create(NODE_COUNT);

        devList = lrWpanHelper.Install(n);
        nodeList = n;

        // manual configuration
        lrWpanHelper.AssociateToPan(devList, 5);
        lrWpanHelper.AddSlotframe(devList, 0, NODE_COUNT + 1); // bcast

        // lrwpanhelper.m_slotframehandle will broken
        AddLinkParams alparams;
        alparams.slotframeHandle = 0; // m_slotframehandle
        alparams.channelOffset = 0;
        alparams.linkHandle = 0;
        alparams.timeslot = 0;

        uint16_t c = 1;
        for (int j = 0; j < NODE_COUNT; j++, c++) {
            alparams.linkHandle = c;
            alparams.timeslot = c;
            lrWpanHelper.AddLink(devList, 1, 0, alparams, false);
        }

        // lrWpanHelper.ConfigureSlotframeAllToPan(devList, 0, false, false);

        mobHelper = MobilityHelper();
        double radius = 1.0, zErrorRange = 0.5;
        Vector3D center = Vector3D(0, 0, 0);

        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
        Ptr<UniformRandomVariable> angleGen = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> zErrorGen = CreateObject<UniformRandomVariable>();

        angleGen->SetAttribute("Min", DoubleValue(0.0));
        angleGen->SetAttribute("Max", DoubleValue(360.0));

        zErrorGen->SetAttribute("Min", DoubleValue(-zErrorRange));
        zErrorGen->SetAttribute("Max", DoubleValue(zErrorRange));

        center = Vector3D(0, 0, 0);
        for (int j = 0; j < NODE_COUNT; ++j) {
            double angle = angleGen->GetValue(); // 각도 랜덤 생성 (0~360도)
            double x = center.x + radius * std::cos(angle * M_PI / 180.0); // x 좌표
            double y = center.y + radius * std::sin(angle * M_PI / 180.0); // y 좌표
            double z = center.z + zErrorGen->GetValue(); // z 좌표는 ±zErrorRange 범위에서 랜덤 오차

            positionAlloc->Add(Vector(x, y, z)); // 좌표 추가
        }

        mobHelper.SetPositionAllocator(positionAlloc);
        mobHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobHelper.Install(nodeList);
    }

    static void
    mcpsDataIndication(McpsDataIndicationParams p, Ptr<Packet> pkt)
    {
        NS_LOG_UNCOND(
            "SRC PAN ID: " << p.m_srcPanId << "(" << p.m_srcAddr
            << ") DST PAN ID: " << p.m_dstPanId << "(" << p.m_dstAddr << ")"
        );
    }

    void SendPacket()
    {
        lrWpanHelper.SendPacket(devList.Get(1), devList.Get(0)->GetAddress(), 30, 0.1, 2000);
    }
};


int
main(int argc, char* argv[])
{
    // int panCount = 1;
    // int nodeCount = 2;
    // int slotframeSize = 2;
    // int packetSize = 50;
    int simulationTime = 5;


    LogComponentEnable("LrWpanTschMac", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanPhy", LOG_LEVEL_ALL);
    LogComponentEnable("MultiModelSpectrumChannel", LOG_LEVEL_ALL);
    LogComponentEnable("LrWpanTschNetDevice", LOG_LEVEL_ALL);


    PanNetwork* panNetwork = new PanNetwork();
    Simulator::Schedule(
        Seconds(0.1),
        &PanNetwork::SendPacket,
        panNetwork
    );

    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
