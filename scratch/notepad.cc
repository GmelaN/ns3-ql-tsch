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

#include "iam/iam-packet-header.h"

#include <iostream>


using namespace ns3;
using namespace ns3::lrwpan;


NetDeviceContainer devices;
NodeContainer nodes;

static void
macTimeSlotStart(unsigned long i)
{
    NS_LOG_UNCOND("slot " << i << " start.");
}

static void
mcpsDataIndication(McpsDataIndicationParams p, Ptr<Packet> pkt)
{
    NS_LOG_UNCOND(
        "SRC PAN ID: "
        << p.m_srcPanId
        << "("
        << p.m_srcAddr
        << ") DST PAN ID: "
        << p.m_dstPanId
        << "("
        << p.m_dstAddr
        << ")"
    );

    IamHeader header;
    pkt->RemoveHeader(header);
    uint16_t result = header.GetData();

    NS_LOG_UNCOND(
        "IAM-HEADER VALUE: "
        << result
    );
}

static void
printChannelHoppingList()
{
    NS_LOG_UNCOND("=======================");
    for(auto i = devices.Begin(); i < devices.End(); i++) {
        Ptr<LrWpanTschNetDevice> d = DynamicCast<LrWpanTschNetDevice>(*i);
        d->GetNMac()->PrintChannelHoppingList(std::cout);
    }
    NS_LOG_UNCOND("=======================");

    Simulator::Schedule(
        Seconds(0.5),
        printChannelHoppingList
    );
}

uint16_t panIdIter = 0;



void
SendIamPacket(Ptr<NetDevice> dev,
                             Address dst,
                             int packet_size,
                             double interval,
                             double end)
{
    static uint16_t data = 0;
    if (Now().GetSeconds() <= end)
    {
        IamHeader header;
        header.SetData(data++);
        Ptr<Packet> pkt = Create<Packet>(packet_size);
        pkt->AddHeader(header);
        NS_LOG_UNCOND("Issuing IAM packet: " << header.GetData());
        dev->Send(pkt, dst, 0x86DD);
    }

    if (Now().GetSeconds() <= end + interval)
    {
        Simulator::Schedule(Seconds(interval),
                            &SendIamPacket,
                            dev,
                            dst,
                            packet_size,
                            interval,
                            end);
    }
}


void
InitializeNetwork(uint16_t node_count,
                  uint16_t slotframe_size,
                  Ptr<SpectrumChannel> channel,
                  LrWpanTschHelper* lrWpanHelper)
{
    nodes.Create(node_count);

    MobilityHelper mobHelper = MobilityHelper();

    double radius = 1.0, zErrorRange = 0.5;
    Vector3D center = Vector3D(0, 0, 0);

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<UniformRandomVariable> angleGen = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> zErrorGen = CreateObject<UniformRandomVariable>();

    angleGen->SetAttribute("Min", DoubleValue(0.0));
    angleGen->SetAttribute("Max", DoubleValue(360.0));

    zErrorGen->SetAttribute("Min", DoubleValue(-zErrorRange));
    zErrorGen->SetAttribute("Max", DoubleValue(zErrorRange));

    for (int i = 0; i < node_count; ++i) {
        double angle = angleGen->GetValue(); // 각도 랜덤 생성 (0~360도)
        double x = center.x + radius * std::cos(angle * M_PI / 180.0); // x 좌표
        double y = center.y + radius * std::sin(angle * M_PI / 180.0); // y 좌표
        double z = center.z + zErrorGen->GetValue(); // z 좌표는 ±zErrorRange 범위에서 랜덤 오차

        positionAlloc->Add(Vector(x, y, z)); // 좌표 추가
    }

    mobHelper.SetPositionAllocator(positionAlloc);
    mobHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobHelper.Install(nodes);

    devices = lrWpanHelper->Install(nodes);

    for (int i = 0; i < node_count - 1; i++)
    {
        Ptr<LrWpanTschNetDevice> dev = DynamicCast<LrWpanTschNetDevice>(devices.Get(i));

        if (i > 0)
        {
            dev->GetNMac()->m_macPromiscuousMode = true;

            // McpsDataConfirmCallback confirm_cb;
            // dev->GetNMac()->SetMcpsDataConfirmCallback(confirm_cb);
        }
        dev->GetNMac()->SetMcpsDataIndicationCallback(MakeBoundCallback(&mcpsDataIndication));
        dev->GetNMac()->SetMacTimeSlotStartCallback(MakeBoundCallback(&macTimeSlotStart));
    }

    LrWpanEnergySourceHelper sourceHelper;
    // configure energy source
    sourceHelper.Set("LrWpanEnergySourceInitialEnergyJ", DoubleValue(0.1));
    // install source
    EnergySourceContainer sources = sourceHelper.Install(nodes);
    /* device energy model */
    LrWpanRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
    // install device model
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(devices, sources);

    const uint16_t PAN_ID = 1;
    lrWpanHelper->AssociateToPan(devices, PAN_ID);

    // lrWpanHelper->AddSlotframe(devices, 1, slotframe_size);
    lrWpanHelper->ConfigureSlotframeAllToPan(devices, 0, false, false);

    SendIamPacket(devices.Get(1), Mac16Address("00:01"), 64, 0.1, 2000);


    Ptr<LrWpanTschNetDevice> other = DynamicCast<LrWpanTschNetDevice>(devices.Get(2));
    other->GetNMac()->SendIamInit();
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("LrWpanTschMac", LOG_DEBUG);
    int panCount = 1;
    int nodeCount = 3;
    int slotframeSize = 2;
    int packetSize = 50;
    int simulationTime = 2000;

    // CommandLine cmd(__FILE__);

    // cmd.AddValue("nodeCount", "Number of nodes in the network", nodeCount);
    // cmd.AddValue("slotframeSize", "Size of the slotframe", slotframeSize);
    // cmd.AddValue("packetSize", "Size of the packet", packetSize);
    // cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);

    // cmd.Parse(argc, argv);

    // Print all input values
    std::cout << "nodeCount = " << nodeCount << std::endl;
    std::cout << "slotframeSize = " << slotframeSize << std::endl;
    std::cout << "packetSize = " << packetSize << std::endl;
    std::cout << "simulationTime = " << simulationTime << std::endl;

    // LrWpanTschHelper lrWpanHelper;
    // lrWpanHelper.EnableLogComponents();

    // Enable calculation of FCS in the trailers. Only necessary when interacting with real devices
    // or wireshark. GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

    // Each device must be attached to the same channel
    Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
    Ptr<LogDistancePropagationLossModel> propModel =
        CreateObject<LogDistancePropagationLossModel>();
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    channel->AddPropagationLossModel(propModel);
    channel->SetPropagationDelayModel(delayModel);

    // Tracing
    // lrWpanHelper.EnablePcapAll(std::string("lr-wpan-data"), true);
    // AsciiTraceHelper ascii;
    // Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("lr-wpan-data.tr");
    // lrWpanHelper.EnableAsciiAll(stream);
    LrWpanTschHelper lrWpanHelper = LrWpanTschHelper(channel);

    lrWpanHelper.EnableLogComponents();

    for(int i = 0; i < panCount; i++) {
        InitializeNetwork(nodeCount, slotframeSize, channel, &lrWpanHelper);
    }

    Simulator::Schedule(
        Seconds(0.01),
        printChannelHoppingList
    );

    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
