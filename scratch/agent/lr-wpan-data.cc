#include "ns3/multi-model-spectrum-channel.h"
#include "rl-agent.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/packet.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"

#include <iostream>

using namespace ns3;
using namespace ns3::lrwpan;

std::string
MacStatusToString(MacStatus status)
{
    switch (status)
    {
    case ns3::lrwpan::MacStatus::SUCCESS:
        return "MAC_SUCCESS";
    case ns3::lrwpan::MacStatus::CHANNEL_ACCESS_FAILURE:
        return "MAC_CHANNEL_ACCESS_FAILURE";
    case ns3::lrwpan::MacStatus::NO_ACK:
        return "MAC_NO_ACK";
    case ns3::lrwpan::MacStatus::NO_DATA:
        return "MAC_NO_DATA";
    case ns3::lrwpan::MacStatus::NO_SHORT_ADDRESS:
        return "MAC_NO_SHORT_ADDRESS";
    case MacStatus::FULL_CAPACITY:
        return "MAC_FULL_CAPACITY";
    case MacStatus::ACCESS_DENIED:
        return "MAC_ACCESS_DENIED";
    case MacStatus::COUNTER_ERROR:
        return "MAC_COUNTER_ERROR";
    case MacStatus::IMPROPER_KEY_TYPE:
        return "MAC_IMPROPER_KEY_TYPE";
    case MacStatus::IMPROPER_SECURITY_LEVEL:
        return "MAC_IMPROPER_SECURITY_LEVEL";
    case MacStatus::UNSUPPORTED_LEGACY:
        return "MAC_UNSUPPORTED_LEGACY";
    case MacStatus::UNSUPPORTED_SECURITY:
        return "MAC_UNSUPPORTED_SECURITY";
    case MacStatus::BEACON_LOSS:
        return "MAC_BEACON_LOSS";
    case MacStatus::DENIED:
        return "MAC_DENIED";
    case MacStatus::DISABLE_TRX_FAILURE:
        return "MAC_DISABLE_TRX_FAILURE";
    case MacStatus::SECURITY_ERROR:
        return "MAC_SECURITY_ERROR";
    case MacStatus::FRAME_TOO_LONG:
        return "MAC_FRAME_TOO_LONG";
    case MacStatus::INVALID_GTS:
        return "MAC_INVALID_GTS";
    case MacStatus::INVALID_HANDLE:
        return "MAC_INVALID_HANDLE";
    case MacStatus::INVALID_PARAMETER:
        return "MAC_INVALID_PARAMETER";
    case MacStatus::NO_BEACON:
        return "MAC_NO_BEACON";
    case MacStatus::OUT_OF_CAP:
        return "MAC_OUT_OF_CAP";
    case MacStatus::PAN_ID_CONFLICT:
        return "MAC_PAN_ID_CONFLICT";
    case MacStatus::REALIGMENT:
        return "MAC_REALIGMENT";
    case MacStatus::TRANSACTION_EXPIRED:
        return "MAC_TRANSACTION_EXPIRED";
    case MacStatus::TRANSACTION_OVERFLOW:
        return "MAC_TRANSACTION_OVERFLOW";
    case MacStatus::TX_ACTIVE:
        return "MAC_TX_ACTIVE";
    case MacStatus::UNAVAILABLE_KEY:
        return "MAC_UNAVAILABLE_KEY";
    case MacStatus::UNSUPPORTED_ATTRIBUTE:
        return "MAC_UNSUPPORTED_ATTRIBUTE";
    case MacStatus::INVALID_ADDRESS:
        return "MAC_INVALID_ADDRESS";
    case MacStatus::ON_TIME_TOO_LONG:
        return "MAC_ON_TIME_TOO_LONG";
    case MacStatus::PAST_TIME:
        return "MAC_PAST_TIME";
    case MacStatus::TRACKING_OFF:
        return "MAC_TRACKING_OFF";
    case MacStatus::INVALID_INDEX:
        return "MAC_INVALID_INDEX";
    case MacStatus::LIMIT_REACHED:
        return "MAC_LIMIT_REACHED";
    case MacStatus::READ_ONLY:
        return "MAC_READ_ONLY";
    case MacStatus::SCAN_IN_PROGRESS:
        return "MAC_SCAN_IN_PROGRESS";
    case MacStatus::SUPERFRAME_OVERLAP:
        return "MAC_SUPERFRAME_OVERLAP";
    default:
        return "UNSUPPORTED_ATTRIBUTE";
    }
}

/**
 * Function called when a the PHY state changes
 * \param context context
 * \param now time at which the function is called
 * \param oldState old PHY state
 * \param newState new PHY state
 */
static void
StateChangeNotification(std::string context,
                        Time now,
                        PhyEnumeration oldState,
                        PhyEnumeration newState)
{
    NS_LOG_UNCOND(context << " state change at " << now.As(Time::S) << " from "
                          << LrWpanTschHelper::LrWpanPhyEnumerationPrinter(oldState) << " to "
                          << LrWpanTschHelper::LrWpanPhyEnumerationPrinter(newState));
}

std::pair<std::vector<Agent*>*, DeviceEnergyModelContainer>
InitializeNetwork(uint16_t node_count,
                  uint16_t slotframe_size,
                  QAgentParams params,
                  Ptr<SpectrumChannel> channel,
                  LrWpanTschHelper* lrWpanHelper)
{
    std::vector<Agent*>* agents = new std::vector<Agent*>();
    NetDeviceContainer devices;
    NodeContainer nodes;
    for (int i = 0; i < node_count; i++)
    {
        Agent* agent = new Agent(i, slotframe_size);
        agent->SetQAgentParams(params);
        agent->SetLrWpanHelper(lrWpanHelper);
        agents->push_back(agent);

        Ptr<Node> n = CreateObject<Node>();
        nodes.Add(n);
        Ptr<LrWpanTschNetDevice> dev = CreateObject<LrWpanTschNetDevice>();
        devices.Add(dev);
        dev->SetChannel(channel);
        n->AddDevice(dev);
        dev->SetTschMode(true);
        dev->SetAddress(Mac16Address(i + 1));

        Ptr<ConstantPositionMobilityModel> sender0Mobility =
            CreateObject<ConstantPositionMobilityModel>();
        sender0Mobility->SetPosition(Vector((i % 2) / 5.0, (i / 2) / 10.0, 0));
        dev->GetPhy()->SetMobility(sender0Mobility);

        if (i > 0)
        {
            dev->GetNMac()->m_macPromiscuousMode = true;

            McpsDataConfirmCallback confirm_cb;
            confirm_cb = MakeCallback(&Agent::DataConfirm, agent);
            dev->GetNMac()->SetMcpsDataConfirmCallback(confirm_cb);
        }
        else
        {
            agent->SetIsSink(true);
        }

        McpsDataIndicationCallback indication_cb;
        indication_cb = MakeCallback(&Agent::DataIndication, agent);
        dev->GetNMac()->SetMcpsDataIndicationCallback(indication_cb);

        MacTimeSlotStartCallback slot_cb;
        slot_cb = MakeCallback(&Agent::TimeSlotStart, agent);
        dev->GetNMac()->SetMacTimeSlotStartCallback(slot_cb);

        agent->SetDevice(dev);
        agent->SetSinkDevice(StaticCast<LrWpanTschNetDevice>(devices.Get(0)));
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

    lrWpanHelper->AddSlotframe(devices, 1, slotframe_size);

    return std::pair(agents, deviceModels);
}

int
main(int argc, char* argv[])
{
    int nodeCount = 2;
    int slotframeSize = 15;
    double packetProbability = 0.03;
    int packetSize = 50;
    int simulationTime = 2;
    double successReward = 1;
    double failureReward = -1;

    CommandLine cmd(__FILE__);

    cmd.AddValue("nodeCount", "Number of nodes in the network", nodeCount);
    cmd.AddValue("slotframeSize", "Size of the slotframe", slotframeSize);
    cmd.AddValue("packetProbability", "Probability of sending a packet in each slotframe", packetProbability);
    cmd.AddValue("packetSize", "Size of the packet", packetSize);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("successReward", "Reward for successful packet transmission", successReward);
    cmd.AddValue("failureReward", "Reward for failed packet transmission", failureReward);

    cmd.Parse(argc, argv);

    // Print all input values
    std::cout << "nodeCount = " << nodeCount << std::endl;
    std::cout << "slotframeSize = " << slotframeSize << std::endl;
    std::cout << "packetProbability = " << packetProbability << std::endl;
    std::cout << "packetSize = " << packetSize << std::endl;
    std::cout << "simulationTime = " << simulationTime << std::endl;
    std::cout << "successReward = " << successReward << std::endl;
    std::cout << "failureReward = " << failureReward << std::endl;

    LrWpanTschHelper lrWpanHelper;

    lrWpanHelper.EnableLogComponents();

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
    lrWpanHelper.EnablePcapAll(std::string("lr-wpan-data"), true);
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("lr-wpan-data.tr");
    lrWpanHelper.EnableAsciiAll(stream);

    QAgentParams agentParams;
    agentParams.alpha = 0.1;
    agentParams.gamma = 0.95;
    agentParams.epsilon = 0.1;
    agentParams.sigma = 0.8;
    agentParams.packetProbability = packetProbability;
    agentParams.packetSize = packetSize;
    agentParams.successReward = successReward;
    agentParams.failureReward = failureReward;

    auto network = InitializeNetwork(nodeCount, slotframeSize, agentParams, channel, &lrWpanHelper);
    std::vector<Agent*>* agents = network.first;
    DeviceEnergyModelContainer energies = network.second;

    StaticCast<LrWpanTschNetDevice>(agents->at(0)->GetDevice())
        ->GetPhy()
        ->TraceConnect("TrxState", std::string("phy0"), MakeCallback(&StateChangeNotification));

    Simulator::Stop(Seconds(simulationTime));

    Simulator::Run();

    Simulator::Destroy();

    uint32_t total_count = 0;
    uint32_t success_count = 0;
    double_t totalDelay = 0;
    for (auto agent : *agents)
    {
        total_count += agent->total_count;
        success_count += agent->success_count;
        totalDelay += agent->totalDelay;

    }
    std::cout << "Total success rate: " << (double)success_count / total_count << " ("
              << success_count << "/" << total_count << ")" << std::endl;
    std::cerr << (double)success_count / total_count << std::endl;

    std::cout << "Total delay: " << totalDelay << std::endl;
    std::cerr << totalDelay / success_count << std::endl;

    double totalEnergyConsumed = 0;
    for (auto iter = energies.Begin(); iter != energies.End(); iter++)
    {
        double energyConsumed = (*iter)->GetTotalEnergyConsumption();
        std::cout << "Total energy consumed by radio = " << energyConsumed << "J" << std::endl;
        totalEnergyConsumed += energyConsumed;
    }
    std::cout << "Total energy consumed by all radios = " << totalEnergyConsumed << "J" << std::endl;
    std::cout << "Average energy consumed by all radios = " << totalEnergyConsumed / nodeCount << "J" << std::endl;
    std::cerr << totalEnergyConsumed / nodeCount << std::endl;
    return 0;
}
