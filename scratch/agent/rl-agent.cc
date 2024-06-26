//
// Created by Nazanin Azrian on 6/7/24.
//

#include "rl-agent.h"

namespace ns3
{

Agent::Agent(uint16_t id, uint16_t size)
{
    random = CreateObject<UniformRandomVariable>();

    nodeId = id;
    slotframeSize = size;

    qTable = new double_t[size];
    actionPeakingTable = new double_t[size];
}

void
Agent::SetIsSink(bool is_sink)
{
    isSink = is_sink;
}

void
Agent::SetDevice(Ptr<LrWpanTschNetDevice> dev)
{
    device = dev;
}

Ptr<LrWpanTschNetDevice>
Agent::GetDevice()
{
    return device;
}

void
Agent::SetSinkDevice(Ptr<LrWpanTschNetDevice> dev)
{
    sinkDevice = dev;
}

void
Agent::SetLrWpanHelper(LrWpanTschHelper* helper)
{
    lrWpanHelper = helper;
}

void
Agent::SetQAgentParams(QAgentParams params)
{
    qAgentParams = params;
}

void
Agent::DataConfirm(McpsDataConfirmParams params)
{
    if (!isSink)
    {
        totalDelay += Simulator::Now().GetSeconds() - sentPacketTime;
        qUpdate(params.m_status == MacStatus::SUCCESS);
        if (params.m_status == MacStatus::SUCCESS)
        {
            success_count++;
//            NS_LOG_UNCOND(Simulator::Now().GetSeconds()
//                          << " Node " << nodeId << " successfully sent a packet at slot "
//                          << (int)currentAction << " asn " << params.m_macASN);
        }
    }
    else
    {
        NS_ABORT_MSG("Sink should not receive data confirm");
    }
}

void
Agent::DataIndication(McpsDataIndicationParams params, Ptr<Packet> p)
{
    if (!isSink)
    {
        uint8_t ts = params.m_macASN % slotframeSize;
        actionPeakingTable[ts] += 1;
    }
    else
    {
//        NS_LOG_UNCOND(Simulator::Now().GetSeconds()
//                      << " Sink received packet from " << params.m_srcAddr);
    }
}

void
Agent::TimeSlotStart(uint64_t mac_asn)
{
    qAgentParams.epsilon = std::min(0.5, 10000.0 / mac_asn);
    if (mac_asn % slotframeSize == 0)
    {
        if (!isSink)
        {
            for (uint8_t i = 0; i < slotframeSize; i++)
            {
                actionPeakingTable[i] *= qAgentParams.sigma;
            }
            // generate a random uniform number
            double rand = random->GetValue();
            if (rand < qAgentParams.epsilon)
            {
                currentAction = std::distance(
                    actionPeakingTable,
                    std::min_element(actionPeakingTable, actionPeakingTable + slotframeSize));
            }
            else
            {
                currentAction =
                    std::distance(qTable, std::max_element(qTable, qTable + slotframeSize));
            }

            AddLinkParams params;
            params.slotframeHandle = 1;
            params.linkHandle = nodeId;
            lrWpanHelper->DeleteLink(device, sinkDevice, params);

            params.timeslot = currentAction;
            lrWpanHelper->AddLink(device, sinkDevice, params, false);

            rand = random->GetValue();
            if (rand < qAgentParams.packetProbability)
            {
                sentPacketTime = Simulator::Now().GetSeconds();
                total_count++;
                // Send a packet
                McpsDataRequestParams sendParams;

                Ptr<Packet> packet = Create<Packet>(qAgentParams.packetSize);

                sendParams.m_dstPanId = 0;
                sendParams.m_srcAddrMode = SHORT_ADDR;
                sendParams.m_dstAddrMode = SHORT_ADDR;
                sendParams.m_dstAddr = Mac16Address(1);

                sendParams.m_msduHandle = random->GetInteger();
                sendParams.m_txOptions = TX_OPTION_ACK;

                sendParams.m_ACK_TX = true;

                Simulator::ScheduleWithContext(random->GetInteger(),
                                               Seconds(0),
                                               &LrWpanMac::McpsDataRequest,
                                               device->GetMac(),
                                               sendParams,
                                               packet);
            }
        }
        else
        {
            // Sink does not need to take any action
        }
    }
}

void
Agent::qUpdate(bool success)
{
    double_t r = success ? qAgentParams.successReward : qAgentParams.failureReward;
    qTable[currentAction] =
        (1 - qAgentParams.alpha) * qTable[currentAction] +
        qAgentParams.alpha *
            (r + qAgentParams.gamma * *std::max_element(qTable, qTable + slotframeSize) -
             qTable[currentAction]);
}

void
Agent::PrintStats()
{
    NS_LOG_UNCOND("Node " << nodeId << " success rate: " << (double)success_count / total_count << " ("
                          << success_count << "/" << total_count << ")");
}

} // namespace ns3
