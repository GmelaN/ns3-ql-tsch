#include "rl-agent.h"


namespace ns3
{
NS_LOG_COMPONENT_DEFINE("Agent");
//NS_OBJECT_ENSURE_REGISTERED(Agent);

TypeId
Agent::GetTypeId()
{
    static TypeId tid = TypeId("ns3::Agent")
                            .SetParent<Object>()
                            .AddConstructor<Agent>();
    return tid;
}

Agent::Agent()
{
    NS_ASSERT_MSG(false, "DO NOT USE THIS");
}
Agent::Agent(NetDeviceContainer devs)
{
    m_random = CreateObject<UniformRandomVariable>();
    m_devs = devs;

    for(auto i = m_devs.Begin(); i < m_devs.End(); i++)
    {
        DynamicCast<LrWpanTschNetDevice>(*i)->GetNMac()->TraceConnectWithoutContext(
            "MacTxDataRxAck",
            MakeCallback(&Agent::CountSucceed, this);
        );
    }


    m_qTable.resize(m_timeslotCount, std::vector<double>(m_channelCount, 0.0));
    m_isSucceed.resize(m_timeslotCount, std::vector<bool>(m_channelCount, false));
    m_linkHandles = (uint8_t*) malloc(sizeof(uint8_t) * m_devs.GetN());
    memset(m_linkHandles, 0, sizeof(uint8_t) * m_devs.GetN());
    for(int i = 0; i < m_devs.GetN(); i++)
    {
        m_linkHandles[i] = i;
    }
}

Agent::~Agent()
{
    free(m_linkHandles);
}


uint8_t
Agent::ChooseAction(uint32_t slot)
{
    // exploration
    if(m_random->GetValue() < m_params.epsilon)
    {
        uint8_t randChannel = m_random->GetInteger() % m_channelCount + 11;

        return randChannel;
    }

    // exploitation
    double maxQ = -1e10;
    uint32_t bestChannel = 0;

    for (uint32_t c = 0; c < m_channelCount; ++c)
    {
        if (m_qTable[slot][c] > maxQ)
        {
            maxQ = m_qTable[slot][c];
            bestChannel = c;
        }
    }

    return bestChannel;
}


void
Agent::OnePeriodHoppingSequencePassed(uint64_t macAsn)
{
    for (uint32_t t = 0; t < m_timeslotCount; ++t)
    {
        for (uint32_t c = 0; c < m_channelCount; ++c)
        {
            int r = m_isSucceed[t][c] ? 1 : -1;
            m_qTable[t][c] = m_qTable[t][c] + m_params.alpha * (r - m_qTable[t][c]);
        }
    }
    // initialize isSucceed
    m_isSucceed.resize(m_timeslotCount, std::vector<bool>(m_channelCount, false));
}


void
Agent::DeployNewPolicy()
{
    std::vector<uint8_t> slots;
    // get actions
    for(int i = 0; i < m_timeslotCount; i++)
    {
         slots.push_back(ChooseAction(i));
    }

    for(auto i = m_devs.Begin(); i < m_devs.End(); i++)
    {
        DynamicCast<LrWpanTschNetDevice>(*i)->GetNMac()->SetHoppingSequence(slots, 0);
    }

//    int a = 0;
//    for(int i = 0; i < m_timeslotCount; i++)
//    {
//        // considers only slotframeHandle and linkHandle.
//        AddLinkParams params;
//        params.linkHandle = m_linkHandles[i];
//        params.slotframeHandle = 0;
//        params.channelOffset = 0;
//        params.timeslot = 0;
//
//        // dstPos is always 0.
//        if(a == m_devs.GetN())
//        {
//            // end of device reached.
//            return;
//        }
//
//        m_helper->DeleteLink(m_devs, a, 0, params, false);
//
//        params.linkHandle = m_linkHandles[i];
//        params.slotframeHandle = 0;
//        params.channelOffset = 0;
//        params.timeslot = 0;
//        m_helper->AddLink(m_devs, a, 0, params, false);
//        a++;
//    }
}

void
Agent::CountSucceed(std::pair<uint8_t, uint32_t> info)
{
    uint8_t ch = info.first;
    uint32_t slot = info.second;

    NS_LOG_DEBUG("transmission succeed at: [channel " << ch << "\t, slot " << slot << "]");

    m_isSucceed[slot][ch] = true;
}


// TODO: 전송 성공/실패 카운트 로직 구현 필요
void
Agent::DataConfirm(McpsDataConfirmParams params)
{
    // TODO: we need slot information
    if(params.m_status != MacStatus::SUCCESS)
    {
        if(
            params.m_status != MacStatus::NO_ACK &&
            params.m_status != MacStatus::CHANNEL_ACCESS_FAILURE
        )
        {
            std::cout << "DATA FAILURE REASON: " << params.m_status << std::endl;
        }

        m_macRxDrop++;
    }
}

} // namespace ns3
