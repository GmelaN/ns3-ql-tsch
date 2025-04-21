#include "rl-agent.h"


namespace ns3
{
NS_LOG_COMPONENT_DEFINE("RlAgent");
NS_OBJECT_ENSURE_REGISTERED(Agent);

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
    m_timeslotCount = devs.GetN();
    m_channelCount = 16;


    for(auto i = m_devs.Begin(); i < m_devs.End(); i++)
    {
        DynamicCast<LrWpanTschNetDevice>(*i)->GetNMac()->TraceConnectWithoutContext(
            "MacTxDataRxAck",
            MakeCallback(&Agent::CountSucceed, this)
        );
    }


    for (size_t i = 0; i < m_devs.GetN(); i++)
    {
        m_currentConfiguration.push_back(m_random->GetValue() * m_channelCount + 11);
    }


    m_qTable.resize(m_timeslotCount, std::vector<double>(m_channelCount, 0.0));
    m_isSucceed.resize(m_timeslotCount, std::vector<bool>(m_channelCount, false));

}

Agent::~Agent()
{
}


uint8_t
Agent::ChooseAction(uint32_t slot)
{
    // exploration
    if(active && m_random->GetValue() < m_params.epsilon)
    {
        uint8_t randChannel = m_random->GetInteger() % m_channelCount + 11;
        NS_LOG_DEBUG("(exploration) channel " << 11 + (int) randChannel);
        return randChannel;
    }

    NS_LOG_DEBUG("PAN " << panId << " not active, only exploitation.");
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
    NS_LOG_DEBUG("(exploitation) channel " << 11 + (int) bestChannel);
    return 11 + bestChannel;
}


void
Agent::OnePeriodHoppingSequencePassed(uint64_t macAsn)
{
    NS_LOG_FUNCTION(this);
    if (active)
    {
        for (uint32_t t = 0; t < m_timeslotCount; ++t)
        {
            uint8_t c = m_currentConfiguration[t];
            int r = m_isSucceed[t][c-11] ? 1 : -1;

            // get best Q value from next slot
            if (t >= m_timeslotCount - 1)
            {
                m_qTable[t][c] = (1-m_params.alpha) * m_qTable[t][c] + m_params.alpha * r;
            }
            else
            {
                double maxQNext = -1e10;
                for (uint32_t c2 = 0; c2 < m_channelCount; ++c2)
                {
                    maxQNext = std::max(maxQNext, m_qTable[(t + 1) % m_timeslotCount][c2]);
                }
                m_qTable[t][c] = (1 - m_params.alpha) * m_qTable[t][c]
                    + m_params.alpha * (r + m_params.gamma * maxQNext);
            }
            NS_LOG_DEBUG("giving reward " << r << " to timeslot: " << t << " channel: " << static_cast<int>(c));
        }
    }
    else
    {
        NS_LOG_DEBUG("PAN " << panId << " not active, deactiveCount: " << deactiveCount << " of " << m_deactiveCount);
        deactiveCount++;
    }

    // initialize isSucceed
    m_isSucceed.assign(m_timeslotCount, std::vector<bool>(m_channelCount, false));

    for (uint32_t t = 0; t < m_timeslotCount; ++t)
    {
        for (uint32_t c = 0; c < m_channelCount; ++c)
        {
            NS_ASSERT(m_isSucceed[t][c] == false);
            // int r = m_isSucceed[t][c] ? 1 : -1;
            // m_qTable[t][c] = m_qTable[t][c] + m_params.alpha * (r - m_qTable[t][c]);
        }
    }
    DeployNewPolicy();

    if (deactiveCount == m_deactiveCount)
    {
        NS_LOG_DEBUG("PAN " << panId << "now active.");
        active = true;
        deactiveCount = 0;
    }
}


void
Agent::DeployNewPolicy()
{
    NS_LOG_FUNCTION(this);
    std::vector<uint8_t> slots; // slots[slot] = channel

    // get actions
    NS_LOG_DEBUG("time slot configuration: ");
    for(size_t i = 0; i < m_timeslotCount; i++)
    {
        uint8_t action = ChooseAction(i);
        slots.push_back(action);
        m_currentConfiguration.push_back(action);
        NS_LOG_DEBUG("slot " << i << ": " << static_cast<int>(action));
    }

    for(auto i = m_devs.Begin(); i < m_devs.End(); i++)
    {
        DynamicCast<LrWpanTschNetDevice>(*i)->GetNMac()->SetHoppingSequence(slots, 0);
    }
}

void
Agent::CountSucceed(std::pair<uint8_t, uint32_t> info)
{
    if (!active)
    {
        NS_LOG_DEBUG("PAN " << panId << " not actived.");
        return;
    }
    NS_LOG_FUNCTION(this);
    uint8_t ch = info.first;
    uint32_t slot = info.second;

    NS_LOG_DEBUG("transmission succeed at: [channel " << static_cast<int>(ch) << "\t, slot " << slot << "]");

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
