//
// Created by Nazanin Azarian on 6/7/24.
//

#ifndef MULTI_PAN_RL_AGENT_H
#define MULTI_PAN_RL_AGENT_H

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include <ns3/lr-wpan-tsch-net-device.h>
#include <ns3/lr-wpan-tsch-mac.h>
#include <ns3/callback.h>
#include <ns3/log.h>

NS_LOG_COMPONENT_DEFINE("Agent");

using namespace ns3::lrwpan;

namespace ns3
{

struct QAgentParams
{
    double_t alpha;
    double_t gamma;
    double_t epsilon;
    double_t sigma;
    double_t successReward;
    double_t failureReward;
};


class Agent: public Object
{
  public:
    TypeId GetTypeId();
    Agent();
    Agent(NetDeviceContainer devs);
    ~Agent();
    /**
     * Function called when a Data confirm is invoked
     * \param params MCPS data confirm parameters
     */
    void DataConfirm(McpsDataConfirmParams params);

    void SetLrWpanHelper(LrWpanTschHelper* helper)
    {
        this->m_helper = helper;
    }
    void SetQAgentParams(QAgentParams params)
    {
        this->m_params = params;
    }
    void SetDevices(NetDeviceContainer devs)
    {
        this->m_devs = devs;
    };
    const NetDeviceContainer GetDevices()
    {
        return this->m_devs;
    };

    /**
    * deploy new policy.
    * generate new configuration, deploy configuration to all nodes.
    */
    void DeployNewPolicy();
    /**
    * one slotframe passed, update Q-table.
    */
    void OnePeriodHoppingSequencePassed(uint64_t macAsn);
    /**
    * choose action from epsilion-greedy strategy.
    * result: channel
    */
    uint8_t ChooseAction(uint32_t slot);
    void CountSucceed(std::pair<uint8_t, uint32_t> info);

    uint32_t success_count = 0;
    uint32_t total_count = 0;
    double_t totalDelay = 0;


private:
    LrWpanTschHelper* m_helper;
    Ptr<RandomVariableStream> m_random;
    NetDeviceContainer m_devs;

    QAgentParams m_params;

    std::vector<std::vector<double>> m_qTable; // [timeslot][channel]
    std::vector<std::vector<bool>> m_isSucceed; // [timeslot][channel]
    std::vector<uint8_t> m_currentConfiguration; // currentConfiguration[timeSlot] = channel
    uint32_t m_timeslotCount;
    uint8_t m_channelCount;


    uint8_t m_currentAction;
    uint8_t m_panId;

    uint8_t* m_linkHandles;

    // beacon scale stats
    uint32_t m_macRxDrop = 0;

    // total statistics
    uint32_t m_txEnqueued = 0;
    uint32_t m_txDequeued = 0;
    uint32_t m_txSuccess = 0;
    uint32_t m_txFailed = 0;
    uint32_t m_phyRxDrop = 0;

    void MacTxEnqueueCallback(Ptr<const Packet> /*pkt*/)
    {
        ++m_txEnqueued;
    }
    void MacTxDequeueCallback(Ptr<const Packet> /*pkt*/)
    {
        ++m_txDequeued;
    }
    void MacTxOkCallback(Ptr<const Packet> /*pkt*/)
    {
        ++m_txSuccess;
    }
    void MacTxDropCallback(Ptr<const Packet> /*pkt*/)
    {
        ++m_txFailed;
    }
    void PhyRxDropCallback(Ptr<const Packet> /*pkt*/)
    {
        ++m_phyRxDrop;
    }
};

} // namespace ns3

#endif // MULTI_PAN_RL_AGENT_H
