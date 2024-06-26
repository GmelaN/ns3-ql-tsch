//
// Created by Nazanin Azarian on 6/7/24.
//

#ifndef NS3_RL_AGENT_H
#define NS3_RL_AGENT_H

#include "ns3/core-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"

using namespace ns3::lrwpan;

namespace ns3
{

struct QAgentParams
{
    double_t alpha;
    double_t gamma;
    double_t epsilon;
    double_t sigma;
    double_t packetProbability;
    double_t packetSize;
    double_t successReward;
    double_t failureReward;
};

class Agent
{
  public:
    Agent(uint16_t id, uint16_t size);

    void SetIsSink(bool is_sink);
    void SetDevice(Ptr<LrWpanTschNetDevice> dev);
    Ptr<LrWpanTschNetDevice> GetDevice();
    void SetSinkDevice(Ptr<LrWpanTschNetDevice> dev);
    void SetLrWpanHelper(LrWpanTschHelper* helper);
    void SetQAgentParams(QAgentParams params);

    /**
     * Function called when a Data confirm is invoked
     * \param params MCPS data confirm parameters
     */
    void DataConfirm(McpsDataConfirmParams params);

    /**
     * Function called when a Data indication is invoked
     * \param params MCPS data indication parameters
     * \param p packet
     */
    void DataIndication(McpsDataIndicationParams params, Ptr<Packet> p);

    void TimeSlotStart(uint64_t mac_asn);

    void PrintStats();

    double_t sentPacketTime = 0;

    uint32_t success_count = 0;
    uint32_t total_count = 0;

    double_t totalDelay = 0;


  private:
    uint16_t nodeId;
    bool isSink = false;
    uint16_t slotframeSize;
    Ptr<LrWpanTschNetDevice> device;
    Ptr<LrWpanTschNetDevice> sinkDevice;
    LrWpanTschHelper* lrWpanHelper;
    Ptr<RandomVariableStream> random;

    QAgentParams qAgentParams;
    double_t* actionPeakingTable;
    double_t* qTable;
    uint8_t currentAction;

    void qUpdate(bool success);
};

} // namespace ns3

#endif // NS3_RL_AGENT_H
