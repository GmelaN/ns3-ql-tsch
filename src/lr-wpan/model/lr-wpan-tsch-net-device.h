/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:
 *  Tom Henderson <thomas.r.henderson@boeing.com>
 *  Tommaso Pecorella <tommaso.pecorella@unifi.it>
 *  Margherita Filippetti <morag87@gmail.com>
 *  Peter Kourzanov <peter.kourzanov@gmail.com>
 */
#ifndef LR_WPAN_TSCH_NET_DEVICE_H
#define LR_WPAN_TSCH_NET_DEVICE_H

#include <ns3/lr-wpan-tsch-mac.h>
#include <ns3/net-device.h>
#include <ns3/traced-callback.h>

namespace ns3
{
class SpectrumChannel;
class Node;

namespace lrwpan
{

class LrWpanPhy;
class LrWpanCsmaCa;

/**
 * \ingroup lr-wpan
 *
 * \brief Network layer to device interface.
 *
 * The ns3::NetDevice includes IP-specific API such as GetMulticast(), Send()
 * and SendTo() methods, which do not map well the the 802.15.4 MAC MCPS
 * DataRequest primitive.  So, the basic design is to provide, as
 * much as makes sense, the class ns3::NetDevice API, but rely on the user
 * accessing the LrWpanMac pointer to make 802.15.4-specific API calls.
 * As such, this is really just an encapsulating class.
 */
class LrWpanTschNetDevice : public NetDevice
{
  public:
    /**
     * Get the type ID.
     *
     * \return the object TypeId
     */
    static TypeId GetTypeId(void);

    LrWpanTschNetDevice(void);
    virtual ~LrWpanTschNetDevice(void);

    /**
     * Set the MAC to be used by this NetDevice.
     *
     * \param mac the MAC to be used
     */
    void SetMac(Ptr<LrWpanTschMac> mac);

    /**
     * Set the PHY to be used by the MAC and this NetDevice.
     *
     * \param phy the PHY to be used
     */
    void SetPhy(Ptr<LrWpanPhy> phy);

    /**
     * Set the CSMA/CA implementation to be used by the MAC and this NetDevice.
     *
     * \param csmaca the CSMA/CA implementation to be used
     */
    void SetCsmaCa(Ptr<LrWpanCsmaCa> csmaca);

    /**
     * Set the channel to which the NetDevice, and therefore the PHY, should be
     * attached to.
     *
     * \param channel the channel to be used
     */
    void SetChannel(Ptr<SpectrumChannel> channel);
    void SetChannel(uint8_t channel);

    /**
     * Get the MAC used by this NetDevice.
     *
     * \return the MAC object
     */
    Ptr<LrWpanMac> GetMac(void) const;
    Ptr<LrWpanTschMac> GetNMac(void) const;
    Ptr<LrWpanMac> GetOMac(void) const;

    /**
     * Get the PHY used by this NetDevice.
     *
     * \return the PHY object
     */
    Ptr<LrWpanPhy> GetPhy(void) const;

    /**
     * Get the CSMA/CA implementation used by this NetDevice.
     *
     * \return the CSMA/CA implementation object
     */
    Ptr<LrWpanCsmaCa> GetCsmaCa(void) const;

    // From class NetDevice
    void SetIfIndex(const uint32_t index) override;
    uint32_t GetIfIndex(void) const override;
    Ptr<Channel> GetChannel(void) const override;
    /**
     * This method indirects to LrWpanMac::SetShortAddress ()
     */
    void SetAddress(Address address) override;
    /**
     * This method indirects to LrWpanMac::SetShortAddress ()
     */
    Address GetAddress(void) const override;
    bool SetMtu(const uint16_t mtu) override;
    uint16_t GetMtu(void) const override;
    bool IsLinkUp(void) const override;
    void AddLinkChangeCallback(Callback<void> callback) override;
    bool IsBroadcast(void) const override;
    Address GetBroadcast(void) const override;
    bool IsMulticast(void) const override;
    Address GetMulticast(Ipv4Address multicastGroup) const override;
    Address GetMulticast(Ipv6Address addr) const override;
    bool IsBridge(void) const override;
    bool IsPointToPoint(void) const override;
    bool Send(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber) override;
    virtual bool Send(Ptr<Packet> packet,
                      const Address& dest,
                      bool use_ack,
                      uint16_t protocolNumber);
    bool SendFrom(Ptr<Packet> packet,
                          const Address& source,
                          const Address& dest,
                          uint16_t protocolNumber) override;
    Ptr<Node> GetNode(void) const override;
    void SetNode(Ptr<Node> node) override;
    bool NeedsArp(void) const override;

    void SetReceiveCallback(ReceiveCallback cb) override;
    void SetPromiscReceiveCallback(PromiscReceiveCallback cb) override;
    bool SupportsSendFrom(void) const override;

    /**
     * The callback used by the MAC to hand over incoming packets to the
     * NetDevice. This callback will in turn use the ReceiveCallback set by
     * SetReceiveCallback() to notify upper layers.
     *
     * \param params 802.15.4 specific parameters, including source and destination addresses
     * \param pkt the packet do be delivered
     */
    void McpsDataIndication(McpsDataIndicationParams params, Ptr<Packet> pkt);

    /**
     * Assign a fixed random variable stream number to the random variables
     * used by this model.  Return the number of streams that have been assigned.
     *
     * \param stream first stream index to use
     * \return the number of stream indices assigned by this model
     */
    int64_t AssignStreams(int64_t stream);

    /********************TSCH*************************/
    /**
     * Show the setted slotframe status
     * \param TSCH MAC slotframe status
     */
    void SlotframeConfirm(MlmeSetSlotframeConfirmParams params);

    /**
     * Show the setted link status
     * \param TSCH MAC link status
     */
    void LinkConfirm(MlmeSetLinkConfirmParams params);

    /**
     * Show the setted TSCH mode status
     * \param TSCH mode status
     */
    void ModeConfirm(MlmeTschModeConfirmParams params);

    void SetTschMode(bool enable);

  private:
    // Inherited from NetDevice/Object
    void DoDispose(void) override;
    void DoInitialize(void) override;

    /**
     * Mark NetDevice link as up.
     */
    void LinkUp(void);

    /**
     * Mark NetDevice link as down.
     */
    void LinkDown(void);

    /**
     * Attribute accessor method for the "Channel" attribute.
     *
     * \return the channel to which this NetDevice is attached
     */
    Ptr<SpectrumChannel> DoGetChannel(void) const;

    /**
     * Configure PHY, MAC and CSMA/CA.
     */
    void CompleteConfig(void);

    int m_isTsch;
    /**
     * The TSCH MAC for this NetDevice.
     */
    Ptr<LrWpanTschMac> m_mac;

    /**
     * The standard MAC for this NetDevice.
     */
    Ptr<LrWpanMac> m_omac;

    /**
     * The PHY for this NetDevice.
     */
    Ptr<LrWpanPhy> m_phy;

    /**
     * The CSMA/CA implementation for this NetDevice.
     */
    Ptr<LrWpanCsmaCa> m_csmaca;

    /**
     * The node associated with this NetDevice.
     */
    Ptr<Node> m_node;

    /**
     * True if MAC, PHY and CSMA/CA where successfully configured and the
     * NetDevice is ready for being used.
     */
    bool m_configComplete;

    /**
     * Configure the NetDevice to request MAC layer acknowledgements when sending
     * packets using the Send() API.
     */
    bool m_useAcks;

    /**
     * Is the link/device currently up and running?
     */
    bool m_linkUp;

    /**
     * The interface index of this NetDevice.
     */
    uint32_t m_ifIndex;

    /**
     * Trace source for link up/down changes.
     */
    TracedCallback<> m_linkChanges;

    /**
     * Upper layer callback used for notification of new data packet arrivals.
     */
    ReceiveCallback m_receiveCallback;
};

}

} // namespace ns3

#endif /* LR_WPAN_NET_DEVICE_H */
