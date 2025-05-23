/*
 * Copyright (c) 2008 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
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
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Mirko Banchi <mk.banchi@gmail.com>
 */

#include "wifi-helper.h"

#include "ns3/ampdu-subframe-header.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/config.h"
#include "ns3/eht-configuration.h"
#include "ns3/he-configuration.h"
#include "ns3/ht-configuration.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/names.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/obss-pd-algorithm.h"
#include "ns3/pointer.h"
#include "ns3/qos-txop.h"
#include "ns3/qos-utils.h"
#include "ns3/radiotap-header.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/vht-configuration.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-mac-trailer.h"
#include "ns3/wifi-net-device.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("WifiHelper");

/**
 * ASCII trace PHY transmit sink with context
 * \param stream the output stream
 * \param context the context name
 * \param p the packet
 * \param mode the wifi mode
 * \param preamble the wifi preamble
 * \param txLevel the transmit power level
 */
static void
AsciiPhyTransmitSinkWithContext(Ptr<OutputStreamWrapper> stream,
                                std::string context,
                                Ptr<const Packet> p,
                                WifiMode mode,
                                WifiPreamble preamble,
                                uint8_t txLevel)
{
    NS_LOG_FUNCTION(stream << context << p << mode << preamble << txLevel);
    auto pCopy = p->Copy();
    WifiMacTrailer fcs;
    pCopy->RemoveTrailer(fcs);
    *stream->GetStream() << "t " << Simulator::Now().GetSeconds() << " " << context << " " << mode
                         << " " << *pCopy << " " << fcs << std::endl;
}

/**
 * ASCII trace PHY transmit sink without context
 * \param stream the output stream
 * \param p the packet
 * \param mode the wifi mode
 * \param preamble the wifi preamble
 * \param txLevel the transmit power level
 */
static void
AsciiPhyTransmitSinkWithoutContext(Ptr<OutputStreamWrapper> stream,
                                   Ptr<const Packet> p,
                                   WifiMode mode,
                                   WifiPreamble preamble,
                                   uint8_t txLevel)
{
    NS_LOG_FUNCTION(stream << p << mode << preamble << txLevel);
    auto pCopy = p->Copy();
    WifiMacTrailer fcs;
    pCopy->RemoveTrailer(fcs);
    *stream->GetStream() << "t " << Simulator::Now().GetSeconds() << " " << mode << " " << *pCopy
                         << " " << fcs << std::endl;
}

/**
 * ASCII trace PHY receive sink with context
 * \param stream the output stream
 * \param context the context name
 * \param p the packet
 * \param snr the SNR
 * \param mode the wifi mode
 * \param preamble the wifi preamble
 */
static void
AsciiPhyReceiveSinkWithContext(Ptr<OutputStreamWrapper> stream,
                               std::string context,
                               Ptr<const Packet> p,
                               double snr,
                               WifiMode mode,
                               WifiPreamble preamble)
{
    NS_LOG_FUNCTION(stream << context << p << snr << mode << preamble);
    auto pCopy = p->Copy();
    WifiMacTrailer fcs;
    pCopy->RemoveTrailer(fcs);
    *stream->GetStream() << "r " << Simulator::Now().GetSeconds() << " " << mode << " " << context
                         << " " << *pCopy << " " << fcs << std::endl;
}

/**
 * ASCII trace PHY receive sink without context
 * \param stream the output stream
 * \param p the packet
 * \param snr the SNR
 * \param mode the wifi mode
 * \param preamble the wifi preamble
 */
static void
AsciiPhyReceiveSinkWithoutContext(Ptr<OutputStreamWrapper> stream,
                                  Ptr<const Packet> p,
                                  double snr,
                                  WifiMode mode,
                                  WifiPreamble preamble)
{
    NS_LOG_FUNCTION(stream << p << snr << mode << preamble);
    auto pCopy = p->Copy();
    WifiMacTrailer fcs;
    pCopy->RemoveTrailer(fcs);
    *stream->GetStream() << "r " << Simulator::Now().GetSeconds() << " " << mode << " " << *pCopy
                         << " " << fcs << std::endl;
}

WifiPhyHelper::WifiPhyHelper(uint8_t nLinks)
    : m_pcapDlt(PcapHelper::DLT_IEEE802_11)
{
    NS_ABORT_IF(nLinks == 0);
    m_phys.resize(nLinks);
    m_errorRateModel.resize(nLinks);
    m_frameCaptureModel.resize(nLinks);
    m_preambleDetectionModel.resize(nLinks);

    SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel");
}

WifiPhyHelper::~WifiPhyHelper()
{
}

void
WifiPhyHelper::Set(std::string name, const AttributeValue& v)
{
    for (auto& phy : m_phys)
    {
        phy.Set(name, v);
    }
}

void
WifiPhyHelper::Set(uint8_t linkId, std::string name, const AttributeValue& v)
{
    m_phys.at(linkId).Set(name, v);
}

void
WifiPhyHelper::DisablePreambleDetectionModel()
{
    for (auto& preambleDetectionModel : m_preambleDetectionModel)
    {
        preambleDetectionModel.SetTypeId(TypeId());
    }
}

void
WifiPhyHelper::PcapSniffTxEvent(Ptr<PcapFileWrapper> file,
                                Ptr<const Packet> packet,
                                uint16_t channelFreqMhz,
                                WifiTxVector txVector,
                                MpduInfo aMpdu,
                                uint16_t staId)
{
    uint32_t dlt = file->GetDataLinkType();
    switch (dlt)
    {
    case PcapHelper::DLT_IEEE802_11:
        file->Write(Simulator::Now(), packet);
        return;
    case PcapHelper::DLT_PRISM_HEADER: {
        NS_FATAL_ERROR("PcapSniffTxEvent(): DLT_PRISM_HEADER not implemented");
        return;
    }
    case PcapHelper::DLT_IEEE802_11_RADIO: {
        Ptr<Packet> p = packet->Copy();
        RadiotapHeader header;
        GetRadiotapHeader(header, p, channelFreqMhz, txVector, aMpdu, staId);
        p->AddHeader(header);
        file->Write(Simulator::Now(), p);
        return;
    }
    default:
        NS_ABORT_MSG("PcapSniffTxEvent(): Unexpected data link type " << dlt);
    }
}

void
WifiPhyHelper::PcapSniffRxEvent(Ptr<PcapFileWrapper> file,
                                Ptr<const Packet> packet,
                                uint16_t channelFreqMhz,
                                WifiTxVector txVector,
                                MpduInfo aMpdu,
                                SignalNoiseDbm signalNoise,
                                uint16_t staId)
{
    uint32_t dlt = file->GetDataLinkType();
    switch (dlt)
    {
    case PcapHelper::DLT_IEEE802_11:
        file->Write(Simulator::Now(), packet);
        return;
    case PcapHelper::DLT_PRISM_HEADER: {
        NS_FATAL_ERROR("PcapSniffRxEvent(): DLT_PRISM_HEADER not implemented");
        return;
    }
    case PcapHelper::DLT_IEEE802_11_RADIO: {
        Ptr<Packet> p = packet->Copy();
        RadiotapHeader header;
        GetRadiotapHeader(header, p, channelFreqMhz, txVector, aMpdu, staId, signalNoise);
        p->AddHeader(header);
        file->Write(Simulator::Now(), p);
        return;
    }
    default:
        NS_ABORT_MSG("PcapSniffRxEvent(): Unexpected data link type " << dlt);
    }
}

void
WifiPhyHelper::GetRadiotapHeader(RadiotapHeader& header,
                                 Ptr<Packet> packet,
                                 uint16_t channelFreqMhz,
                                 WifiTxVector txVector,
                                 MpduInfo aMpdu,
                                 uint16_t staId,
                                 SignalNoiseDbm signalNoise)
{
    header.SetAntennaSignalPower(signalNoise.signal);
    header.SetAntennaNoisePower(signalNoise.noise);
    GetRadiotapHeader(header, packet, channelFreqMhz, txVector, aMpdu, staId);
}

void
WifiPhyHelper::GetRadiotapHeader(RadiotapHeader& header,
                                 Ptr<Packet> packet,
                                 uint16_t channelFreqMhz,
                                 WifiTxVector txVector,
                                 MpduInfo aMpdu,
                                 uint16_t staId)
{
    WifiPreamble preamble = txVector.GetPreambleType();

    uint8_t frameFlags = RadiotapHeader::FRAME_FLAG_NONE;
    header.SetTsft(Simulator::Now().GetMicroSeconds());

    // Our capture includes the FCS, so we set the flag to say so.
    frameFlags |= RadiotapHeader::FRAME_FLAG_FCS_INCLUDED;

    if (preamble == WIFI_PREAMBLE_SHORT)
    {
        frameFlags |= RadiotapHeader::FRAME_FLAG_SHORT_PREAMBLE;
    }

    if (txVector.GetGuardInterval() == 400)
    {
        frameFlags |= RadiotapHeader::FRAME_FLAG_SHORT_GUARD;
    }

    header.SetFrameFlags(frameFlags);

    uint64_t rate = 0;
    if (txVector.GetMode(staId).GetModulationClass() != WIFI_MOD_CLASS_HT &&
        txVector.GetMode(staId).GetModulationClass() != WIFI_MOD_CLASS_VHT &&
        txVector.GetMode(staId).GetModulationClass() != WIFI_MOD_CLASS_HE)
    {
        rate = txVector.GetMode(staId).GetDataRate(txVector.GetChannelWidth(),
                                                   txVector.GetGuardInterval(),
                                                   1) *
               txVector.GetNss(staId) / 500000;
        header.SetRate(static_cast<uint8_t>(rate));
    }

    uint16_t channelFlags = 0;
    switch (rate)
    {
    case 2:  // 1Mbps
    case 4:  // 2Mbps
    case 10: // 5Mbps
    case 22: // 11Mbps
        channelFlags |= RadiotapHeader::CHANNEL_FLAG_CCK;
        break;
    default:
        channelFlags |= RadiotapHeader::CHANNEL_FLAG_OFDM;
        break;
    }

    if (channelFreqMhz < 2500)
    {
        channelFlags |= RadiotapHeader::CHANNEL_FLAG_SPECTRUM_2GHZ;
    }
    else
    {
        channelFlags |= RadiotapHeader::CHANNEL_FLAG_SPECTRUM_5GHZ;
    }

    header.SetChannelFrequencyAndFlags(channelFreqMhz, channelFlags);

    if (txVector.GetMode(staId).GetModulationClass() == WIFI_MOD_CLASS_HT)
    {
        uint8_t mcsKnown = RadiotapHeader::MCS_KNOWN_NONE;
        uint8_t mcsFlags = RadiotapHeader::MCS_FLAGS_NONE;

        mcsKnown |= RadiotapHeader::MCS_KNOWN_INDEX;

        mcsKnown |= RadiotapHeader::MCS_KNOWN_BANDWIDTH;
        if (txVector.GetChannelWidth() == 40)
        {
            mcsFlags |= RadiotapHeader::MCS_FLAGS_BANDWIDTH_40;
        }

        mcsKnown |= RadiotapHeader::MCS_KNOWN_GUARD_INTERVAL;
        if (txVector.GetGuardInterval() == 400)
        {
            mcsFlags |= RadiotapHeader::MCS_FLAGS_GUARD_INTERVAL;
        }

        mcsKnown |= RadiotapHeader::MCS_KNOWN_HT_FORMAT;

        mcsKnown |= RadiotapHeader::MCS_KNOWN_NESS;
        if (txVector.GetNess() & 0x01) // bit 1
        {
            mcsFlags |= RadiotapHeader::MCS_FLAGS_NESS_BIT_0;
        }
        if (txVector.GetNess() & 0x02) // bit 2
        {
            mcsKnown |= RadiotapHeader::MCS_KNOWN_NESS_BIT_1;
        }

        mcsKnown |= RadiotapHeader::MCS_KNOWN_FEC_TYPE; // only BCC is currently supported

        mcsKnown |= RadiotapHeader::MCS_KNOWN_STBC;
        if (txVector.IsStbc())
        {
            mcsFlags |= RadiotapHeader::MCS_FLAGS_STBC_STREAMS;
        }

        header.SetMcsFields(mcsKnown, mcsFlags, txVector.GetMode(staId).GetMcsValue());
    }

    if (txVector.IsAggregation())
    {
        uint16_t ampduStatusFlags = RadiotapHeader::A_MPDU_STATUS_NONE;
        ampduStatusFlags |= RadiotapHeader::A_MPDU_STATUS_LAST_KNOWN;
        /* For PCAP file, MPDU Delimiter and Padding should be removed by the MAC Driver */
        AmpduSubframeHeader hdr;
        uint32_t extractedLength;
        packet->RemoveHeader(hdr);
        extractedLength = hdr.GetLength();
        packet = packet->CreateFragment(0, static_cast<uint32_t>(extractedLength));
        if (aMpdu.type == LAST_MPDU_IN_AGGREGATE || (hdr.GetEof() && hdr.GetLength() > 0))
        {
            ampduStatusFlags |= RadiotapHeader::A_MPDU_STATUS_LAST;
        }
        header.SetAmpduStatus(aMpdu.mpduRefNumber, ampduStatusFlags, 1 /*CRC*/);
    }

    if (txVector.GetMode(staId).GetModulationClass() == WIFI_MOD_CLASS_VHT)
    {
        uint16_t vhtKnown = RadiotapHeader::VHT_KNOWN_NONE;
        uint8_t vhtFlags = RadiotapHeader::VHT_FLAGS_NONE;
        uint8_t vhtBandwidth = 0;
        uint8_t vhtMcsNss[4] = {0, 0, 0, 0};
        uint8_t vhtCoding = 0;
        uint8_t vhtGroupId = 0;
        uint16_t vhtPartialAid = 0;

        vhtKnown |= RadiotapHeader::VHT_KNOWN_STBC;
        if (txVector.IsStbc())
        {
            vhtFlags |= RadiotapHeader::VHT_FLAGS_STBC;
        }

        vhtKnown |= RadiotapHeader::VHT_KNOWN_GUARD_INTERVAL;
        if (txVector.GetGuardInterval() == 400)
        {
            vhtFlags |= RadiotapHeader::VHT_FLAGS_GUARD_INTERVAL;
        }

        vhtKnown |= RadiotapHeader::VHT_KNOWN_BEAMFORMED; // Beamforming is currently not supported

        vhtKnown |= RadiotapHeader::VHT_KNOWN_BANDWIDTH;
        // not all bandwidth values are currently supported
        if (txVector.GetChannelWidth() == 40)
        {
            vhtBandwidth = 1;
        }
        else if (txVector.GetChannelWidth() == 80)
        {
            vhtBandwidth = 4;
        }
        else if (txVector.GetChannelWidth() == 160)
        {
            vhtBandwidth = 11;
        }

        // only SU PPDUs are currently supported
        vhtMcsNss[0] |= (txVector.GetNss(staId) & 0x0f);
        vhtMcsNss[0] |= ((txVector.GetMode(staId).GetMcsValue() << 4) & 0xf0);

        header.SetVhtFields(vhtKnown,
                            vhtFlags,
                            vhtBandwidth,
                            vhtMcsNss,
                            vhtCoding,
                            vhtGroupId,
                            vhtPartialAid);
    }

    if (txVector.GetMode(staId).GetModulationClass() == WIFI_MOD_CLASS_HE)
    {
        uint16_t data1 = RadiotapHeader::HE_DATA1_BSS_COLOR_KNOWN |
                         RadiotapHeader::HE_DATA1_DATA_MCS_KNOWN |
                         RadiotapHeader::HE_DATA1_BW_RU_ALLOC_KNOWN;
        if (preamble == WIFI_PREAMBLE_HE_ER_SU)
        {
            data1 |= RadiotapHeader::HE_DATA1_FORMAT_EXT_SU;
        }
        else if (preamble == WIFI_PREAMBLE_HE_MU)
        {
            data1 |= RadiotapHeader::HE_DATA1_FORMAT_MU;
            data1 |= RadiotapHeader::HE_DATA1_SPTL_REUSE2_KNOWN;
        }
        else if (preamble == WIFI_PREAMBLE_HE_TB)
        {
            data1 |= RadiotapHeader::HE_DATA1_FORMAT_TRIG;
        }

        uint16_t data2 = RadiotapHeader::HE_DATA2_GI_KNOWN;
        if (preamble == WIFI_PREAMBLE_HE_MU || preamble == WIFI_PREAMBLE_HE_TB)
        {
            data2 |= RadiotapHeader::HE_DATA2_RU_OFFSET_KNOWN;
            // HeRu indices start at 1 whereas RadioTap starts at 0
            data2 |= (((txVector.GetHeMuUserInfo(staId).ru.GetIndex() - 1) << 8) & 0x3f00);
            data2 |= (((!txVector.GetHeMuUserInfo(staId).ru.GetPrimary80MHz()) << 15) & 0x8000);
        }

        uint16_t data3 = 0;
        data3 |= (txVector.GetBssColor() & 0x003f);
        data3 |= ((txVector.GetMode(staId).GetMcsValue() << 8) & 0x0f00);

        uint16_t data4 = 0;
        if (preamble == WIFI_PREAMBLE_HE_MU)
        {
            data4 |= ((staId << 4) & 0x7ff0);
        }

        uint16_t data5 = 0;
        if (preamble == WIFI_PREAMBLE_HE_MU || preamble == WIFI_PREAMBLE_HE_TB)
        {
            HeRu::RuType ruType = txVector.GetHeMuUserInfo(staId).ru.GetRuType();
            switch (ruType)
            {
            case HeRu::RU_26_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_26T;
                break;
            case HeRu::RU_52_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_52T;
                break;
            case HeRu::RU_106_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_106T;
                break;
            case HeRu::RU_242_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_242T;
                break;
            case HeRu::RU_484_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_484T;
                break;
            case HeRu::RU_996_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_996T;
                break;
            case HeRu::RU_2x996_TONE:
                data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_2x996T;
                break;
            default:
                NS_ABORT_MSG("Unexpected RU type");
            }
        }
        else if (txVector.GetChannelWidth() == 40)
        {
            data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_40MHZ;
        }
        else if (txVector.GetChannelWidth() == 80)
        {
            data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_80MHZ;
        }
        else if (txVector.GetChannelWidth() == 160)
        {
            data5 |= RadiotapHeader::HE_DATA5_DATA_BW_RU_ALLOC_160MHZ;
        }
        if (txVector.GetGuardInterval() == 1600)
        {
            data5 |= RadiotapHeader::HE_DATA5_GI_1_6;
        }
        else if (txVector.GetGuardInterval() == 3200)
        {
            data5 |= RadiotapHeader::HE_DATA5_GI_3_2;
        }

        header.SetHeFields(data1, data2, data3, data4, data5, 0);
    }

    if (preamble == WIFI_PREAMBLE_HE_MU)
    {
        // TODO: fill in fields (everything is set to 0 so far)
        std::array<uint8_t, 4> ruChannel1;
        std::array<uint8_t, 4> ruChannel2;
        header.SetHeMuFields(0, 0, ruChannel1, ruChannel2);
        header.SetHeMuPerUserFields(0, 0, 0, 0);
    }
}

void
WifiPhyHelper::SetPcapDataLinkType(SupportedPcapDataLinkTypes dlt)
{
    switch (dlt)
    {
    case DLT_IEEE802_11:
        m_pcapDlt = PcapHelper::DLT_IEEE802_11;
        return;
    case DLT_PRISM_HEADER:
        m_pcapDlt = PcapHelper::DLT_PRISM_HEADER;
        return;
    case DLT_IEEE802_11_RADIO:
        m_pcapDlt = PcapHelper::DLT_IEEE802_11_RADIO;
        return;
    default:
        NS_ABORT_MSG("WifiPhyHelper::SetPcapFormat(): Unexpected format");
    }
}

PcapHelper::DataLinkType
WifiPhyHelper::GetPcapDataLinkType() const
{
    return m_pcapDlt;
}

void
WifiPhyHelper::EnablePcapInternal(std::string prefix,
                                  Ptr<NetDevice> nd,
                                  bool promiscuous,
                                  bool explicitFilename)
{
    NS_LOG_FUNCTION(this << prefix << nd << promiscuous << explicitFilename);

    // All of the Pcap enable functions vector through here including the ones
    // that are wandering through all of devices on perhaps all of the nodes in
    // the system. We can only deal with devices of type WifiNetDevice.
    Ptr<WifiNetDevice> device = nd->GetObject<WifiNetDevice>();
    if (!device)
    {
        NS_LOG_INFO("WifiHelper::EnablePcapInternal(): Device "
                    << &device << " not of type ns3::WifiNetDevice");
        return;
    }

    NS_ABORT_MSG_IF(device->GetPhys().empty(),
                    "WifiPhyHelper::EnablePcapInternal(): Phy layer in WifiNetDevice must be set");

    PcapHelper pcapHelper;

    std::string filename;
    if (explicitFilename)
    {
        filename = prefix;
    }
    else
    {
        filename = pcapHelper.GetFilenameFromDevice(prefix, device);
    }

    uint8_t linkId = 0;
    // find the last point in the filename
    auto pos = filename.find_last_of('.');
    // if not found, set pos to filename size
    pos = (pos == std::string::npos) ? filename.size() : pos;

    for (auto& phy : device->GetPhys())
    {
        std::string tmp = filename;
        if (device->GetNPhys() > 1)
        {
            // insert LinkId only for multi-link devices
            tmp.insert(pos, "-" + std::to_string(linkId++));
        }
        auto file = pcapHelper.CreateFile(tmp, std::ios::out, m_pcapDlt);
        phy->TraceConnectWithoutContext("MonitorSnifferTx",
                                        MakeBoundCallback(&WifiPhyHelper::PcapSniffTxEvent, file));
        phy->TraceConnectWithoutContext("MonitorSnifferRx",
                                        MakeBoundCallback(&WifiPhyHelper::PcapSniffRxEvent, file));
    }
}

void
WifiPhyHelper::EnableAsciiInternal(Ptr<OutputStreamWrapper> stream,
                                   std::string prefix,
                                   Ptr<NetDevice> nd,
                                   bool explicitFilename)
{
    // All of the ASCII enable functions vector through here including the ones
    // that are wandering through all of devices on perhaps all of the nodes in
    // the system. We can only deal with devices of type WifiNetDevice.
    Ptr<WifiNetDevice> device = nd->GetObject<WifiNetDevice>();
    if (!device)
    {
        NS_LOG_INFO("WifiHelper::EnableAsciiInternal(): Device "
                    << device << " not of type ns3::WifiNetDevice");
        return;
    }

    // Our trace sinks are going to use packet printing, so we have to make sure
    // that is turned on.
    Packet::EnablePrinting();

    uint32_t nodeid = nd->GetNode()->GetId();
    uint32_t deviceid = nd->GetIfIndex();
    std::ostringstream oss;

    // If we are not provided an OutputStreamWrapper, we are expected to create
    // one using the usual trace filename conventions and write our traces
    // without a context since there will be one file per context and therefore
    // the context would be redundant.
    if (!stream)
    {
        // Set up an output stream object to deal with private ofstream copy
        // constructor and lifetime issues. Let the helper decide the actual
        // name of the file given the prefix.
        AsciiTraceHelper asciiTraceHelper;

        std::string filename;
        if (explicitFilename)
        {
            filename = prefix;
        }
        else
        {
            filename = asciiTraceHelper.GetFilenameFromDevice(prefix, device);
        }

        // find the last point in the filename
        auto pos = filename.find_last_of('.');
        // if not found, set pos to filename size
        pos = (pos == std::string::npos) ? filename.size() : pos;

        for (uint8_t linkId = 0; linkId < device->GetNPhys(); linkId++)
        {
            std::string tmp = filename;
            if (device->GetNPhys() > 1)
            {
                // insert LinkId only for multi-link devices
                tmp.insert(pos, "-" + std::to_string(linkId));
            }
            auto theStream = asciiTraceHelper.CreateFileStream(tmp);
            // We could go poking through the PHY and the state looking for the
            // correct trace source, but we can let Config deal with that with
            // some search cost.  Since this is presumably happening at topology
            // creation time, it doesn't seem much of a price to pay.
            oss.str("");
            oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
                << "/$ns3::WifiNetDevice/Phys/" << +linkId << "/State/RxOk";
            Config::ConnectWithoutContext(
                oss.str(),
                MakeBoundCallback(&AsciiPhyReceiveSinkWithoutContext, theStream));

            oss.str("");
            oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
                << "/$ns3::WifiNetDevice/Phys/" << +linkId << "/State/Tx";
            Config::ConnectWithoutContext(
                oss.str(),
                MakeBoundCallback(&AsciiPhyTransmitSinkWithoutContext, theStream));
        }

        return;
    }

    // If we are provided an OutputStreamWrapper, we are expected to use it, and
    // to provide a context. We are free to come up with our own context if we
    // want, and use the AsciiTraceHelper Hook*WithContext functions, but for
    // compatibility and simplicity, we just use Config::Connect and let it deal
    // with coming up with a context.
    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::WifiNetDevice/Phy/State/RxOk";
    Config::Connect(oss.str(), MakeBoundCallback(&AsciiPhyReceiveSinkWithContext, stream));

    oss.str("");
    oss << "/NodeList/" << nodeid << "/DeviceList/" << deviceid
        << "/$ns3::WifiNetDevice/Phy/State/Tx";
    Config::Connect(oss.str(), MakeBoundCallback(&AsciiPhyTransmitSinkWithContext, stream));
}

WifiHelper::~WifiHelper()
{
}

WifiHelper::WifiHelper()
    : m_standard(WIFI_STANDARD_80211ax),
      m_selectQueueCallback(&SelectQueueByDSField),
      m_enableFlowControl(true)
{
    SetRemoteStationManager("ns3::IdealWifiManager");
    m_htConfig.SetTypeId("ns3::HtConfiguration");
    m_vhtConfig.SetTypeId("ns3::VhtConfiguration");
    m_heConfig.SetTypeId("ns3::HeConfiguration");
    m_ehtConfig.SetTypeId("ns3::EhtConfiguration");
}

void
WifiHelper::SetStandard(WifiStandard standard)
{
    m_standard = standard;
}

void
WifiHelper::DisableFlowControl()
{
    m_enableFlowControl = false;
}

void
WifiHelper::SetSelectQueueCallback(SelectQueueCallback f)
{
    m_selectQueueCallback = f;
}

NetDeviceContainer
WifiHelper::Install(const WifiPhyHelper& phyHelper,
                    const WifiMacHelper& macHelper,
                    NodeContainer::Iterator first,
                    NodeContainer::Iterator last) const
{
    NetDeviceContainer devices;
    for (auto i = first; i != last; ++i)
    {
        Ptr<Node> node = *i;
        Ptr<WifiNetDevice> device = CreateObject<WifiNetDevice>();
        node->AddDevice(device);
        device->SetStandard(m_standard);
        if (m_standard == WIFI_STANDARD_UNSPECIFIED)
        {
            NS_FATAL_ERROR("No standard specified!");
            return devices;
        }
        if (m_standard >= WIFI_STANDARD_80211n)
        {
            auto htConfiguration = m_htConfig.Create<HtConfiguration>();
            device->SetHtConfiguration(htConfiguration);
        }
        if (m_standard >= WIFI_STANDARD_80211ac)
        {
            // Create the VHT Configuration object even if the PHY band is 2.4GHz
            // (WifiNetDevice::GetVhtConfiguration() checks the PHY band being used).
            // This approach allows us not to worry about deleting this object when
            // the PHY band is switched from 5GHz to 2.4GHz and creating this object
            // when the PHY band is switched from 2.4GHz to 5GHz.
            auto vhtConfiguration = m_vhtConfig.Create<VhtConfiguration>();
            device->SetVhtConfiguration(vhtConfiguration);
        }
        if (m_standard >= WIFI_STANDARD_80211ax)
        {
            auto heConfiguration = m_heConfig.Create<HeConfiguration>();
            device->SetHeConfiguration(heConfiguration);
        }
        if (m_standard >= WIFI_STANDARD_80211be)
        {
            auto ehtConfiguration = m_ehtConfig.Create<EhtConfiguration>();
            device->SetEhtConfiguration(ehtConfiguration);
        }
        std::vector<Ptr<WifiRemoteStationManager>> managers;
        std::vector<Ptr<WifiPhy>> phys = phyHelper.Create(node, device);
        device->SetPhys(phys);
        // if only one remote station manager model was provided, replicate it for all the links
        auto stationManagers = m_stationManager;
        if (stationManagers.size() == 1 && phys.size() > 1)
        {
            stationManagers.resize(phys.size(), stationManagers[0]);
        }
        NS_ABORT_MSG_IF(stationManagers.size() != phys.size(),
                        "Number of station manager models ("
                            << stationManagers.size() << ") does not match the number of links ("
                            << phys.size() << ")");
        for (std::size_t i = 0; i < phys.size(); i++)
        {
            phys[i]->ConfigureStandard(m_standard);
            managers.push_back(stationManagers[i].Create<WifiRemoteStationManager>());
        }
        device->SetRemoteStationManagers(managers);
        Ptr<WifiMac> mac = macHelper.Create(device, m_standard);
        if ((m_standard >= WIFI_STANDARD_80211ax) && (m_obssPdAlgorithm.IsTypeIdSet()))
        {
            Ptr<ObssPdAlgorithm> obssPdAlgorithm = m_obssPdAlgorithm.Create<ObssPdAlgorithm>();
            device->AggregateObject(obssPdAlgorithm);
            obssPdAlgorithm->ConnectWifiNetDevice(device);
        }
        devices.Add(device);
        NS_LOG_DEBUG("node=" << node << ", mob=" << node->GetObject<MobilityModel>());
        if (m_enableFlowControl)
        {
            Ptr<NetDeviceQueueInterface> ndqi;
            BooleanValue qosSupported;
            Ptr<WifiMacQueue> wmq;

            mac->GetAttributeFailSafe("QosSupported", qosSupported);
            if (qosSupported.Get())
            {
                ndqi = CreateObjectWithAttributes<NetDeviceQueueInterface>("NTxQueues",
                                                                           UintegerValue(4));
                for (auto& ac : {AC_BE, AC_BK, AC_VI, AC_VO})
                {
                    Ptr<QosTxop> qosTxop = mac->GetQosTxop(ac);
                    wmq = qosTxop->GetWifiMacQueue();
                    ndqi->GetTxQueue(static_cast<std::size_t>(ac))->ConnectQueueTraces(wmq);
                }
                ndqi->SetSelectQueueCallback(m_selectQueueCallback);
            }
            else
            {
                ndqi = CreateObject<NetDeviceQueueInterface>();

                wmq = mac->GetTxop()->GetWifiMacQueue();
                ndqi->GetTxQueue(0)->ConnectQueueTraces(wmq);
            }
            device->AggregateObject(ndqi);
        }
    }
    return devices;
}

NetDeviceContainer
WifiHelper::Install(const WifiPhyHelper& phyHelper,
                    const WifiMacHelper& macHelper,
                    NodeContainer c) const
{
    return Install(phyHelper, macHelper, c.Begin(), c.End());
}

NetDeviceContainer
WifiHelper::Install(const WifiPhyHelper& phy, const WifiMacHelper& mac, Ptr<Node> node) const
{
    return Install(phy, mac, NodeContainer(node));
}

NetDeviceContainer
WifiHelper::Install(const WifiPhyHelper& phy, const WifiMacHelper& mac, std::string nodeName) const
{
    Ptr<Node> node = Names::Find<Node>(nodeName);
    return Install(phy, mac, NodeContainer(node));
}

void
WifiHelper::EnableLogComponents(LogLevel logLevel)
{
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_NODE);

    LogComponentEnable("AarfWifiManager", logLevel);
    LogComponentEnable("AarfcdWifiManager", logLevel);
    LogComponentEnable("AdhocWifiMac", logLevel);
    LogComponentEnable("AmrrWifiManager", logLevel);
    LogComponentEnable("ApWifiMac", logLevel);
    LogComponentEnable("AparfWifiManager", logLevel);
    LogComponentEnable("ArfWifiManager", logLevel);
    LogComponentEnable("BlockAckAgreement", logLevel);
    LogComponentEnable("BlockAckManager", logLevel);
    LogComponentEnable("CaraWifiManager", logLevel);
    LogComponentEnable("ChannelAccessManager", logLevel);
    LogComponentEnable("ConstantObssPdAlgorithm", logLevel);
    LogComponentEnable("ConstantRateWifiManager", logLevel);
    LogComponentEnable("DefaultEmlsrManager", logLevel);
    LogComponentEnable("DsssErrorRateModel", logLevel);
    LogComponentEnable("DsssPhy", logLevel);
    LogComponentEnable("DsssPpdu", logLevel);
    LogComponentEnable("EhtFrameExchangeManager", logLevel);
    LogComponentEnable("EhtPhy", logLevel);
    LogComponentEnable("EhtPpdu", logLevel);
    LogComponentEnable("EmlsrManager", logLevel);
    LogComponentEnable("ErpOfdmPhy", logLevel);
    LogComponentEnable("ErpOfdmPpdu", logLevel);
    LogComponentEnable("FrameExchangeManager", logLevel);
    LogComponentEnable("HeConfiguration", logLevel);
    LogComponentEnable("HeFrameExchangeManager", logLevel);
    LogComponentEnable("HePhy", logLevel);
    LogComponentEnable("HePpdu", logLevel);
    LogComponentEnable("HtConfiguration", logLevel);
    LogComponentEnable("HtFrameExchangeManager", logLevel);
    LogComponentEnable("HtPhy", logLevel);
    LogComponentEnable("HtPpdu", logLevel);
    LogComponentEnable("IdealWifiManager", logLevel);
    LogComponentEnable("InterferenceHelper", logLevel);
    LogComponentEnable("MacRxMiddle", logLevel);
    LogComponentEnable("MacTxMiddle", logLevel);
    LogComponentEnable("MinstrelHtWifiManager", logLevel);
    LogComponentEnable("MinstrelWifiManager", logLevel);
    LogComponentEnable("MpduAggregator", logLevel);
    LogComponentEnable("MsduAggregator", logLevel);
    LogComponentEnable("MultiUserScheduler", logLevel);
    LogComponentEnable("NistErrorRateModel", logLevel);
    LogComponentEnable("ObssPdAlgorithm", logLevel);
    LogComponentEnable("OfdmPhy", logLevel);
    LogComponentEnable("OfdmPpdu", logLevel);
    LogComponentEnable("OnoeWifiManager", logLevel);
    LogComponentEnable("OriginatorBlockAckAgreement", logLevel);
    LogComponentEnable("ParfWifiManager", logLevel);
    LogComponentEnable("PhyEntity", logLevel);
    LogComponentEnable("QosFrameExchangeManager", logLevel);
    LogComponentEnable("QosTxop", logLevel);
    LogComponentEnable("RecipientBlockAckAgreement", logLevel);
    LogComponentEnable("RrMultiUserScheduler", logLevel);
    LogComponentEnable("RraaWifiManager", logLevel);
    LogComponentEnable("RrpaaWifiManager", logLevel);
    LogComponentEnable("SimpleFrameCaptureModel", logLevel);
    LogComponentEnable("SpectrumWifiPhy", logLevel);
    LogComponentEnable("StaWifiMac", logLevel);
    LogComponentEnable("SupportedRates", logLevel);
    LogComponentEnable("TableBasedErrorRateModel", logLevel);
    LogComponentEnable("ThompsonSamplingWifiManager", logLevel);
    LogComponentEnable("ThresholdPreambleDetectionModel", logLevel);
    LogComponentEnable("Txop", logLevel);
    LogComponentEnable("VhtConfiguration", logLevel);
    LogComponentEnable("VhtFrameExchangeManager", logLevel);
    LogComponentEnable("VhtPhy", logLevel);
    LogComponentEnable("VhtPpdu", logLevel);
    LogComponentEnable("WifiAckManager", logLevel);
    LogComponentEnable("WifiAssocManager", logLevel);
    LogComponentEnable("WifiDefaultAckManager", logLevel);
    LogComponentEnable("WifiDefaultAssocManager", logLevel);
    LogComponentEnable("WifiDefaultProtectionManager", logLevel);
    LogComponentEnable("WifiMac", logLevel);
    LogComponentEnable("WifiMacQueue", logLevel);
    LogComponentEnable("WifiMpdu", logLevel);
    LogComponentEnable("WifiNetDevice", logLevel);
    LogComponentEnable("WifiPhyStateHelper", logLevel);
    LogComponentEnable("WifiPhyOperatingChannel", logLevel);
    LogComponentEnable("WifiPhy", logLevel);
    LogComponentEnable("WifiPpdu", logLevel);
    LogComponentEnable("WifiProtectionManager", logLevel);
    LogComponentEnable("WifiPsdu", logLevel);
    LogComponentEnable("WifiRadioEnergyModel", logLevel);
    LogComponentEnable("WifiRemoteStationManager", logLevel);
    LogComponentEnable("WifiSpectrumPhyInterface", logLevel);
    LogComponentEnable("WifiSpectrumSignalParameters", logLevel);
    LogComponentEnable("WifiTxCurrentModel", logLevel);
    LogComponentEnable("WifiTxParameters", logLevel);
    LogComponentEnable("WifiTxTimer", logLevel);
    LogComponentEnable("YansErrorRateModel", logLevel);
    LogComponentEnable("YansWifiChannel", logLevel);
    LogComponentEnable("YansWifiPhy", logLevel);

    LogComponentEnable("Athstats", logLevel);
    LogComponentEnable("WifiHelper", logLevel);
    LogComponentEnable("SpectrumWifiHelper", logLevel);
    LogComponentEnable("YansWifiHelper", logLevel);

    // From Spectrum
    LogComponentEnable("WifiSpectrumValueHelper", logLevel);
}

int64_t
WifiHelper::AssignStreams(NetDeviceContainer c, int64_t stream)
{
    int64_t currentStream = stream;
    Ptr<NetDevice> netDevice;
    for (auto i = c.Begin(); i != c.End(); ++i)
    {
        netDevice = (*i);
        if (auto wifi = DynamicCast<WifiNetDevice>(netDevice))
        {
            // Handle any random numbers in the PHY objects.
            for (auto& phy : wifi->GetPhys())
            {
                currentStream += phy->AssignStreams(currentStream);
            }

            // Handle any random numbers in the station managers.
            for (auto& manager : wifi->GetRemoteStationManagers())
            {
                currentStream += manager->AssignStreams(currentStream);
            }

            // Handle any random numbers in the MAC objects.
            auto mac = wifi->GetMac();
            PointerValue ptr;
            if (!mac->GetQosSupported())
            {
                mac->GetAttribute("Txop", ptr);
                auto txop = ptr.Get<Txop>();
                currentStream += txop->AssignStreams(currentStream);
            }
            else
            {
                mac->GetAttribute("VO_Txop", ptr);
                auto vo_txop = ptr.Get<QosTxop>();
                currentStream += vo_txop->AssignStreams(currentStream);

                mac->GetAttribute("VI_Txop", ptr);
                auto vi_txop = ptr.Get<QosTxop>();
                currentStream += vi_txop->AssignStreams(currentStream);

                mac->GetAttribute("BE_Txop", ptr);
                auto be_txop = ptr.Get<QosTxop>();
                currentStream += be_txop->AssignStreams(currentStream);

                mac->GetAttribute("BK_Txop", ptr);
                auto bk_txop = ptr.Get<QosTxop>();
                currentStream += bk_txop->AssignStreams(currentStream);
            }

            // if an AP, handle any beacon jitter
            if (auto apMac = DynamicCast<ApWifiMac>(mac); apMac)
            {
                currentStream += apMac->AssignStreams(currentStream);
            }
            // if a STA, handle any probe request jitter
            if (auto staMac = DynamicCast<StaWifiMac>(mac); staMac)
            {
                currentStream += staMac->AssignStreams(currentStream);
            }
        }
    }
    return (currentStream - stream);
}

} // namespace ns3
