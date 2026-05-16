//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "ExampleService.h"
#include "artery/traci/VehicleController.h"
#include <omnetpp/cpacket.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include "artery/application/MultiChannelPolicy.h"
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/dcc/flow_control.hpp>

using namespace omnetpp;
using namespace vanetza;

int recFlagEs = 0;

int selch = 0;

namespace artery
{


FILE *myfile3; //Registra RecExa.csv
FILE *myfile5; //Registra RecExaCh.csv

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

Define_Module(ExampleService)

ExampleService::ExampleService()
{	
	if(recFlagEs == 0){
		myfile3 = fopen("RecExa.csv", "w");
		fprintf(myfile3, "%s,%s,%s,%s\n", "nodeName","timestamp","channel","size");
		fclose(myfile3);
		recFlagEs = 1;
		
		myfile3 = fopen("SentExaCh.csv", "w");
		fprintf(myfile3, "%s,%s,%s,%s\n","nodeName","ch180","ch172","ch176");
		fclose(myfile3);
	}
	lastChannel = 0;
	roundRobin = intuniform(0,2);
	genCh[0] = 0;
	genCh[1] = 0;
	genCh[2] = 0;
}

ExampleService::~ExampleService()
{
	cancelAndDelete(m_self_msg);
}

void ExampleService::indicate(const btp::DataIndication& ind, cPacket* packet, const NetworkInterface& net)
{
	Enter_Method("indicate");

	if (packet->getByteLength() == 640 || packet->getByteLength() == 320) {
		EV_INFO << "packet indication on channel " << net.channel << "\n";
		myfile3 = fopen("RecExa.csv", "a");
		unsigned int canal = net.channel;
		fprintf(myfile3, "%s,%f,%d,%d\n", findHost()->getFullName(),SIMTIME_DBL(simTime()),canal,packet->getByteLength());
		fclose(myfile3);
	}

	delete(packet);
}

void ExampleService::initialize()
{
	ItsG5Service::initialize();
	m_self_msg = new cMessage("Example Service");
	subscribe(scSignalCamReceived);


	scheduleAt(simTime() + 1.0, m_self_msg);
}

void ExampleService::finish()
{
	// you could record some scalars at this point
	myfile3 = fopen("SentExaCh.csv", "a");
	fprintf(myfile3, "%s,%d,%d,%d\n",findHost()->getFullName(),genCh[0],genCh[1],genCh[2]);
	fclose(myfile3);
	ItsG5Service::finish();
}

void ExampleService::handleMessage(cMessage* msg)
{
	Enter_Method("handleMessage");

	if (msg == m_self_msg) {
		EV_INFO << "self message\n";
	}
}

void ExampleService::trigger()
{
	Enter_Method("trigger");
	checkTriggeringConditions(simTime());
}
void ExampleService::checkTriggeringConditions(const SimTime& T_now)
{
	if((T_now - mLastExaTimestamp) >= mGenExa){
	// use an ITS-AID reserved for testing purposes
	static const vanetza::ItsAid example_its_aid = 16480;

	auto& mco = getFacilities().get_const<MultiChannelPolicy>();
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
	double channelsDcc[3][2];

	channelsDcc[0][0] = 180;
	//channelsDcc[0][1] = getQueueOccupancy((int)channelsDcc[0][0],2) * SIMTIME_DBL(genInterval((int)channelsDcc[0][0],2)) + SIMTIME_DBL(genInterval((int)channelsDcc[0][0],2)) + SIMTIME_DBL(genGot((int)channelsDcc[0][0],2));
	channelsDcc[0][1] = getCbr((int)channelsDcc[0][0]);
	channelsDcc[1][0] = 172;
	channelsDcc[1][1] = getCbr((int)channelsDcc[1][0]);
	//channelsDcc[1][1] = SIMTIME_DBL(genInterval((int)channelsDcc[1][0],2));
	channelsDcc[2][0] = 176;
	channelsDcc[2][1] = getCbr((int)channelsDcc[2][0]);
	//channelsDcc[2][1] = SIMTIME_DBL(genInterval((int)channelsDcc[2][0],2));
	
	int selectedChannel;
	
	int randomizer = intuniform(0,2);
	/*
	selectedChannel = (int)channelsDcc[randomizer][0];
	double minDelay = channelsDcc[randomizer][1];
	for(int i = 0; i < 3 ; i++){
		//std::cout << findHost()->getFullName() << " Channel " << channelsDcc[i][0]  << " Delay " << channelsDcc[i][1] <<"\n";
		if(channelsDcc[i][1] < minDelay){
			//std::cout << "Delay "<< minDelay <<"\n";
			minDelay = channelsDcc[i][1];
			selch = i;
			selectedChannel = (int)channelsDcc[i][0];
			//std::cout << "Changed channel "<< selectedChannel <<"\n";
			//std::cout << "Delay "<< minDelay <<"\n";
		} else {
			if(channelsDcc[i][1] == minDelay){
				if(intuniform(0,3) == 0){
					minDelay = channelsDcc[i][1];
					selch = i;
					selectedChannel = (int)channelsDcc[i][0];
					//std::cout << "Changed channel "<< selectedChannel <<"\n";
					//std::cout << "Delay "<< minDelay <<"\n";
				}	
			}
		}
	}
	*/
	
	selectedChannel = (int)channelsDcc[0][0];
	selch = 0;
	if(channelsDcc[0][1] > 0.55){
		if(channelsDcc[1][1] < 0.55){ 
			selectedChannel = (int)channelsDcc[1][0];
			selch = 1;
		} else {
			if(channelsDcc[2][1] < 0.55){
				selectedChannel = (int)channelsDcc[2][0];
				selch = 2;
			} else {
				selch = randomizer;
				selectedChannel = (int)channelsDcc[randomizer][0];
			}
		}
	}
	/*
	if(lastChannel == 0){ 
		selectedChannel = (int)channelsDcc[roundRobin][0];
		selch = roundRobin;
	} else {
		roundRobin++; 
		selectedChannel = (int)channelsDcc[roundRobin%3][0];
		selch = roundRobin%3;
	}
	*/
	int packetSize = 320;//intuniform(0,1)*320 + 320;
	//for (auto channel : mco.allChannels(example_its_aid)) {
		auto network = networks.select(selectedChannel);
		//std::cout << "Selected channel "<< selectedChannel <<"\n";
		//std::cout << "Primary channel " << getFacilities().get_const<MultiChannelPolicy>().primaryChannel(example_its_aid) <<"\n";
		// OAM Medir si el cbr esta por debajo de 60
		
			btp::DataRequestB req;
				// use same port number as configured for listening on this channel
			req.destination_port = host_cast(getPortNumber(selectedChannel));
			network = networks.select(selectedChannel);
			auto netifc = network;
			req.gn.transport_type = geonet::TransportType::SHB;
			//req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP0));
			if(selectedChannel != 180){
				req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
			} else {
				req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
			} 
			req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			req.gn.its_aid = example_its_aid;

			cPacket* packet = new cPacket("Example Service Packet");
			packet->setByteLength(packetSize);
			//std::cout << findHost()->getFullName() << "Transmitting in channel" << selectedChannel << "\n";
			lastChannel = selectedChannel;
			genCh[selch] = genCh[selch] + 1;
			// send packet on specific network interface
			request(req, packet, network.get());
			mLastExaTimestamp = T_now;
			mGenExa = std::min(1.0,std::max(exponential(0.025),0.001));
			
		
	//}
	}
}

void ExampleService::receiveSignal(cComponent* source, simsignal_t signal, cObject*, cObject*)
{
	Enter_Method("receiveSignal");

	if (signal == scSignalCamReceived) {
		auto& vehicle = getFacilities().get_const<traci::VehicleController>();
		EV_INFO << "Vehicle " << vehicle.getVehicleId() << " received a CAM in sibling serivce\n";
	}
}

SimTime ExampleService::genInterval(int channel, int tc )
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
	auto network = networks.select(channel);
	auto netifc = network;
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", channel);
	}

	static vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	//vanetza::Clock::duration interval = trc->interval(ca_tx);
	vanetza::Clock::duration interval;
	if(tc == 0){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP0, 0);
		interval = trc->interval(ca_tx);
	} else if (tc == 1){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP1, 0);
		interval = trc->interval(ca_tx);
	} else if (tc == 2){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
		interval = trc->interval(ca_tx);
	} else if (tc == 3){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP3, 0);
		interval = trc->interval(ca_tx);
	}
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return dcc;
}

SimTime ExampleService::genGot(int channel, int tc)
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
	auto network = networks.select(channel);
	auto netifc = network;
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", channel);
	}

	static vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	//vanetza::Clock::duration interval = trc->interval(ca_tx);
	vanetza::Clock::duration interval;
	if(tc == 0){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP0, 0);
		interval = trc->delay(ca_tx);
	} else if (tc == 1){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP1, 0);
		interval = trc->delay(ca_tx);
	} else if (tc == 2){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
		interval = trc->delay(ca_tx);
	} else if (tc == 3){
		vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP3, 0);
		interval = trc->delay(ca_tx);
	}
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return dcc;
}

int ExampleService::getQueueOccupancy(int channel, int tc)
{
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
	auto network = networks.select(channel);
	auto netifc = network;
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	auto& dcc_entity = netifc->getDccEntity();
	auto* req = dcc_entity.getRequestInterface();
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", channel);
	}
	auto* fc = dynamic_cast<vanetza::dcc::FlowControl*>(req);
	auto ac = vanetza::access::AccessCategory::BE;

	if(fc){
		auto size = fc->get_size(vanetza::access::AccessCategory::VO);
		if(tc == 1){
			size += fc->get_size(vanetza::access::AccessCategory::VI);
		}		
		if(tc == 2){
			size += fc->get_size(vanetza::access::AccessCategory::VI);
			size += fc->get_size(vanetza::access::AccessCategory::BE);
		}		
		if(tc == 3){
			size += fc->get_size(vanetza::access::AccessCategory::VI);
			size += fc->get_size(vanetza::access::AccessCategory::BE);
			size += fc->get_size(vanetza::access::AccessCategory::BK);
		}
		//if (size > 0) std::cout << findHost()->getFullName() << " " << size << " packets waiting on channel " << channel << "\n";
		return (int)size;
	}

	return 0;	
}
double ExampleService::getCbr(int channel)
{
	
	auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
	auto network = networks.select(channel);
	auto netifc = network;

	if(!netifc) return -100.0;
	const auto& dcc_entity = netifc->getDccEntity();

	return dcc_entity.getChannelLoad().value();
}

} // namespace artery
