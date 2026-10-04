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



namespace artery
{


FILE *myfile3; //Registra RecExa.csv
FILE *myfile5; //Registra RecExaCh.csv
FILE *myfile7; //Registra MessageTriggerCLR.csv

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");

Define_Module(ExampleService)

ExampleService::ExampleService()
{	
	if(recFlagEs == 0){
		myfile3 = fopen("RecExa.csv", "w");
		fprintf(myfile3, "%s,%s,%s,%s\n", "nodeName","timestamp","channel","size");
		fclose(myfile3);
		recFlagEs = 1;
		
		myfile5 = fopen("SentExaCh.csv", "w");
		fprintf(myfile5, "%s,%s,%s,%s,%s,%s,%s\n","nodeName","ch180","ch172","ch176","Triggered","Discarded","avgPlace");
		fclose(myfile5);
		
		myfile7 = fopen("MessageTriggerCLR.csv", "w");
		fprintf(myfile7, "%s,%s,%s,%s,%s,%s\n","nodeName","timestamp","180","172","176","Selected");
		fclose(myfile7);
	}
	lastChannel = 0;
	roundRobin = intuniform(0,2);
	genCh[0] = 0;
	genCh[1] = 0;
	genCh[2] = 0;
	genCh[3] = 0;


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
	mAliSelection = par("aliSelection");
	if(mAliSelection > 5 || mAliSelection < 0) mAliSelection = 0;
	mSeqFillTh = par("seqFillThreshold");
	mCasfTh = par("casfThreshold");
	tcPrim = par("tcPrimary");
	tcAlt = par("tcAlternate");
	genRate = par("genRate");
	queueTrigger =  par("queueFactor");
	mdcPolicy = par("handlingPolicy");
	occPolicy = par("occPolicy");

	cModule* app = getParentModule();
	mCaService = dynamic_cast<CaService*>(app->getSubmodule("CaService"));
	mRequestedCamRate = mCaService->getRequestedCamRate();
	mRequestedExaRate = 1.0;

	cModule* dccEnity = getModuleByPath("^.^.vanetza[0].dcc");
	if(!dccEnity) throw cRuntimeError("DCC module not found");
	dccQueueLength = dccEnity->par("queueLength");

	scheduleAt(simTime() + 1.0, m_self_msg);
}

void ExampleService::finish()
{
	// you could record some scalars at this point
	myfile3 = fopen("SentExaCh.csv", "a");
	fprintf(myfile3, "%s,%d,%d,%d,%d,%d,%f\n",findHost()->getFullName(),genCh[0],genCh[1],genCh[2],countDesired,genCh[3],avgQueuePlace/(genCh[0]+genCh[1]+genCh[2]));
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

int ExampleService::calis(std::vector<int> candidateChannels)
{
	int selectedChannel = -1;
	selch = -1;
	if(!candidateChannels.empty()){
		int n = static_cast<int>(candidateChannels.size());
		double channelsDcc[n][3];
		int nCand = 0;
		
		for(int i = 0; i < n; i++){
			int tc = tcAlt;
			if(candidateChannels[i] == 180) tc = tcPrim;
			channelsDcc[i][0] = candidateChannels[i];
			channelsDcc[i][1] = getQueueOccupancy((int)channelsDcc[i][0],tc) * SIMTIME_DBL(genInterval((int)channelsDcc[i][0],tc)) + SIMTIME_DBL(genInterval((int)channelsDcc[i][0],tc)) + SIMTIME_DBL(genGot((int)channelsDcc[i][0],tc));
			channelsDcc[i][2] = getCbr((int)channelsDcc[i][0])*1000;
			nCand++;
		}

		
		std::vector<int> candidates; 
		int randomizer = intuniform(0,nCand-1);
		
		selectedChannel = (int)channelsDcc[randomizer][0];
		double minDelay = channelsDcc[randomizer][1];
		int candidateCBR = (int)channelsDcc[randomizer][2];
		for(int i = 0; i < nCand ; i++){
			if(channelsDcc[i][1] < minDelay){
				minDelay = channelsDcc[i][1];
				candidateCBR = channelsDcc[i][2];
				candidates.clear();
				candidates.push_back(channelsDcc[i][0]);
			} else {
				if(channelsDcc[i][1] == minDelay){
					if((int)channelsDcc[i][2] < candidateCBR){
						minDelay = channelsDcc[i][1];
						candidateCBR = (int)channelsDcc[i][2];
						candidates.clear();
						candidates.push_back(channelsDcc[i][0]);
					} else if((int)channelsDcc[i][2] == candidateCBR) {
						 candidates.push_back(channelsDcc[i][0]);
					}	
				}
			}
		}
		
		

		if(!candidates.empty()){
			int randomCandidate = intuniform(0,candidates.size()-1);
			selectedChannel = candidates[randomCandidate];
			if (selectedChannel == 180) selch = 0;
			if (selectedChannel == 172) selch = 1;
			if (selectedChannel == 176) selch = 2;
		}/**/
	}
	
	return selectedChannel;
}

int ExampleService::minCBR(std::vector<int> candidateChannels)
{
	
	int selectedChannel = -1;
	selch = -1;
	if(!candidateChannels.empty()){
		int n = static_cast<int>(candidateChannels.size());
		double channelsDcc[n][2];
		int nCand = 0;
		
		for(int i = 0; i < n; i++){
			channelsDcc[i][0] = candidateChannels[i];
			channelsDcc[i][1] = getCbr((int)channelsDcc[i][0]);
			nCand++;
		}


		int selectedChannel;
		std::vector<int> candidates; 
		int randomizer = intuniform(0,nCand);
	
		selectedChannel = (int)channelsDcc[randomizer][0];
		double minLoad = channelsDcc[randomizer][1];
		for(int i = 0; i < 3 ; i++){
			if(channelsDcc[i][1] < minLoad){
				minLoad = channelsDcc[i][1];
				candidates.clear();
				candidates.push_back(channelsDcc[i][0]);
			} else {
				if(channelsDcc[i][1] == minLoad){
					candidates.push_back(channelsDcc[i][0]);	
				}
			}
		}
		if(!candidates.empty()){
			int randomCandidate = intuniform(0,candidates.size()-1);
			selectedChannel = candidates[randomCandidate];
			if (selectedChannel == 180) selch = 0;
			if (selectedChannel == 172) selch = 1;
			if (selectedChannel == 176) selch = 2;
		}/**/
	}
	return selectedChannel;
}


int ExampleService::minTRC(std::vector<int> candidateChannels)
{
	int selectedChannel = -1;
	selch = -1;
	if(!candidateChannels.empty()){
		int n = static_cast<int>(candidateChannels.size());
		double channelsDcc[n][2];
		int nCand = 0;
		
		for(int i = 0; i < n; i++){
			int tc = tcAlt;
			if(candidateChannels[i] == 180) tc = tcPrim;
			channelsDcc[i][0] = candidateChannels[i];
			channelsDcc[i][1] = SIMTIME_DBL(genInterval((int)channelsDcc[i][0],tc));
			nCand++;
		}


		int selectedChannel;
		std::vector<int> candidates; 
		int randomizer = intuniform(0,nCand);
	
		selectedChannel = (int)channelsDcc[randomizer][0];
		double minLoad = channelsDcc[randomizer][1];
		for(int i = 0; i < 3 ; i++){
			if(channelsDcc[i][1] < minLoad){
				minLoad = channelsDcc[i][1];
				candidates.clear();
				candidates.push_back(channelsDcc[i][0]);
			} else {
				if(channelsDcc[i][1] == minLoad){
					candidates.push_back(channelsDcc[i][0]);	
				}
			}
		}
		if(!candidates.empty()){
			int randomCandidate = intuniform(0,candidates.size()-1);
			selectedChannel = candidates[randomCandidate];
			if (selectedChannel == 180) selch = 0;
			if (selectedChannel == 172) selch = 1;
			if (selectedChannel == 176) selch = 2;
		}/**/
	}
	return selectedChannel;
}

int ExampleService::loadBalancing(std::vector<int> candidateChannels)
{
	int selectedChannel = -1;
	selch = -1;
	int n = static_cast<int>(candidateChannels.size());
	if(!candidateChannels.empty() && n == 3){

		double channelsDcc[n];
		int nCand = 0;
		
		for(int i = 0; i < n; i++){
			int tc = tcAlt;
			if(candidateChannels[i] == 180) tc = tcPrim;
			channelsDcc[i] = candidateChannels[i];
			nCand++;
		}

		if(lastChannel == 0){ 
			selectedChannel = (int)channelsDcc[roundRobin];
			selch = roundRobin;
		} else {
			roundRobin++; 
			selectedChannel = (int)channelsDcc[roundRobin%3];
			selch = roundRobin%3;
		}
		lastChannel = selectedChannel;	
	}

	
	return selectedChannel;
}

int ExampleService::seqFillCBR(std::vector<int> candidateChannels)
{
	int selectedChannel = -1;
	selch = -1;
	int n = static_cast<int>(candidateChannels.size());
	if(!candidateChannels.empty()){

		double channelsDcc[n][2];
		int nCand = 0;
		
		for(int i = 0; i < n; i++){
			int tc = tcAlt;
			if(candidateChannels[i] == 180) tc = tcPrim;
			channelsDcc[i][0] = candidateChannels[i];
			channelsDcc[i][1] = getCbr((int)channelsDcc[i][0]);
			nCand++;
		}
	
		
		int randomizer = intuniform(0,nCand-1);

		selectedChannel = (int)channelsDcc[0][0];
		selch = 0;
		bool success = false;
		if(channelsDcc[0][1] > mSeqFillTh){
			for(int i = 1; i < nCand; i++){
				if(channelsDcc[i][1] < mSeqFillTh && success == false){
					success = true;
					selectedChannel = (int)channelsDcc[i][0];
					selch = i;
				}
			}
			if(success == false){
				if(mdcPolicy == 0){
						selch = randomizer;
						selectedChannel = (int)channelsDcc[randomizer][0];
					} else if(mdcPolicy == 1){
						selch = -1;
						selectedChannel = -1;
					} else if(mdcPolicy == 2){
						selch = minCBR(candidateChannels);
						selectedChannel = selch;
					}
			}
		}
			
	}

	return selectedChannel; 
}

int ExampleService::casf(std::vector<int> candidateChannels)
{
	int selectedChannel = -1;
	selch = -1;
	if(!candidateChannels.empty()){
		int n = static_cast<int>(candidateChannels.size());
		double channelsDcc[n][2];
		int nCand = 0;
		
		for(int i = 0; i < n; i++){
			int tc = tcAlt;
			if(candidateChannels[i] == 180) tc = tcPrim;
			channelsDcc[i][0] = candidateChannels[i];
			channelsDcc[i][1] = getQueueOccupancy((int)channelsDcc[i][0],tc) * SIMTIME_DBL(genInterval((int)channelsDcc[i][0],tc)) + SIMTIME_DBL(genInterval((int)channelsDcc[i][0],tc)) + SIMTIME_DBL(genGot((int)channelsDcc[i][0],tc));
			nCand++;
		}
	
	
		int randomizer = intuniform(0,nCand);

		selectedChannel = (int)channelsDcc[0][0];
		selch = 0;
		bool success = false;
		if(channelsDcc[0][1] > mCasfTh){
			for(int i = 1; i < nCand; i++){
				if(channelsDcc[i][1] < mCasfTh && success == false){
					success = true;
					selectedChannel = (int)channelsDcc[i][0];
					selch = i;
				}
			}
			if(success == false){
				if(mdcPolicy == 0){
						selch = randomizer;
						selectedChannel = (int)channelsDcc[randomizer][0];
					} else if(mdcPolicy == 1){
						selch = -1;
						selectedChannel = -1;
					} else if(mdcPolicy == 2){
						selch = minCBR(candidateChannels);
						selectedChannel = selch;
					}
			}
		}
			
	}

	return selectedChannel; 
}




void ExampleService::checkTriggeringConditions(const SimTime& T_now)
{
	const SimTime T_elapsed = T_now - mLastExaTimestamp;
	if(T_elapsed >= mGenExa){

		mRequestedCamRate = mCaService->getRequestedCamRate();
		mRequestedExaRate = 1.0 / mGenExa.dbl();
		double availableRate = ((1.0 / genInterval(180,tcPrim)) + (1.0 / genInterval(172,tcAlt)) + (1.0 / genInterval(176,tcAlt))) - mRequestedCamRate;
		bool decisionByRate = mRequestedExaRate < availableRate;

		int selectedChannel = 180; 
		std::vector<int> candidateChannels;
		int allChannels[3];
		allChannels[0] = 180;
		allChannels[1] = 172;
		allChannels[2] = 176;
		double clrLimit = par("clrLimit");
		for(int i = 0; i < 3; i++){
			if(occPolicy == 1){
				if(getCbr(allChannels[i]) < clrLimit) candidateChannels.push_back(allChannels[i]);
			} else {
				candidateChannels.push_back(allChannels[i]);
			}
		}
		

		if (mAliSelection == 0) selectedChannel = loadBalancing(candidateChannels);
		if (mAliSelection == 1) selectedChannel = seqFillCBR(candidateChannels);
		if (mAliSelection == 2) selectedChannel = calis(candidateChannels);
		if (mAliSelection == 3) selectedChannel = casf(candidateChannels);
		if (mAliSelection == 4) selectedChannel = minCBR(candidateChannels);
		if (mAliSelection == 5) selectedChannel = minTRC(candidateChannels);

	
		int ratePol = par("ratePolicy");
		if(ratePol != 0){
			sendExample(T_now, selectedChannel);	
		} else {
			if (decisionByRate) {
				sendExample(T_now, selectedChannel);
			} else {
				genRate = par("genRate");
				mGenExa = std::min(1.0,std::max(genRate,0.001));
				mLastExaTimestamp = T_now;
			}
		}
				
			
		
	}
}

void ExampleService::sendExample(const SimTime& T_now, int selectedChannel)
{
	// use an ITS-AID reserved for testing purposes
		static const vanetza::ItsAid example_its_aid = 16480;

		auto& mco = getFacilities().get_const<MultiChannelPolicy>();
		auto& networks = getFacilities().get_const<NetworkInterfaceTable>();
		
		
		/*
		*/
		if(selectedChannel == 180) {
			seltc = tcPrim;
		} else {
			seltc = tcAlt;
		} 

		countDesired++;
		if(selectedChannel != -1){
			int packetSize = par("messageSize");
			auto network = networks.select(selectedChannel);
			btp::DataRequestB req;
			// use same port number as configured for listening on this channel
			req.destination_port = host_cast(getPortNumber(selectedChannel));
			network = networks.select(selectedChannel);
			auto netifc = network;
			req.gn.transport_type = geonet::TransportType::SHB;
			req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP3));
			if(seltc == 0){
				req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP0));
				} else if(seltc == 1){
					req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP1));
				} else if(seltc == 2){
					req.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
				}
			avgQueuePlace += getQueueOccupancy(selectedChannel,seltc) + 1.0;
			req.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
			req.gn.its_aid = example_its_aid;

			cPacket* packet = new cPacket("Example Service Packet");
			packet->setByteLength(packetSize);
			//std::cout << findHost()->getFullName() << "Transmitting in channel" << selectedChannel << "\n";
					
			genCh[selch] = genCh[selch] + 1;
			// send packet on specific network interface
			request(req, packet, network.get());
		} else {
			genCh[3] = genCh[3] + 1;
		}
		
		myfile7 = fopen("MessageTriggerCLR.csv", "a");
		fprintf(myfile7, "%s,%f,%f,%f,%f,%d\n",findHost()->getFullName(),SIMTIME_DBL(simTime()),getCbr(180),getCbr(172),getCbr(176),selectedChannel);
		fclose(myfile7);

		mLastExaTimestamp = T_now;
		mGenExa = std::min(1.0,std::max(genRate,0.001));
		genRate = par("genRate");	
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
