/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/application/CaObject.h"
#include "artery/application/CaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>

namespace artery
{

using namespace omnetpp;

/*
OAM BEGIN
Variables para las métricas de evaluación del CA Service
*/
int flag = 1; //OAM Flag en 1 para registrar las distancias y en 0 para no registrar
int headerFlag = 1;
int flagCounter = 1; //
int messageID;
static const int numCars = 4000 + 1;
static const int centralNode = 150; //OAM 103 PARA 300C, 46 PARA NODO ESQUINA;
long double lldm[numCars][2];
double recTime = 0;
int messageCounter = 3;
long double xlima = 98603640;
long double xlimb = 98611880;
long double xlimc = 99128000;
long double xlimd = 99150360;
long double ylim = 5437300;

FILE *myfile; //Registra SentCam.csv
FILE *myfile2; //Registra RecCam.csv
/*
OAM END
*/

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(10);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

SpeedValue_t buildSpeedValue(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = round(v, centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}


Define_Module(CaService)

CaService::CaService() :
		mGenCamMin { 100, SIMTIME_MS },
		mGenCamMax { 1000, SIMTIME_MS },
		mGenCam(mGenCamMax),
		mGenCamLowDynamicsCounter(0),
		mGenCamLowDynamicsLimit(3)
{
}

void CaService::initialize()
{
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCamTimestamp = simTime();

	// first generated CAM shall include the low frequency container
	mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);

	// generation rate boundaries
	mGenCamMin = par("minInterval");
	mGenCamMax = par("maxInterval");
	mGenCam = mGenCamMax;
	tgoExtension = par("applyGot");
	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");

	// look up primary channel for CA
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);

	//OAM Inicializar el array que registra las distancias hacia el nodo que envía un CAM
	nodeName = findHost()->getFullName();
	nodeName = &nodeName[5];
	std::string hostName(nodeName);
	hostName = hostName.substr(0,hostName.size()-1);
	nodeIndex = std::stoi (hostName);
	if (headerFlag == 1){
		myfile = fopen("RecCam.csv", "w");
		fprintf(myfile, "%s, %s, %s, %s, %s, %s\n", "nodeName", "RxStationID", "timestamp", "TxStationID", "messageID", "distance");
		fclose(myfile);
		myfile2 = fopen("SentCam.csv", "w");
		fprintf(myfile2,"%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n", "nodeName", "TxStationID", "messageID", "timestamp", "T_elapsed", "T_GO","x","y", "delayDcc", "T_GenCam", "EarlyReq", "CamType","25m", "75m", "125m", "175m", "225m", "275m", "325m", "375m", "425m", "475m", "525m", "575m", "625m", "675m", "725m", "775m", "825m","875m","925m","975m","1025m","1075m","1125m","1175m","1225m","1275m","1325m","1375m","1425m","1475m");
		fclose(myfile2);
		headerFlag = 0;
	}

}

void CaService::trigger()
{
	Enter_Method("trigger");
	//OAM actualizar la ubicación propia en el LLDM - Begin
	lldm[nodeIndex][0] = round(mVehicleDataProvider->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	lldm[nodeIndex][1] = round(mVehicleDataProvider->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	//OAM End
	//if (nodeIndex >= 200){
		checkTriggeringConditions(simTime());
	//}
}

void CaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");

	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	if (cam && cam->validate()) {
		CaObject obj = visitor.shared_wrapper;
		emit(scSignalCamReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
		//OAM Registrar la distancia del nodo hacia el nodo central - Begin
		long double myX = round(mVehicleDataProvider->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
		long double myY = round(mVehicleDataProvider->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
		//if(SIMTIME_DBL(simTime())>recTime &&(((myX>=xlima && myX<=xlimb)||(myX>=xlimc && myX<=xlimd))&&( myY >= ylim))){
		if(SIMTIME_DBL(simTime())>recTime){
			const vanetza::asn1::Cam& msg = obj.asn1();
			if(msg->header.messageID > 3){
			long double visitorX = msg->cam.camParameters.basicContainer.referencePosition.longitude;
			
			long double visitorY = msg->cam.camParameters.basicContainer.referencePosition.latitude;

			myId = mVehicleDataProvider->station_id();
			long double distance = (sqrt(pow(myX - visitorX, 2) + pow(myY - visitorY, 2)))/100;
			distance = distance * 1.108;
			myfile = fopen("RecCam.csv", "a");
			fprintf(myfile, "%s, %li, %f, %lu, %li, %Lf\n", findHost()->getFullName(), myId, SIMTIME_DBL(simTime()),msg->header.stationID, msg->header.messageID, distance);
			fclose(myfile); 
			}
		}
		//OAM End
	}	
}

void CaService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : mGenCamMin;
	const SimTime T_elapsed = T_now - mLastCamTimestamp;
	//OAM variables Tgo
	const SimTime T_GO = whenWillGateOpen();



	
    mTCam = T_elapsed;
    mTGo = T_GO;

	if(tgoExtension == false){
		if (T_elapsed >= T_GenCamDcc) {
			if (mFixedRate) {
				flagProcedence = 3;
				sendCam(T_now, T_elapsed);
			} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
				flagProcedence = 1;
				sendCam(T_now, T_elapsed);
				T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
				mGenCamLowDynamicsCounter = 0;
			} else if (T_elapsed >= T_GenCam) {
				flagProcedence = 2;
				sendCam(T_now, T_elapsed);
				if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
					T_GenCam = T_GenCamMax;
				}
			}
		}
	} else {
		//BEGIN OAM MODIFIED 2
		//if(T_elapsed >= T_GenCamMin && T_elapsed >= T_GenCamDcc) {
		if(T_elapsed >= T_GenCamDcc) {
			if ((checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) && flagReqCam != 2) {
				if(SIMTIME_DBL(T_GO) >= 0) {
					flagProcedence = 1;
					sendCam(T_now, T_elapsed);
					//mGenCam = std::min(T_elapsed - (mLastCamTimestamp - mTReqCam), T_GenCamMax); /*< if middleware update interval is too long */
					//mGenCam = std::min(T_elapsed, mGenCamMax);
					mGenCamLowDynamicsCounter = 0;
					flagReqCam = 0;
				} else if(flagReqCam == 0){
						mTReqCam = T_now;
						mLastReqPosition = mVehicleDataProvider->position();
						mLastReqSpeed = mVehicleDataProvider->speed();
						mLastReqHeading = mVehicleDataProvider->heading();
						flagReqCam = 1;
					}
			} else if (T_elapsed >= mGenCam && flagReqCam != 1) {
				if (SIMTIME_DBL(T_GO) >= 0) {		
					flagProcedence = 2;
					sendCam(T_now, T_elapsed);
					mLastCamTimestamp = T_now;
					if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
						mGenCam = T_GenCamMax;
					}
					flagReqCam = 0;
				}else if(flagReqCam == 0){
						mTReqCam = T_now;
						mLastReqPosition = mVehicleDataProvider->position();
						mLastReqSpeed = mVehicleDataProvider->speed();
						mLastReqHeading = mVehicleDataProvider->heading();
						flagReqCam = 2;
					}

			}
		}
		//END MODIFIED 2
	}
}

bool CaService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool CaService::checkPositionDelta() const
{
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool CaService::checkSpeedDelta() const
{
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void CaService::sendCam(const SimTime& T_now, const SimTime& T_elapsed)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod);

	mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
	mLastCamTimestamp = T_now;
	if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		addLowFrequencyContainer(cam, par("pathHistoryLength"));
		mLastLowCamTimestamp = T_now;
	}

	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CAM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	/*
		OAM Al enviar un CAM registrar a qué distancia estoy de los receptores - Begin
	*/
	long double myX = round(mVehicleDataProvider->longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	long double myY = round(mVehicleDataProvider->latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	//if(SIMTIME_DBL(simTime())>recTime &&(myX>=xlima && myX<=xlimb)&&( myY >= ylim)){
	if(SIMTIME_DBL(simTime())>recTime ){
		int to25 = -1;
		int to75 = 0;
		int to125 = 0;
		int to175 = 0;
		int to225 = 0;
		int to275 = 0;
		int to325 = 0;
		int to375 = 0;
		int to425 = 0;
		int to475 = 0;
		int to525 = 0;
		int to575 = 0;
		int to625 = 0;
		int to675 = 0;
		int to725 = 0;
		int to775 = 0;
		int to825 = 0;
		int to875 = 0;
		int to925 = 0;
		int to975 = 0;
		int to1025 = 0;
		int to1075 = 0;
		int to1125 = 0;
		int to1175 = 0;
		int to1225 = 0;
		int to1275 = 0;
		int to1325 = 0;
		int to1375 = 0;
		int to1425 = 0;
		int to1475 = 0;
		for (int i = 0; i < numCars; i++){

			long double visitorX = lldm[i][0];
			
			long double visitorY = lldm[i][1];
			long double distance = (sqrt(pow(myX - visitorX, 2) + pow(myY - visitorY, 2)))/100;
			distance = distance * 1.108;
			if (distance >= 0 && distance <= 25) to25++;
			if (distance > 25 && distance <= 75) to75++;
			if (distance > 75 && distance <= 125) to125++;
			if (distance > 125 && distance <= 175) to175++;
			if (distance > 175&& distance <= 225) to225++;
			if (distance > 225&& distance <= 275) to275++;
			if (distance > 275&& distance <= 325) to325++;
			if (distance > 325&& distance <= 375) to375++;
			if (distance > 375&& distance <= 425) to425++;
			if (distance > 425&& distance <= 475) to475++;
			if (distance > 475&& distance <= 525) to525++;
			if (distance > 525&& distance <= 575) to575++;
			if (distance > 575&& distance <= 625) to625++;
			if (distance > 625&& distance <= 675) to675++;
			if (distance > 675&& distance <= 725) to725++;
			if (distance > 725&& distance <= 775) to775++;
			if (distance > 775&& distance <= 825) to825++;
			if (distance > 825&& distance <= 875) to875++;
			if (distance > 875&& distance <= 925) to925++;
			if (distance > 925&& distance <= 975) to975++;
			if (distance > 975&& distance <= 1025) to1025++;
			if (distance > 1025&& distance <= 1075) to1075++;
			if (distance > 1075&& distance <= 1125) to1125++;
			if (distance > 1125&& distance <= 1175) to1175++;
			if (distance > 1175&& distance <= 1225) to1225++;
			if (distance > 1225&& distance <= 1275) to1275++;
			if (distance > 1275&& distance <= 1325) to1325++;
			if (distance > 1325&& distance <= 1375) to1375++;
			if (distance > 1375&& distance <= 1425) to1425++;
			if (distance > 1425&& distance <= 1475) to1475++;
		}
		myId = mVehicleDataProvider->station_id();
		myfile2 = fopen("SentCam.csv","a");
		fprintf(myfile2,"%s, %li, %i, %f, %f, %f, %Lf, %Lf, %f, %f, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", findHost()->getFullName(), myId, messageID, SIMTIME_DBL(T_now), SIMTIME_DBL(mTCam), SIMTIME_DBL(mTGo), myX, myY, delayCamDcc, SIMTIME_DBL(mGenCam),flagReqCam,flagProcedence,to25, to75, to125, to175, to225, to275, to325,to375, to425,to475, to525, to575, to625, to675, to725, to775, to825, to875, to925, to975, to1025, to1075,to1125, to1175, to1225, to1275, to1325, to1375, to1425, to1475);
		fclose(myfile2);
	}
	//OAM End

	CaObject obj(std::move(cam));
	emit(scSignalCamSent, &obj);

	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));

	if(flagReqCam > 0){
		if(flagProcedence == 1){
			mGenCam = std::min(T_elapsed - (T_now - mTReqCam), mGenCamMax);
		}
			mLastCamTimestamp = mTReqCam;
			mLastCamPosition = mLastReqPosition;
			mLastCamSpeed = mLastReqSpeed;
			mLastCamHeading = mLastReqHeading;	
		
	} else if(flagProcedence == 1){
			mGenCam = std::min(T_elapsed, mGenCamMax);
		}
}

SimTime CaService::genCamDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	delayCamDcc = SIMTIME_DBL(dcc) ;
	return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}

SimTime CaService::whenWillGateOpen()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration delay = trc->delay(ca_tx);
	SimTime dccDelay { std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS };
	dccDelay = mTriggerCompensation - dccDelay;
	return dccDelay;
}

vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Cam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	//OAM Identificar cada mensaje con una ID diferente - Begin
	long double myX = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	long double myY = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	//if(SIMTIME_DBL(simTime())>recTime &&(myX>=xlima && myX<=xlimb)&&( myY >= ylim)){
	if(SIMTIME_DBL(simTime())>recTime ){
		header.messageID = messageCounter;
		messageID = messageCounter;
		messageCounter = messageCounter + 1;
	} else {
		header.messageID = ItsPduHeader__messageID_cam;
		messageID = header.messageID;
	}
	//OAM End
	//header.messageID = ItsPduHeader__messageID_cam;
	header.stationID = vdp.station_id();

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = round(vdp.heading(), decidegree);
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = buildSpeedValue(vdp.speed());
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (bvc.yawRate.yawRateValue < -32766 || bvc.yawRate.yawRateValue > 32766) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

	return message;
}

void addLowFrequencyContainer(vanetza::asn1::Cam& message, unsigned pathHistoryLength)
{
	if (pathHistoryLength > 40) {
		EV_WARN << "path history can contain 40 elements at maximum";
		pathHistoryLength = 40;
	}

	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	bvc.vehicleRole = VehicleRole_default;
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);

	for (unsigned i = 0; i < pathHistoryLength; ++i) {
		PathPoint* pathPoint = vanetza::asn1::allocate<PathPoint>();
		pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
		*(pathPoint->pathDeltaTime) = 0;
		pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
		pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
		pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
		ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
	}

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}
}

} // namespace artery
