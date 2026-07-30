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

#ifndef EXAMPLESERVICE_H_
#define EXAMPLESERVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/NetworkInterface.h"
#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

class ExampleService : public ItsG5Service
{
    public:
        ExampleService();
        ~ExampleService();

        void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*, const NetworkInterface&) override;
        void trigger() override;
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
        int getQueueOccupancy(int channel, int tc);
        omnetpp::SimTime genInterval(int channel, int tc);
        omnetpp::SimTime genGot(int channel, int tc);
        double getCbr(int channel);
        

    protected:
        void initialize() override;
        void finish() override;
        void handleMessage(omnetpp::cMessage*) override;

    private:
        void checkTriggeringConditions(const omnetpp::SimTime&);
        omnetpp::cMessage* m_self_msg;
        omnetpp::SimTime mLastExaTimestamp;
        omnetpp::SimTime mGenExa;
        int mAliSelection = 0;
        double mSeqFillTh = 0.50;
        double mCasfTh = 0.050;
        double genRate = exponential(0.025);
        int tcPrim = 3;
        int tcAlt = 0;
        int lastChannel;
        long roundRobin;
        int genCh[4];
        int calis();
        int casf();
        int casfCLR();
        int loadBalancing();
        int seqFillCBR();
        int minCBR();
        int minTRC();
        int selch = 0;
        int seltc = 0;
        double avgQueuePlace = 0.0;
        double queueTrigger = 0.5;
        int dccQueueLength = 0;
        int mdcPolicy = 0;
};

} // namespace artery

#endif /* EXAMPLESERVICE_H_ */
