#include "OLT.h"
#include "DataPacket_m.h"
#include <algorithm>
#include <string>

OLT::OLT() {
    // TODO Auto-generated constructor stub
}

OLT::~OLT() {
    delete buffer;
    delete [] downstream_buffer;
}
void OLT::initialize() {
    /*network parameter*/
    upstream_channel = check_and_cast<cDatarateChannel*>(gate("olt_inout$i")->getIncomingTransmissionChannel());
    downstream_channel = check_and_cast<cDatarateChannel*>(gate("olt_inout$o")->getTransmissionChannel());
    N = getAncestorPar("num_onu");
    MPCPlength = getAncestorPar("MPCP_length");//84*8 bits;

    /* OLT information */
    rtt = par("rtt");
    T_g = par("guard_time");

    buffer = new cPacketQueue("olt_buffer");
    max_length = par("max_length");
    downstream_buffer = new cPacketQueue[N];

    t_scheduled = 0;

    /* initialize polling table */
    for (int llid = 0; llid < N; llid++) {
        ONU_Info* onu_info = new ONU_Info;
        polling_table.push_back(*onu_info);
        polling_table[llid].T_a = MPCPlength/downstream_channel->getDatarate();
        polling_table[llid].t_st = -1;//-1 represents start to send packets and REPORT immediately


        /* QoS Requirements & Traffic Profiles */
        polling_table[llid].D[0] = par("ef_delay_requirement");
        polling_table[llid].D[1] = par("af_delay_requirement");
        polling_table[llid].D[2] = par("be_delay_requirement");

        //polling_table[llid].load = getParentModule()->getSubmodule("onu", llid)->par("load");
        polling_table[llid].P_a = getParentModule()->getSubmodule("onu", llid)->par("P_a");
        polling_table[llid].P_s = getParentModule()->getSubmodule("onu", llid)->par("P_s");
        //polling_table[llid].P_d = getParentModule()->getSubmodule("onu", llid)->par("P_d");

        polling_table[llid].T_o = getParentModule()->getSubmodule("onu", llid)->par("T_o_1");
        polling_table[llid].T_s = 0;
        //polling_table[llid].t_wakeup = simTime();
        polling_table[llid].cycle_start_time = -1;
        polling_table[llid].cycle_number = 0;
        polling_table[llid].cycle_length = 0;
        polling_table[llid].T_extend = T_g;

        GATE* gate = setGATE(llid);
        if(downstream_channel->isBusy()){
            sendDelayed(gate, downstream_channel->getTransmissionFinishTime()-simTime(), "olt_inout$o");
        }
        else{
            send(gate, "olt_inout$o");
        }
    }


}

void OLT::handleMessage(cMessage* msg) {
    switch(msg->getKind())
    {
        case EF:
        case AF:
        case BE:
            {
                DataPacket *packet = check_and_cast<DataPacket*>(msg);

                if(packet->arrivedOn("olt_inout$i"))//upstream packet
                {
                    if(simTime() > simulation.getWarmupPeriod())
                    {   //collect packet bitlength
                        std_packet_bitlength[packet->getKind()].collect(packet->getBitLength());

                        //collect each priority packet delay
                        std_delay[packet->getKind()].collect(simTime()-packet->getTimestamp());
                        std_total_packet_number.collect(1);
                        if(simTime()-packet->getTimestamp()>polling_table[packet->getLlid()].D[0])
                        {    std_violated_packet_number.collect(1); }
                    }
                    delete packet;//buffer->insert(packet);
                }
                else//downstream packet
                {
                    if(downstream_buffer[packet->getArrivalGate()->getIndex()].getByteLength()<10e6)
                    {
                        packet->setTimestamp(simTime());
                        packet->setLlid(packet->getArrivalGate()->getIndex());
                        downstream_buffer[packet->getLlid()].insert(packet);
                    }
                    else{ delete packet;}

                }
                break;
            }
       case R://R for REPORT
            {

                REPORT* report = check_and_cast<REPORT*>(msg);
                int llid = report->getLlid();
                std::vector<ONU_Info>::iterator onu_ = polling_table.begin();

                if(onu_[llid].cycle_start_time==-1)//initialize
                {
                    onu_[llid].cycle_start_time = simTime();

                    onu_[llid].T_a = grantedService(report->getRequested_length() + MPCPlength) / upstream_channel->getDatarate();
                    onu_[llid].t_arriv = report->getT_arriv();

                    if(t_scheduled < simTime() + rtt  + MPCPlength/downstream_channel->getDatarate())
                    {
                        t_scheduled = simTime() + rtt  + MPCPlength/downstream_channel->getDatarate();
                        onu_[llid].T_s = 0;
                    }
                    else
                    {
                       onu_[llid].T_s = t_scheduled.dbl()-simTime().dbl()-rtt-MPCPlength/downstream_channel->getDatarate();
                       if(onu_[llid].T_s>onu_[llid].T_o)
                       {   onu_[llid].T_s = onu_[llid].T_s - onu_[llid].T_o;  }
                       else
                       {   onu_[llid].T_s = 0;   }
                    }
                    onu_[llid].t_st = t_scheduled - rtt/2;
                    t_scheduled = t_scheduled + onu_[llid].T_a + T_g;

                    GATE* gate = setGATE(llid);
                    send(gate, "olt_inout$o");
                }
                else
                {   onu_[llid].cycle_length = simTime().dbl() - onu_[llid].cycle_start_time.dbl();
                    ++onu_[llid].cycle_number;
                    onu_[llid].cycle_start_time = simTime();

                    //collect cycle time
                    if(simTime() > simulation.getWarmupPeriod())
                    {
                        std_cycle_time.collect(onu_[llid].cycle_length);

                        /*collect energy consumption*/
                        double energy_consumption;
                        energy_consumption = onu_[llid].P_a*(onu_[llid].cycle_length -onu_[llid].T_s)+onu_[llid].P_s*onu_[llid].T_s;// includes onu_[llid].T_o
                        //  onu_[next_llid].E.collect(energy_consumption);
                        E_cycle.collect( energy_consumption/onu_[llid].cycle_length);
                        sleep_time.collect(onu_[llid].T_s);
                    }

                    onu_[llid].T_a = grantedService(report->getRequested_length() + MPCPlength) / upstream_channel->getDatarate();
                    onu_[llid].t_arriv = report->getT_arriv();

                    /*Sleep Time Sizing Unit*/
                    if(t_scheduled < simTime() + rtt  + MPCPlength/downstream_channel->getDatarate())
                    {
                         t_scheduled = simTime() + rtt  + MPCPlength/downstream_channel->getDatarate();
                         onu_[llid].T_s = 0;
                    }
                    else
                    {
                        onu_[llid].T_s = t_scheduled.dbl()-simTime().dbl()-rtt-MPCPlength/downstream_channel->getDatarate();
                        if(onu_[llid].T_s>onu_[llid].T_o)
                        {   onu_[llid].T_s = onu_[llid].T_s - onu_[llid].T_o;  }
                        else
                        {   onu_[llid].T_s = 0;   }
                    }

                    //double D_max = simTime().dbl() -  onu_[llid].t_arriv.dbl() + MPCPlength/downstream_channel->getDatarate() + onu_[llid].T_a +rtt;
                    double D_max = simTime().dbl() -  onu_[llid].t_arriv.dbl() + onu_[llid].T_a - MPCPlength/downstream_channel->getDatarate();

                    double T_s_lb =  onu_[llid].D[0] - D_max;



                    if(t_scheduled < simTime()+T_s_lb)
                    {
                        t_scheduled = simTime()+T_s_lb;
                        onu_[llid].T_s = T_s_lb-rtt-MPCPlength/downstream_channel->getDatarate();
                        if(onu_[llid].T_s>onu_[llid].T_o)
                        {   onu_[llid].T_s = onu_[llid].T_s - onu_[llid].T_o;  }
                        else
                        {   onu_[llid].T_s = 0;   }
                    }

                    onu_[llid].t_st = t_scheduled - rtt/2;
                    t_scheduled = t_scheduled + onu_[llid].T_a + T_g;


                    GATE* gate = setGATE(llid);
                    send(gate, "olt_inout$o");

                }


                delete report;
                break;
            }// end of case R
       case DOWN:
       {
           delete msg;
           break;
       }
    }//end of switch
}
GATE* OLT::setGATE(const int i) {
    GATE* gate = new GATE("gate");
    std::vector<ONU_Info>::iterator onu_ = polling_table.begin();
    gate->setLlid(i);
    gate->setGranted_timeslot( onu_[i].T_a);
    gate->setBitLength(MPCPlength);
    gate->setStart_time(onu_[i].t_st);
    gate->setT_s(onu_[i].T_s);
    gate->setKind(G);

    return gate;
}

void OLT::finish()
{
    double bandwidth_utilization = 0;
    for(int i=EF; i<=BE; i++)
    {
        bandwidth_utilization += std_packet_bitlength[i].getSum()/(SIMTIME_DBL(simTime()-simulation.getWarmupPeriod()))/upstream_channel->getDatarate();
    }

    EV << "OLT Bandwidth utilization: " << bandwidth_utilization << endl;
    recordScalar("OLT Bandwidth utilization", bandwidth_utilization);
    recordScalar("OLT EF Bandwidth utilization", std_packet_bitlength[EF].getSum()/(SIMTIME_DBL(simTime()-simulation.getWarmupPeriod()))/upstream_channel->getDatarate());
    recordScalar("OLT AF Bandwidth utilization", std_packet_bitlength[AF].getSum()/(SIMTIME_DBL(simTime()-simulation.getWarmupPeriod()))/upstream_channel->getDatarate());
    recordScalar("OLT BE Bandwidth utilization", std_packet_bitlength[BE].getSum()/(SIMTIME_DBL(simTime()-simulation.getWarmupPeriod()))/upstream_channel->getDatarate());

    EV << "Average cycle time: " << std_cycle_time.getMean() << endl;
    recordScalar("Cycle time mean", std_cycle_time.getMean());
    recordScalar("Packet Delay 0", std_delay[0].getMean());
    recordScalar("Packet Delay 1", std_delay[1].getMean());
    recordScalar("Packet Delay 2", std_delay[2].getMean());
    recordScalar("Max Packet Delay 0", std_delay[0].getMax());
    recordScalar("Energy consumption", E_cycle.getMean()*simTime()/1000);

 //   recordScalar("Idle time Mean", idle_time.getMean());
 //   recordScalar("Idle time Variance", idle_time.getVariance());
    recordScalar("Sleep time Sum", sleep_time.getSum());
    recordScalar("Sleep time Mean", sleep_time.getMean());
    recordScalar("Sleep time Variance", sleep_time.getVariance());
    recordScalar("Violated percentage" , std_violated_packet_number.getSum()/std_total_packet_number.getSum() );


    std::vector<ONU_Info>::iterator onu_ = polling_table.begin();
    for(int i=0; i<N; ++i)
    {
    //  recordScalar("ONU Energy saving percentage", onu_[i].E.getSum()/(onu_[i].P_a*simTime()));
      recordScalar("ONU cycle number", onu_[i].cycle_number);
    }
}
int OLT::grantedService(const unsigned long requested_bitlength)
{
    switch(par("service").longValue())
    {
        double constant;
        case GATED:
            return requested_bitlength;

        case LIMITED:
            return std::min(double(requested_bitlength), max_length);

        case FIXED:
            return max_length;

        case CONSTANT:
            constant = 1518;
            return std::min(requested_bitlength + constant, max_length);

        case LINEAR:
            constant = 1.1;
            return std::min(requested_bitlength * constant, max_length);
    }
    return -1;
}

double OLT::sumT_a(const int i)
{
    std::vector<ONU_Info>::iterator onu_ = polling_table.begin();
    double sum = 0;
    for(int j=0; j<N; j++)
    {   if(j!=i) { sum += onu_[j].T_a;}    }

    return sum;
}


