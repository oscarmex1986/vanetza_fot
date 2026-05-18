#include "limeric.hpp"
#include <vanetza/common/runtime.hpp>
#include <cassert>
#include <cmath>
#include <numeric>
#include <iostream> //OAM
#include <stdio.h> //OAM
#include <cstring> //OAM

int instanceCounter = -1; //OAM contador de vehiculos (id del vehiculo si aparecen en el mismo orden del node[id] )
int algorithm = 1; // OAM selector de algoritmo (esta versión ya incluye dualAlpha como parámetro, lo usaremos cuando implementemos LIMERIC)
int headerFlagLimeric = 0;
FILE *fileLimeric;
char fileName[15] = "LimericStd.csv";

namespace vanetza
{
namespace dcc
{

static const Limeric::Parameters limericDefaultParams;

double calculate_dini(){
    double dini; // OAM Delta inicial de los vehículos
    if(instanceCounter > 300){ // OAM esto lo implementamos para el escenario 300+25
        dini = 0.03;
    } else {
        dini = 0.03;
    }
    return dini;
}

Limeric::Limeric(Runtime& rt) : Limeric(rt, limericDefaultParams)
{
}
/*
OAM esta implementación del ETSI Adaptive comienza con la mean de la delta: m_duty_cycle(mean(params.delta_max, params.delta_min)), m_cbr(2)
nosotros asignamos una delta inicial conveniente (0.003 es la delta para 300 vehículos)
*/
Limeric::Limeric(Runtime& rt, const Parameters& params) :
    on_duty_cycle_change(m_duty_cycle_change), m_runtime(rt), m_params(params),
    m_duty_cycle(calculate_dini()), m_cbr(2)
{
    instanceCounter = instanceCounter + 1;
    id = instanceCounter;
    if (m_dual_alpha) std::strcpy(fileName, "LimericDua.csv");
    if (headerFlagLimeric == 0){
        fileLimeric = fopen(fileName, "w");
        fprintf(fileLimeric, "%s,%s,%s,%s,%s,%s\n","nid","ifid","raw_T","scbr","delta","flag");
        headerFlagLimeric = 1;
        fclose(fileLimeric);
    }
    assert(m_cbr.empty());
    schedule();
}

Limeric::~Limeric()
{
    m_runtime.cancel(this);
}

ChannelLoad Limeric::average_cbr() const
{
    if (m_cbr.full()) {
        return 0.5 * mean(m_cbr.begin(), m_cbr.end()) + 0.5 * m_channel_load;
    } else {
        return m_channel_load;
    }
}

void Limeric::update_cbr(ChannelLoad cbr)
{
    const bool full = m_cbr.full();
    m_cbr.push_back(cbr);
    if (!full) {
        m_channel_load = mean(m_cbr.begin(), m_cbr.end());
    }
}

UnitInterval Limeric::calculate_duty_cycle() const
{
    const double cbr_delta = m_params.cbr_target.value() - m_channel_load.value();
    const double cbrMeas = m_channel_load.value();
    double delta_offset = 0.0;
    int flag_delta = 0;
    
    if (cbr_delta > 0.0) {
        delta_offset = std::min(m_params.beta.value() * cbr_delta, m_params.g_plus_max);
    } else {
        delta_offset = std::max(m_params.beta.value() * cbr_delta, m_params.g_minus_max);
    }
    UnitInterval delta = m_params.alpha.complement() * m_duty_cycle + delta_offset;
    delta = std::min(std::max(delta, m_params.delta_min), m_params.delta_max);

    if (m_dual_alpha) {
        if (m_duty_cycle - delta > m_dual_alpha->threshold) {
            flag_delta = 1; //OAM indicar que usamos delta_high
            delta = m_dual_alpha->alternate_alpha.complement() * m_duty_cycle + delta_offset;
            delta = std::min(std::max(delta, m_params.delta_min), m_params.delta_max);
        }
    }
    /*
    OAM imprimimos las deltas y cbr para evaluar convergencia
    */
    
    fileLimeric = fopen(fileName,"a");
    fprintf(fileLimeric, "%i,%i, %li, %f, %f, %i\n", (int)(id/3),id%3, m_runtime.now(), cbrMeas, delta, flag_delta);
    fclose(fileLimeric);

    /* 
    OAM END
    */
    return delta;
}

void Limeric::calculate(Clock::time_point tp)
{
    m_channel_load = average_cbr();
    m_duty_cycle = calculate_duty_cycle(); // uses m_channel_load
    m_duty_cycle_change(this, tp);
    schedule();
}

void Limeric::schedule()
{
    // schedule for next possible modulo 2 * cbr_interval (usually 200ms) time point
    const Clock::duration scheduling_interval = 2 * m_params.cbr_interval;
    Clock::time_point tp = m_runtime.now() + scheduling_interval;
    const Clock::duration scheduling_bias = tp.time_since_epoch() % scheduling_interval;
    if (scheduling_bias > m_params.cbr_interval) {
        tp += scheduling_interval - scheduling_bias;
    } else if (scheduling_bias > Clock::duration::zero()) {
        tp -= scheduling_bias;
    }
    m_runtime.schedule(tp, [this](Clock::time_point tp) {
        this->calculate(tp);
    });
}

void Limeric::configure_dual_alpha(const boost::optional<DualAlphaParameters>& params)
{
    m_dual_alpha = params;
}

} // namespace dcc
} // namespace vanetza
