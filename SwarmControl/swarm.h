#ifndef Swarm_H
#define Swarm_H

class Swarm {

    // will be using AMR class functions to build swarm control functions
    friend class AutoMR;

    bool UpdateSwarm();

    void PANIC_STOP_SWARM();

    void getSwarmData();

};

#endif // Swarm_H