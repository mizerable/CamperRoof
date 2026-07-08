#pragma once

#include <vector>
#include "system_types.h"

#define P 1
#define R 0
#define ANY -1

struct ButtonTrigger {
    int up, down, set, clr;
    bool matches(const ButtonState& btn) const {
        if (up != ANY && ((up == P) != btn.up)) return false;
        if (down != ANY && ((down == P) != btn.down)) return false;
        if (set != ANY && ((set == P) != btn.set)) return false;
        if (clr != ANY && ((clr == P) != btn.clr)) return false;
        return true;
    }
};

class CoreLogic; // Forward declaration

struct StateNode {
    SystemState stateId;
    ButtonTrigger signature;
    bool (*customCondition)(CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]);
    std::vector<StateNode*> possibleNext;
};

class StateGraph {
public:
    StateNode waitNode;
    StateNode liftingNode;
    StateNode loweringNode;
    StateNode setNode;
    StateNode faultNode;
    StateNode bottomedNode;
    StateNode motor1Node;
    StateNode motor2Node;
    StateNode motor3Node;
    StateNode motor4Node;

    std::vector<StateNode*> allNodes;

    StateGraph();
    void buildGraph();
};

extern StateGraph systemGraph;

