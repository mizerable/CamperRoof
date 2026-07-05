#include "state_graph.h"
#include "core_logic.h"

StateGraph systemGraph;

StateGraph::StateGraph() {
    buildGraph();
}

void StateGraph::buildGraph() {
    waitNode.stateId = SystemState::STATE_WAIT;
    waitNode.signature = { R, R, R, ANY };
    waitNode.customCondition = [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { 
        return ctx->canEnterWait(pos); 
    };

    liftingNode.stateId = SystemState::STATE_LIFTING;
    liftingNode.signature = { P, R, R, ANY };
    liftingNode.customCondition = nullptr;

    loweringNode.stateId = SystemState::STATE_LOWERING;
    loweringNode.signature = { R, P, R, ANY };
    loweringNode.customCondition = nullptr;

    setNode.stateId = SystemState::STATE_SET;
    setNode.signature = { R, R, P, R };
    setNode.customCondition = nullptr;

    faultNode.stateId = SystemState::STATE_FAULT;
    faultNode.signature = { ANY, ANY, ANY, ANY };
    faultNode.customCondition = [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { 
        return ctx->hasDiverged(pos); 
    };

    bottomedNode.stateId = SystemState::STATE_BOTTOMED;
    bottomedNode.signature = { R, P, R, ANY };
    bottomedNode.customCondition = [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { 
        return ctx->hasBottomedOut(); 
    };

    motor1Node.stateId = SystemState::STATE_MOTOR1;
    motor2Node.stateId = SystemState::STATE_MOTOR2;
    motor3Node.stateId = SystemState::STATE_MOTOR3;
    motor4Node.stateId = SystemState::STATE_MOTOR4;

    // Wiring (Order dictates evaluation priority! FAULT checked first)
    waitNode.possibleNext = { &liftingNode, &loweringNode, &setNode };
    
    liftingNode.possibleNext = { &faultNode, &waitNode };
    loweringNode.possibleNext = { &faultNode, &bottomedNode, &waitNode };
    
    setNode.possibleNext = { &waitNode };
    faultNode.possibleNext = { &waitNode };
    bottomedNode.possibleNext = { &waitNode };

    allNodes = { &waitNode, &liftingNode, &loweringNode, &setNode, &faultNode, &bottomedNode, &motor1Node, &motor2Node, &motor3Node, &motor4Node };
}
