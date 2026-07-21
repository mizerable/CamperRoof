#include "state_graph.h"
#include "core_logic.h"

StateGraph systemGraph;

StateGraph::StateGraph() {
    buildGraph();
}

void StateGraph::buildGraph() {
    waitNode.stateId = SystemState::STATE_WAIT;
    liftingNode.stateId = SystemState::STATE_LIFTING;
    loweringNode.stateId = SystemState::STATE_LOWERING;
    setNode.stateId = SystemState::STATE_SET;
    faultNode.stateId = SystemState::STATE_FAULT;
    motor1Node.stateId = SystemState::STATE_MOTOR1;
    motor2Node.stateId = SystemState::STATE_MOTOR2;
    motor3Node.stateId = SystemState::STATE_MOTOR3;
    motor4Node.stateId = SystemState::STATE_MOTOR4;

    // WAIT transitions
    waitNode.transitions.push_back({
        { P, R, R, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return !ctx->isMotorSelRising(); }, 
        &liftingNode, nullptr
    });
    waitNode.transitions.push_back({
        { R, P, R, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return !ctx->isMotorSelRising(); }, 
        &loweringNode, nullptr
    });
    waitNode.transitions.push_back({
        { R, R, P, R, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return !ctx->isMotorSelRising(); }, 
        &setNode, nullptr
    });
    waitNode.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->isMotorSelRising(); },
        &motor1Node, 
        [](CoreLogic* ctx) { ctx->saveStateBeforeMotorSelect(SystemState::STATE_WAIT); }
    });

    // LIFTING transitions
    liftingNode.transitions.push_back({
        { ANY, ANY, ANY, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->hasDiverged(pos); }, 
        &faultNode, 
        nullptr
    });
    liftingNode.transitions.push_back({
        { R, R, R, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->canEnterWait(pos); }, 
        &waitNode, 
        nullptr
    });

    // LOWERING transitions
    loweringNode.transitions.push_back({
        { ANY, ANY, ANY, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->hasDiverged(pos); }, 
        &faultNode, 
        nullptr
    });
    loweringNode.transitions.push_back({
        { R, R, R, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->canEnterWait(pos); }, 
        &waitNode, 
        nullptr
    });

    // SET transitions
    setNode.transitions.push_back({
        { R, R, R, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->canEnterWait(pos); }, 
        &waitNode, 
        nullptr
    });

    // FAULT transitions
    faultNode.transitions.push_back({
        { R, R, R, ANY, ANY }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->canEnterWait(pos) && !ctx->isMotorSelRising(); }, 
        &waitNode, 
        nullptr
    });
    faultNode.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->isMotorSelRising(); },
        &motor1Node, 
        [](CoreLogic* ctx) { ctx->saveStateBeforeMotorSelect(SystemState::STATE_FAULT); }
    });

    // MOTOR JOG transitions
    motor1Node.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->isMotorSelRising(); },
        &motor2Node, 
        nullptr
    });
    motor2Node.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->isMotorSelRising(); },
        &motor3Node, 
        nullptr
    });
    motor3Node.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { return ctx->isMotorSelRising(); },
        &motor4Node, 
        nullptr
    });
    motor4Node.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { 
            return ctx->isMotorSelRising() && ctx->getStateBeforeMotorSelect() == SystemState::STATE_WAIT; 
        },
        &waitNode, 
        nullptr
    });
    motor4Node.transitions.push_back({
        { ANY, ANY, ANY, ANY, P }, 
        [](CoreLogic* ctx, const ButtonState& btn, int32_t pos[4]) { 
            return ctx->isMotorSelRising() && ctx->getStateBeforeMotorSelect() == SystemState::STATE_FAULT; 
        },
        &faultNode, 
        nullptr
    });

    allNodes = { &waitNode, &liftingNode, &loweringNode, &setNode, &faultNode, &motor1Node, &motor2Node, &motor3Node, &motor4Node };
}
