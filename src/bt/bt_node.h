#ifndef BT_NODE_H
#define BT_NODE_H

#include "../comms/system_state.h"

// Maximum children per composite node — static allocation, no heap
constexpr int BT_MAX_CHILDREN = 8;

// ============================================================================
// Node Status
// ============================================================================

enum class NodeStatus : uint8_t {
    SUCCESS,
    FAILURE,
    RUNNING
};

// ============================================================================
// Abstract Base
// ============================================================================

class BtNode {
public:
    virtual ~BtNode() = default;
    virtual NodeStatus tick(SystemState& state) = 0;
};

// ============================================================================
// Sequence — ticks children in order.
// Returns FAILURE on first child FAILURE.
// Returns RUNNING if any child returns RUNNING.
// Returns SUCCESS only if all children succeed.
// ============================================================================

class Sequence : public BtNode {
public:
    void addChild(BtNode* child) {
        if (count_ < BT_MAX_CHILDREN) {
            children_[count_++] = child;
        }
    }

    NodeStatus tick(SystemState& state) override {
        for (int i = 0; i < count_; ++i) {
            NodeStatus s = children_[i]->tick(state);
            if (s == NodeStatus::FAILURE) return NodeStatus::FAILURE;
            if (s == NodeStatus::RUNNING) return NodeStatus::RUNNING;
        }
        return NodeStatus::SUCCESS;
    }

private:
    BtNode* children_[BT_MAX_CHILDREN] = {};
    int count_ = 0;
};

// ============================================================================
// Selector — ticks children in order.
// Returns SUCCESS on first child SUCCESS.
// Returns RUNNING if any child returns RUNNING.
// Returns FAILURE only if all children fail.
// ============================================================================

class Selector : public BtNode {
public:
    void addChild(BtNode* child) {
        if (count_ < BT_MAX_CHILDREN) {
            children_[count_++] = child;
        }
    }

    NodeStatus tick(SystemState& state) override {
        for (int i = 0; i < count_; ++i) {
            NodeStatus s = children_[i]->tick(state);
            if (s == NodeStatus::SUCCESS) return NodeStatus::SUCCESS;
            if (s == NodeStatus::RUNNING) return NodeStatus::RUNNING;
        }
        return NodeStatus::FAILURE;
    }

private:
    BtNode* children_[BT_MAX_CHILDREN] = {};
    int count_ = 0;
};

// ============================================================================
// Condition — wraps a bool(*)(const SystemState&) function pointer.
// Returns SUCCESS if predicate is true, FAILURE otherwise.
// ============================================================================

class Condition : public BtNode {
public:
    using Predicate = bool (*)(const SystemState&);

    explicit Condition(Predicate fn) : fn_(fn) {}

    NodeStatus tick(SystemState& state) override {
        return fn_(state) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }

private:
    Predicate fn_;
};

// ============================================================================
// Action — wraps a NodeStatus(*)(SystemState&) function pointer.
// Returns whatever the wrapped function returns.
// ============================================================================

class Action : public BtNode {
public:
    using ActionFn = NodeStatus (*)(SystemState&);

    explicit Action(ActionFn fn) : fn_(fn) {}

    NodeStatus tick(SystemState& state) override {
        return fn_(state);
    }

private:
    ActionFn fn_;
};

#endif // BT_NODE_H
