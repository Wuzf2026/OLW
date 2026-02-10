#ifndef LAWNMOWER_UTILS_STATEMACHINE_H
#define LAWNMOWER_UTILS_STATEMACHINE_H
#include <string>
#include <memory>
#include <vector>
#include <atomic>
namespace lawnmower {
class State;
class StateMachine {
private:
    std::unique_ptr<State> current_state_;
    std::vector<std::unique_ptr<State>> states_;
    
    std::atomic<bool> is_running_;
    std::atomic<bool> is_paused_;
    
public:
    StateMachine();
    ~StateMachine();
    
    void addState(std::unique_ptr<State> state);
    void setInitialState(const std::string& state_name);
    void transitionTo(const std::string& state_name);
    
    void start();
    void stop();
    void pause();
    void resume();
    
    void update();
    
    const State* getCurrentState() const { return current_state_.get(); }
    bool isRunning() const { return is_running_; }
    bool isPaused() const { return is_paused_; }
};
class State {
friend class StateMachine;
private:
    std::string name_;
    StateMachine* state_machine_;
    
protected:
    virtual void enter() = 0;
    virtual void update() = 0;
    virtual void exit() = 0;
    virtual void handleEvent(const std::string& event) = 0;
    
public:
    State(const std::string& name) : name_(name), state_machine_(nullptr) {}
    virtual ~State() {}
    
    const std::string& getName() const { return name_; }
    StateMachine* getStateMachine() const { return state_machine_; }
};
} // namespace lawnmower
#endif // LAWNMOWER_UTILS_STATEMACHINE_H
