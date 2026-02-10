#ifndef LAWNMOWER_UTILS_SINGLETON_H
#define LAWNMOWER_UTILS_SINGLETON_H
#include <memory>
#include <mutex>
namespace lawnmower {
template<typename T>
class Singleton {
protected:
    Singleton() = default;
    ~Singleton() = default;
    
    static std::unique_ptr<T> instance_;
    static std::mutex mtx_;
    
public:
    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;
    
    static T& getInstance() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!instance_) {
            instance_.reset(new T());
        }
        return *instance_;
    }
};
template<typename T>
std::unique_ptr<T> Singleton<T>::instance_ = nullptr;
template<typename T>
std::mutex Singleton<T>::mtx_;
} // namespace lawnmower
#endif // LAWNMOWER_UTILS_SINGLETON_H
