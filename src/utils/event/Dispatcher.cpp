#include "utils/event/Dispatcher.h"

namespace Utils
{

Dispatcher::Dispatcher() {}

Dispatcher::~Dispatcher()
{
    for (auto list : callbacks_)
        for (auto cb : list.second)
            delete static_cast<std::function<void()>*>(cb.function);
}

Dispatcher::Dispatcher(Dispatcher const &other)
    : callbacks_(other.callbacks_) {}

Dispatcher &Dispatcher::operator=(Dispatcher const &other)
{
    if (this != &other)
        callbacks_ = other.callbacks_;
    return *this;
}

void Dispatcher::removeCallbacks(const std::string &key)
{
    callbacks_.erase(key);
}

}  // namespace utils
