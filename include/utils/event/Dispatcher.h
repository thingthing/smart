#ifndef     UTILS_DISPATCHER_H_
# define    UTILS_DISPATCHER_H_

#include <functional>
#include <map>
#include <vector>
#include <string>
#include <stdexcept>

namespace Utils
{

namespace priv
{

template <typename Function>
struct FunctionTraits
        : public FunctionTraits<decltype(&Function::operator())>
{};

template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType(ClassType::*)(Args...) const>
{
    typedef ReturnType (*pointer)(Args...);
    typedef std::function<ReturnType(Args...)> function;
};

template <typename Lambda>
typename FunctionTraits<Lambda>::pointer makeFunctionPtr(Lambda const &lambda)
{
    return static_cast<typename FunctionTraits<Lambda>::pointer>(lambda);
}

template <typename Lambda>
typename FunctionTraits<Lambda>::function makeFunction(Lambda const &lambda)
{
    return static_cast<typename FunctionTraits<Lambda>::function>(lambda);
}

}  // namespace priv

// Simple design design pattern to dispatch data for a specific key.
// For example you can use it for an asynchronous resources loading:
//
//   Loader loader; // which inherit from Dispatcher
//   loader.RegisterCallback("finish", [] () {
//     std::cout << "Finished!" << std::endl;
//   });
//   loader.RegisterCallback("progress", [] (float progress) {
//     std::cout << "Progress " << progress * 100 << "%" << std::endl;
//   });
//   loader.Start();
//
class Dispatcher
{
public:
    // Default constructor which does compute or initialize anything.
    Dispatcher();

    // Copy constructor.
    Dispatcher(Dispatcher const &other);

    // Asignement operator.
    Dispatcher &operator=(Dispatcher const &other);

    // Delete all registered callbacks.
    virtual ~Dispatcher();

    // Add a callback for the given |key|. For example when
    // `dispatcher.Dispatch("data", myData);` is called, all callbacks
    // registered with `dispatcher.RegisterCallback("data");` will be
    // called.
    template<typename Lambda>
    void registerCallback(const std::string &key, Lambda const &lambda);

    // Remove all registered callbacks for the given |key|.
    void removeCallbacks(const std::string &key);

    // Dispatch according the given |key|. That means that all
    // callbacks registered with |key| will be called.
    template<typename ...Args>
    void dispatch(const std::string &key, Args ...args) const;

private:
    // Simple structure to store informations about the callback.
    struct Callback
    {
        void *function;
        const std::type_info* signature;
    };

    // A map which contains all registered callbacks.
    std::map<std::string, std::vector<Callback>> callbacks_;
};

template<typename Lambda>
void Dispatcher::registerCallback(const std::string &key, const Lambda &lambda)
{
    // Generate the function from the given lambda.
    auto function = new decltype(priv::makeFunction(lambda))
            (priv::makeFunction(lambda));

    callbacks_[key].push_back({static_cast<void*>(function), &typeid(function)});
}

template<typename ...Args>
void Dispatcher::dispatch(const std::string &key, Args ...args) const
{
    if (callbacks_.find(key) == callbacks_.end())
        return;
    for (auto callback : callbacks_.at(key))
    {
        auto function = static_cast<std::function<void(Args...)>*>(callback.function);
        if (false/*Trouver un moyen pour checker uniquement le type et non la constness ou le fait d'avoir une reference*/)
            throw std::invalid_argument("Callback signature doesnt with arguments in dispatcher.");
        (*function)(args...);
    }
}

}  // namespace utils

#endif  // UTILS_DISPATCHER_H_
