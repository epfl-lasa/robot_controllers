#include <iostream>
#include <vector>
#include <control_stack/controllers/passive_ds.hpp>

template <typename U, typename V>
class config_controller{
public:
    config_controller() {};

    template <typename... Args> config_controller(U dim, Args... args) {
        dim_ = dim;
        add(args...);
    }

    template <typename T> void add(T t)
    {
        v_.push_back(t);
    }
    
    template <typename T, typename... Args> void add(T t, Args... args) {
        v_.push_back(t);
        add(args...);
    }

    std::vector<V> v_;
    U dim_;
};

int main(int argc, char const *argv[])
{
    // config_controller<int,double> obj(5,1.2,2.4,3.7);

    // std::cout << obj.dim_ << std::endl;
    
    // for(size_t i = 0; i < 3; i++)
    //     std::cout << obj.v_[i] << std::endl;

    control_stack::controllers::PassiveDS ciao; //(5,1.2,2.3,3.4)

    ciao.SetParams(5, {1., 2.});

    // auto state = mytest.GetParams();

    // std::cout << state.eig_matrix_(0,0) << std::endl;

    return 0;
}
