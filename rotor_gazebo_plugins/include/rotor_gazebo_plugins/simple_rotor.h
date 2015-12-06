#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>

namespace gazebo {

class SimpleRotor: public ModelPlugin {
public:
  SimpleRotor();
  ~SimpleRotor();
  InitializeParams();
  void SendForces();

protected:

};


}
