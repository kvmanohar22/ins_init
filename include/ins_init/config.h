#ifndef _INS_INIT_CONFIG_H_
#define _INS_INIT_CONFIG_H_

#include "ins_init/global.h"

namespace ins_init
{

class Config {
public:
  static Config& instance();

private:
  Config();
  Config(Config& c);
  void operator = (Config& c); 
};

} // namespace ins_init

#endif

