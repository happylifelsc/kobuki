/**
 * @file /kobuki_driver/include/kobuki_driver/events/event_manager.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 14/04/2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_BUTTON_EVENT_HPP_
#define KOBUKI_BUTTON_EVENT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdint.h>
#include <vector>
#include <ecl/sigslots.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Event Structures
*****************************************************************************/

struct ButtonEvent {
  enum State {
    Released = 0,
    Pressed = 1
  } state;
  enum Button {
    F0,
    F1,
    F2
  } button;
};

/*****************************************************************************
** Interfaces
*****************************************************************************/

class EventManager {
public:
  EventManager() : last_button_state(0) {}

  void init(const std::string &sigslots_namespace);
  void update(const uint8_t &new_button_state);

private:
  uint8_t last_button_state;
  ecl::Signal<const ButtonEvent&> sig_button_event;
};


} // namespace kobuki

#endif /* KOBUKI_BUTTON_EVENT_HPP_ */