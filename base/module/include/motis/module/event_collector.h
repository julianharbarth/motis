#pragma once

#include <cinttypes>
#include <functional>
#include <map>
#include <string>

#include "motis/module/message.h"
#include "motis/module/progress_listener.h"
#include "motis/module/registry.h"

namespace motis::module {

struct event_collector : std::enable_shared_from_this<event_collector> {
  struct dependency_matcher {
    dependency_matcher() = default;
    dependency_matcher& operator=(dependency_matcher const&) = delete;
    dependency_matcher(dependency_matcher const&) = delete;
    dependency_matcher& operator=(dependency_matcher&&) noexcept = default;
    dependency_matcher(dependency_matcher&&) noexcept = default;
    ~dependency_matcher() = default;

    bool operator<(dependency_matcher const& o) const {
      return name_ < o.name_;
    }
    bool operator==(dependency_matcher const& o) const {
      return name_ == o.name_;
    }

    std::string name_;
    std::function<bool(msg_ptr)> matcher_fn_;
  };
  using dependencies_map_t = std::map<std::string, msg_ptr>;
  using import_op_t = std::function<void(dependencies_map_t const&)>;

  event_collector(progress_listener&, std::string data_dir, std::string name,
                  registry& reg, import_op_t op);

  event_collector* require(std::string const& name,
                           std::function<bool(msg_ptr)>);

private:
  void update_status(motis::import::Status, uint8_t progress = 0U);

  std::string data_dir_;
  std::string module_name_;
  registry& reg_;
  import_op_t op_;
  dependencies_map_t dependencies_;
  std::set<std::string> waiting_for_;
  std::set<dependency_matcher> matchers_;
  progress_listener& progress_listener_;
  bool executed_{false};
};

}  // namespace motis::module
