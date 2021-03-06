#pragma once

#include <string>
#include <utility>

#include "motis/module/module.h"

namespace motis::path {

struct path : public motis::module::module {
  path();
  ~path() override;

  path(path const&) = delete;
  path& operator=(path const&) = delete;

  path(path&&) = delete;
  path& operator=(path&&) = delete;

  void import(motis::module::progress_listener&,
              motis::module::registry&) override;
  void init(motis::module::registry&) override;

  bool import_successful() const override { return import_successful_; }

private:
  void verify_path_database_available() const;

  motis::module::msg_ptr boxes() const;

  motis::module::msg_ptr by_station_seq(motis::module::msg_ptr const&) const;
  motis::module::msg_ptr by_trip_id(motis::module::msg_ptr const&) const;
  motis::module::msg_ptr by_tile_feature(motis::module::msg_ptr const&) const;

  motis::module::msg_ptr get_response(std::string const&,  //
                                      int zoom_level, bool debug_info) const;

  motis::module::msg_ptr path_tiles(motis::module::msg_ptr const&) const;

  struct data;
  std::unique_ptr<data> data_;
  bool import_successful_{false};
};

}  // namespace motis::path
