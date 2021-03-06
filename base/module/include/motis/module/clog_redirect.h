#pragma once

#include <fstream>
#include <streambuf>

#include "motis/module/ctx_data.h"
#include "motis/module/dispatcher.h"
#include "motis/module/progress_listener.h"

namespace motis::module {

struct clog_redirect : public std::streambuf {
  clog_redirect(progress_listener&, std::string name,
                char const* log_file_path);

  clog_redirect(clog_redirect const&) = delete;
  clog_redirect(clog_redirect&&) = delete;
  clog_redirect& operator=(clog_redirect const&) = delete;
  clog_redirect& operator=(clog_redirect&&) = delete;

  ~clog_redirect() override;

  int_type overflow(int_type) override;

  static void disable();

private:
  enum class output_state {
    NORMAL,
    MODE_SELECT,
    PERCENT,
    ERR,
    STATUS
  } state_{output_state::NORMAL};
  int percent_{0};
  std::ofstream sink_;
  std::string name_;
  std::streambuf* backup_clog_;
  std::string buf_;
  progress_listener& progress_listener_;
  static bool disabled_;
};

}  // namespace motis::module
