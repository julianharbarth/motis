#pragma once

#include <ctime>

#include "motis/core/schedule/schedule.h"
#include "motis/loader/loader_options.h"

namespace motis::loader {

struct Schedule;  // NOLINT

schedule_ptr build_graph(Schedule const* serialized, loader_options const&,
                         unsigned progress_offset = 0U);

}  // namespace motis::loader
